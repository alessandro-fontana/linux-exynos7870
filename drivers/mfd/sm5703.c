// SPDX-License-Identifier: GPL-2.0-only
/*
 * Silicon Mitus SM5703 PMIC core driver
 *
 * The SM5703 combines a charger, LDO and buck regulators, a flash LED
 * driver, a MUIC USB switch and a fuel gauge.  The charger, the
 * regulators and the LED driver share the I2C address this driver binds
 * to; the MUIC and the fuel gauge have addresses of their own, often on
 * a different bus, and are handled by separate drivers.
 *
 * Copyright (C) 2022 Markuss Broks <markuss.broks@gmail.com>
 *
 * Copyright (C) 2026 The LineageOS Project
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/ioport.h>
#include <linux/mfd/core.h>
#include <linux/mfd/sm5703.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/regmap.h>

/*
 * The charger watches the interrupts that tell it charging has started
 * or finished.  mfd_add_devices() turns these into real interrupt
 * numbers against the regmap-irq domain passed to it, so the values
 * here are indices into sm5703_irqs below, not virqs.
 */
static const struct resource sm5703_charger_resources[] = {
	DEFINE_RES_IRQ_NAMED(SM5703_IRQ_CHGON, "CHGON"),
	DEFINE_RES_IRQ_NAMED(SM5703_IRQ_TOPOFF, "TOPOFF"),
	DEFINE_RES_IRQ_NAMED(SM5703_IRQ_DONE, "DONE"),
};

static const struct mfd_cell sm5703_cells[] = {
	MFD_CELL_OF("sm5703-charger", sm5703_charger_resources, NULL, 0, 0,
		    "siliconmitus,sm5703-charger"),
	MFD_CELL_OF("sm5703-flash", NULL, NULL, 0, 0,
		    "siliconmitus,sm5703-flash"),
};

static const struct regmap_irq sm5703_irqs[] = {
	REGMAP_IRQ_REG(SM5703_IRQ_AICL, 0, BIT(0)),
	REGMAP_IRQ_REG(SM5703_IRQ_BATOVP, 0, BIT(1)),
	REGMAP_IRQ_REG(SM5703_IRQ_LOWBATT, 0, BIT(2)),
	REGMAP_IRQ_REG(SM5703_IRQ_VBUSLIMIT, 0, BIT(3)),
	REGMAP_IRQ_REG(SM5703_IRQ_DISLIMIT, 0, BIT(4)),
	REGMAP_IRQ_REG(SM5703_IRQ_VSYSOLP, 0, BIT(5)),
	REGMAP_IRQ_REG(SM5703_IRQ_OTGFAIL, 0, BIT(6)),

	REGMAP_IRQ_REG(SM5703_IRQ_THEMREG, 1, BIT(0)),
	REGMAP_IRQ_REG(SM5703_IRQ_THEMSHDN, 1, BIT(1)),
	REGMAP_IRQ_REG(SM5703_IRQ_VSYSNG, 1, BIT(2)),
	REGMAP_IRQ_REG(SM5703_IRQ_VSYSOK, 1, BIT(3)),
	REGMAP_IRQ_REG(SM5703_IRQ_NOBAT, 1, BIT(4)),
	REGMAP_IRQ_REG(SM5703_IRQ_PRETMROFF, 1, BIT(5)),
	REGMAP_IRQ_REG(SM5703_IRQ_FASTTMROFF, 1, BIT(6)),

	REGMAP_IRQ_REG(SM5703_IRQ_CHGON, 2, BIT(0)),
	REGMAP_IRQ_REG(SM5703_IRQ_Q4FULLON, 2, BIT(1)),
	REGMAP_IRQ_REG(SM5703_IRQ_TOPOFF, 2, BIT(2)),
	REGMAP_IRQ_REG(SM5703_IRQ_DONE, 2, BIT(3)),
	REGMAP_IRQ_REG(SM5703_IRQ_CHGRSTF, 2, BIT(4)),

	REGMAP_IRQ_REG(SM5703_IRQ_FLEDSHORT, 3, BIT(0)),
	REGMAP_IRQ_REG(SM5703_IRQ_FLEDOPEN, 3, BIT(1)),
	REGMAP_IRQ_REG(SM5703_IRQ_BOOSTPOK_NG, 3, BIT(2)),
	REGMAP_IRQ_REG(SM5703_IRQ_BOOSTPOK, 3, BIT(3)),
	REGMAP_IRQ_REG(SM5703_IRQ_ABSTMRON, 3, BIT(4)),
};

static const struct regmap_irq_chip sm5703_irq_chip = {
	.name		= "sm5703",
	.status_base	= SM5703_REG_INT1,
	.mask_base	= SM5703_REG_INTMSK1,
	.num_regs	= 4,
	.irqs		= sm5703_irqs,
	.num_irqs	= ARRAY_SIZE(sm5703_irqs),
};

static const struct regmap_config sm5703_regmap_config = {
	.reg_bits	= 8,
	.val_bits	= 8,
	.max_register	= SM5703_REG_MAX,
};

static int sm5703_i2c_probe(struct i2c_client *i2c)
{
	struct device *dev = &i2c->dev;
	struct regmap_irq_chip_data *irq_data;
	struct gpio_desc *reset_gpio;
	struct regmap *regmap;
	unsigned int dev_id;
	int ret;

	regmap = devm_regmap_init_i2c(i2c, &sm5703_regmap_config);
	if (IS_ERR(regmap))
		return dev_err_probe(dev, PTR_ERR(regmap),
				     "Failed to allocate the register map\n");

	/*
	 * The reset line is wired to the chip's MRSTB pin.  Deasserting it
	 * is all that is needed here: the chip keeps its configuration
	 * across a warm boot, and the charger driver reprograms what it
	 * cares about anyway.
	 */
	reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(reset_gpio))
		return dev_err_probe(dev, PTR_ERR(reset_gpio),
				     "Cannot get reset GPIO\n");

	if (reset_gpio)
		msleep(20);

	ret = regmap_read(regmap, SM5703_REG_DEVICE_ID, &dev_id);
	if (ret)
		return dev_err_probe(dev, ret, "Device not found\n");

	dev_dbg(dev, "SM5703 revision 0x%02x\n", dev_id);

	ret = devm_regmap_add_irq_chip(dev, regmap, i2c->irq,
				       IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				       0, &sm5703_irq_chip, &irq_data);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to add IRQ chip\n");

	ret = devm_mfd_add_devices(dev, PLATFORM_DEVID_AUTO, sm5703_cells,
				   ARRAY_SIZE(sm5703_cells), NULL, 0,
				   regmap_irq_get_domain(irq_data));
	if (ret)
		return dev_err_probe(dev, ret, "Failed to add child devices\n");

	return 0;
}

static const struct of_device_id sm5703_of_match[] = {
	{ .compatible = "siliconmitus,sm5703" },
	{ }
};
MODULE_DEVICE_TABLE(of, sm5703_of_match);

static struct i2c_driver sm5703_driver = {
	.driver = {
		.name		= "sm5703",
		.of_match_table	= sm5703_of_match,
	},
	.probe = sm5703_i2c_probe,
};
module_i2c_driver(sm5703_driver);

MODULE_DESCRIPTION("Silicon Mitus SM5703 PMIC core driver");
MODULE_AUTHOR("Markuss Broks <markuss.broks@gmail.com>");
MODULE_LICENSE("GPL");
