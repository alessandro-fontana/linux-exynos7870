// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2015 - 2016 Samsung Electronics Co., Ltd.
 *
 * Authors: Inha Song <ideal.song@samsung.com>
 *          Sylwester Nawrocki <s.nawrocki@samsung.com>
 *
 * Samsung Exynos SoC series Low Power Audio Subsystem driver.
 *
 * This module provides regmap for the Top SFR region and instantiates
 * devices for IP blocks like DMAC, I2S, UART.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/soc/samsung/exynos-regs-pmu.h>
#include <linux/types.h>

/* LPASS Top register definitions */
#define SFR_LPASS_CORE_SW_RESET		0x08
#define  LPASS_SB_SW_RESET		BIT(11)
#define  LPASS_UART_SW_RESET		BIT(10)
#define  LPASS_PCM_SW_RESET		BIT(9)
#define  LPASS_I2S_SW_RESET		BIT(8)
#define  LPASS_WDT1_SW_RESET		BIT(4)
#define  LPASS_WDT0_SW_RESET		BIT(3)
#define  LPASS_TIMER_SW_RESET		BIT(2)
#define  LPASS_MEM_SW_RESET		BIT(1)
#define  LPASS_DMA_SW_RESET		BIT(0)

#define SFR_LPASS_INTR_CA5_MASK		0x48
#define SFR_LPASS_INTR_CPU_MASK		0x58
#define  LPASS_INTR_APM			BIT(9)
#define  LPASS_INTR_MIF			BIT(8)
#define  LPASS_INTR_TIMER		BIT(7)
#define  LPASS_INTR_DMA			BIT(6)
#define  LPASS_INTR_GPIO		BIT(5)
#define  LPASS_INTR_I2S			BIT(4)
#define  LPASS_INTR_PCM			BIT(3)
#define  LPASS_INTR_SLIMBUS		BIT(2)
#define  LPASS_INTR_UART		BIT(1)
#define  LPASS_INTR_SFR			BIT(0)

/*
 * Exynos7870 DISPAUD_CFG system register.  The audio sub-IP software
 * reset bits live in a single word of the DISPAUD sysreg block; the
 * device node maps exactly that word, so the offset here is 0.
 * Bits are active high: setting a bit holds the IP in reset.
 */
#define EXYNOS7870_DISPAUD_CFG		0x00
#define  DISPAUD_CFG_ADMA_SWRST		BIT(3)
#define  DISPAUD_CFG_AMP_SWRST		BIT(2)
#define  DISPAUD_CFG_MI2S_SWRST		BIT(1)
#define  DISPAUD_CFG_MIXER_SWRST	BIT(0)

#define EXYNOS7870_AUD_PLL_RATE		98304000U

struct exynos_lpass;

struct exynos_lpass_variant {
	const struct regmap_config *reg_conf;
	const char *const *clk_names;
	int num_clks;
	int (*enable)(struct exynos_lpass *lpass);
	void (*disable)(struct exynos_lpass *lpass);
};

struct exynos_lpass {
	/* pointer to the LPASS TOP regmap */
	struct regmap *top;
	struct clk_bulk_data *clks;
	const struct exynos_lpass_variant *variant;
};

static void exynos_lpass_core_sw_reset(struct exynos_lpass *lpass, int mask)
{
	unsigned int val = 0;

	regmap_read(lpass->top, SFR_LPASS_CORE_SW_RESET, &val);

	val &= ~mask;
	regmap_write(lpass->top, SFR_LPASS_CORE_SW_RESET, val);

	usleep_range(100, 150);

	val |= mask;
	regmap_write(lpass->top, SFR_LPASS_CORE_SW_RESET, val);
}

static int exynos5433_lpass_enable(struct exynos_lpass *lpass)
{
	int ret;

	ret = clk_bulk_prepare_enable(lpass->variant->num_clks, lpass->clks);
	if (ret)
		return ret;

	/* Unmask SFR, DMA and I2S interrupt */
	regmap_write(lpass->top, SFR_LPASS_INTR_CA5_MASK,
		     LPASS_INTR_SFR | LPASS_INTR_DMA | LPASS_INTR_I2S);

	regmap_write(lpass->top, SFR_LPASS_INTR_CPU_MASK,
		     LPASS_INTR_SFR | LPASS_INTR_DMA | LPASS_INTR_I2S |
		     LPASS_INTR_UART);

	exynos_lpass_core_sw_reset(lpass, LPASS_I2S_SW_RESET);
	exynos_lpass_core_sw_reset(lpass, LPASS_DMA_SW_RESET);
	exynos_lpass_core_sw_reset(lpass, LPASS_MEM_SW_RESET);
	exynos_lpass_core_sw_reset(lpass, LPASS_UART_SW_RESET);

	return 0;
}

static void exynos5433_lpass_disable(struct exynos_lpass *lpass)
{
	/* Mask any unmasked IP interrupt sources */
	regmap_write(lpass->top, SFR_LPASS_INTR_CPU_MASK, 0);
	regmap_write(lpass->top, SFR_LPASS_INTR_CA5_MASK, 0);

	clk_bulk_disable_unprepare(lpass->variant->num_clks, lpass->clks);
}

/* Assert (active high), then release, one sub-IP reset */
static void exynos7870_lpass_sw_reset(struct exynos_lpass *lpass,
				      unsigned int mask)
{
	regmap_update_bits(lpass->top, EXYNOS7870_DISPAUD_CFG, mask, mask);
	usleep_range(100, 150);
	regmap_update_bits(lpass->top, EXYNOS7870_DISPAUD_CFG, mask, 0);
}

static int exynos7870_lpass_enable(struct exynos_lpass *lpass)
{
	int ret;

	ret = clk_bulk_prepare_enable(lpass->variant->num_clks, lpass->clks);
	if (ret)
		return ret;

	/*
	 * clks[] order is fixed by exynos7870_clk_names: [0] APB access
	 * clock, [1] AUD PLL, [2] MI2S root divider.  The divider request
	 * rounds down to the highest rate the PLL provides.
	 */
	clk_set_rate(lpass->clks[1].clk, EXYNOS7870_AUD_PLL_RATE);
	clk_set_rate(lpass->clks[2].clk, EXYNOS7870_AUD_PLL_RATE);

	exynos7870_lpass_sw_reset(lpass, DISPAUD_CFG_MIXER_SWRST);
	exynos7870_lpass_sw_reset(lpass, DISPAUD_CFG_AMP_SWRST);
	exynos7870_lpass_sw_reset(lpass, DISPAUD_CFG_MI2S_SWRST);
	exynos7870_lpass_sw_reset(lpass, DISPAUD_CFG_ADMA_SWRST);

	return 0;
}

static void exynos7870_lpass_disable(struct exynos_lpass *lpass)
{
	clk_bulk_disable_unprepare(lpass->variant->num_clks, lpass->clks);
}

static const struct regmap_config exynos_lpass_reg_conf = {
	.reg_bits	= 32,
	.reg_stride	= 4,
	.val_bits	= 32,
	.max_register	= 0xfc,
};

static const struct regmap_config exynos7870_lpass_reg_conf = {
	.reg_bits	= 32,
	.reg_stride	= 4,
	.val_bits	= 32,
	.max_register	= 0x0,
};

static const char *const exynos5433_clk_names[] = { "sfr0_ctrl" };
static const char *const exynos7870_clk_names[] = {
	"apb_aud", "fout_aud_pll", "dout_mi2s",
};

static const struct exynos_lpass_variant exynos5433_lpass_data = {
	.reg_conf	= &exynos_lpass_reg_conf,
	.clk_names	= exynos5433_clk_names,
	.num_clks	= ARRAY_SIZE(exynos5433_clk_names),
	.enable		= exynos5433_lpass_enable,
	.disable	= exynos5433_lpass_disable,
};

static const struct exynos_lpass_variant exynos7870_lpass_data = {
	.reg_conf	= &exynos7870_lpass_reg_conf,
	.clk_names	= exynos7870_clk_names,
	.num_clks	= ARRAY_SIZE(exynos7870_clk_names),
	.enable		= exynos7870_lpass_enable,
	.disable	= exynos7870_lpass_disable,
};

static void exynos_lpass_disable_lpass(void *data)
{
	struct platform_device *pdev = data;
	struct exynos_lpass *lpass = platform_get_drvdata(pdev);

	pm_runtime_disable(&pdev->dev);
	if (!pm_runtime_status_suspended(&pdev->dev))
		lpass->variant->disable(lpass);
}

static int exynos_lpass_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct exynos_lpass *lpass;
	void __iomem *base_top;
	int i, ret;

	lpass = devm_kzalloc(dev, sizeof(*lpass), GFP_KERNEL);
	if (!lpass)
		return -ENOMEM;

	lpass->variant = of_device_get_match_data(dev);
	if (!lpass->variant)
		return -EINVAL;

	base_top = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base_top))
		return PTR_ERR(base_top);

	lpass->clks = devm_kcalloc(dev, lpass->variant->num_clks,
				   sizeof(*lpass->clks), GFP_KERNEL);
	if (!lpass->clks)
		return -ENOMEM;

	for (i = 0; i < lpass->variant->num_clks; i++)
		lpass->clks[i].id = lpass->variant->clk_names[i];

	ret = devm_clk_bulk_get(dev, lpass->variant->num_clks, lpass->clks);
	if (ret)
		return ret;

	lpass->top = devm_regmap_init_mmio(dev, base_top,
					   lpass->variant->reg_conf);
	if (IS_ERR(lpass->top)) {
		dev_err(dev, "LPASS top regmap initialization failed\n");
		return PTR_ERR(lpass->top);
	}

	platform_set_drvdata(pdev, lpass);
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	ret = lpass->variant->enable(lpass);
	if (ret) {
		pm_runtime_disable(dev);
		return ret;
	}

	ret = devm_add_action_or_reset(dev, exynos_lpass_disable_lpass, pdev);
	if (ret)
		return ret;

	return devm_of_platform_populate(dev);
}

static int __maybe_unused exynos_lpass_suspend(struct device *dev)
{
	struct exynos_lpass *lpass = dev_get_drvdata(dev);

	lpass->variant->disable(lpass);

	return 0;
}

static int __maybe_unused exynos_lpass_resume(struct device *dev)
{
	struct exynos_lpass *lpass = dev_get_drvdata(dev);

	return lpass->variant->enable(lpass);
}

static const struct dev_pm_ops lpass_pm_ops = {
	SET_RUNTIME_PM_OPS(exynos_lpass_suspend, exynos_lpass_resume, NULL)
	SET_LATE_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				     pm_runtime_force_resume)
};

static const struct of_device_id exynos_lpass_of_match[] = {
	{
		.compatible = "samsung,exynos5433-lpass",
		.data = &exynos5433_lpass_data,
	}, {
		.compatible = "samsung,exynos7870-lpass",
		.data = &exynos7870_lpass_data,
	},
	{ },
};
MODULE_DEVICE_TABLE(of, exynos_lpass_of_match);

static struct platform_driver exynos_lpass_driver = {
	.driver	= {
		.name		= "exynos-lpass",
		.pm		= &lpass_pm_ops,
		.of_match_table	= exynos_lpass_of_match,
	},
	.probe	= exynos_lpass_probe,
};
module_platform_driver(exynos_lpass_driver);

MODULE_DESCRIPTION("Samsung Low Power Audio Subsystem driver");
MODULE_LICENSE("GPL v2");
