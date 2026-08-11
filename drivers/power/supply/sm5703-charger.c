// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Silicon Mitus SM5703 charger driver
 *
 * Derived from the vendor driver drivers/battery_v2/sm5703_charger.c:
 *   Copyright (C) 2013 Silicon Mitus Technology Corp.
 *
 * Copyright (C) 2026 The LineageOS Project
 */

#include <linux/bitfield.h>
#include <linux/extcon.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/mfd/sm5703.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/thermal.h>
#include <linux/workqueue.h>

/* Input current limit, VBUSCNTL: 100 mA to 2100 mA in 50 mA steps. */
#define SM5703_INPUT_CURRENT_MIN	100000
#define SM5703_INPUT_CURRENT_MAX	2100000
#define SM5703_INPUT_CURRENT_STEP	50000

/* Fast charge current, CHGCNTL2: 100 mA to 2500 mA in 50 mA steps. */
#define SM5703_CHARGE_CURRENT_MIN	100000
#define SM5703_CHARGE_CURRENT_MAX	2500000
#define SM5703_CHARGE_CURRENT_STEP	50000

/* Battery regulation voltage, CHGCNTL3: 4120 mV to 4430 mV in 10 mV steps. */
#define SM5703_BATREG_MIN		4120000
#define SM5703_BATREG_MAX		4430000
#define SM5703_BATREG_STEP		10000

/* Termination current, CHGCNTL4: 100 mA to 475 mA in 25 mA steps. */
#define SM5703_TERM_CURRENT_MIN		100000
#define SM5703_TERM_CURRENT_MAX		475000
#define SM5703_TERM_CURRENT_STEP	25000

/*
 * The charger cannot tell a wall adapter from a host port on its own:
 * that is the MUIC's job.  Until one tells us otherwise, assume the
 * least a USB source is required to provide.
 */
#define SM5703_INPUT_CURRENT_SDP	500000
#define SM5703_INPUT_CURRENT_CDP	1500000
#define SM5703_INPUT_CURRENT_ACA	1500000

/*
 * Charging stops when the cell leaves its operating range and resumes
 * only once it is this far back inside, so that a cell sitting right on
 * the limit does not toggle the charger on every reading.
 */
#define SM5703_TEMP_HYSTERESIS_MC	5000

/* How often the cell temperature is checked while a supply is present. */
#define SM5703_TEMP_POLL_MS		30000

struct sm5703_charger {
	struct device *dev;
	struct regmap *regmap;
	struct power_supply *psy;
	struct gpio_desc *enable_gpio;
	struct extcon_dev *edev;
	struct notifier_block extcon_nb;
	struct work_struct extcon_work;

	/*
	 * What the board can take, from the device tree, and what we are
	 * drawing right now, which never exceeds it.
	 */
	unsigned int input_current_max;
	unsigned int input_current_limit;
	unsigned int charge_current;
	unsigned int battery_regulation_voltage;
	unsigned int termination_current;

	/*
	 * Whether the chip may charge is decided by two independent
	 * parties: userspace, through charge_behaviour, and the cell
	 * temperature.  The lock covers both and the register writes that
	 * follow from them.
	 */
	struct mutex lock;
	enum power_supply_charge_behaviour behaviour;
	bool temp_blocked;

	/*
	 * Operating range of the cell in millidegrees, from the battery
	 * description; INT_MIN/INT_MAX when it does not give one.
	 */
	int temp_min;
	int temp_max;
	struct thermal_zone_device *battery_tz;
	struct delayed_work temp_work;
};

static int sm5703_charger_set_field(struct sm5703_charger *chg,
				    unsigned int reg, unsigned int mask,
				    unsigned int val, unsigned int min,
				    unsigned int max, unsigned int step)
{
	unsigned int field;

	val = clamp(val, min, max);
	field = (val - min) / step;

	return regmap_update_bits(chg->regmap, reg, mask,
				  field << (ffs(mask) - 1));
}

static int sm5703_charger_get_field(struct sm5703_charger *chg,
				    unsigned int reg, unsigned int mask,
				    unsigned int min, unsigned int step,
				    int *val)
{
	unsigned int tmp;
	int ret;

	ret = regmap_read(chg->regmap, reg, &tmp);
	if (ret)
		return ret;

	*val = min + ((tmp & mask) >> (ffs(mask) - 1)) * step;

	return 0;
}

/*
 * What the attached port is good for.  A dedicated charger gets whatever
 * the board allows; the various host ports get what they are allowed to
 * promise.  Anything unrecognised, or nothing attached at all, falls
 * back to the least a source can offer, as drawing more than a port
 * provides can make it cut the power.
 *
 * Without a MUIC there is nothing to ask, so the board limit is all
 * there is to go on.
 */
static enum power_supply_usb_type
sm5703_charger_usb_type(struct sm5703_charger *chg)
{
	if (!chg->edev)
		return POWER_SUPPLY_USB_TYPE_UNKNOWN;

	if (extcon_get_state(chg->edev, EXTCON_CHG_USB_DCP) > 0)
		return POWER_SUPPLY_USB_TYPE_DCP;

	if (extcon_get_state(chg->edev, EXTCON_CHG_USB_CDP) > 0)
		return POWER_SUPPLY_USB_TYPE_CDP;

	if (extcon_get_state(chg->edev, EXTCON_CHG_USB_ACA) > 0)
		return POWER_SUPPLY_USB_TYPE_ACA;

	if (extcon_get_state(chg->edev, EXTCON_CHG_USB_SDP) > 0)
		return POWER_SUPPLY_USB_TYPE_SDP;

	return POWER_SUPPLY_USB_TYPE_UNKNOWN;
}

static unsigned int sm5703_charger_wanted_current(struct sm5703_charger *chg)
{
	unsigned int wanted;

	if (!chg->edev)
		return chg->input_current_max;

	switch (sm5703_charger_usb_type(chg)) {
	case POWER_SUPPLY_USB_TYPE_DCP:
		wanted = chg->input_current_max;
		break;
	case POWER_SUPPLY_USB_TYPE_CDP:
		wanted = SM5703_INPUT_CURRENT_CDP;
		break;
	case POWER_SUPPLY_USB_TYPE_ACA:
		wanted = SM5703_INPUT_CURRENT_ACA;
		break;
	default:
		wanted = SM5703_INPUT_CURRENT_SDP;
		break;
	}

	return min(wanted, chg->input_current_max);
}

/*
 * Bring the hardware in line with what the attached port allows.
 *
 * The comparison is against the register rather than against what this
 * driver last wrote, so that anything which changed the limit behind our
 * back -- userspace, or the chip resetting itself on a cable event -- is
 * corrected on the next event instead of persisting.
 */
static int sm5703_charger_apply_input_current(struct sm5703_charger *chg)
{
	unsigned int wanted = sm5703_charger_wanted_current(chg);
	int ret, now;

	ret = sm5703_charger_get_field(chg, SM5703_REG_VBUSCNTL,
				       SM5703_VBUSCNTL_VBUSLIMIT,
				       SM5703_INPUT_CURRENT_MIN,
				       SM5703_INPUT_CURRENT_STEP, &now);
	if (ret)
		return ret;

	chg->input_current_limit = wanted;

	if ((unsigned int)now == wanted)
		return 0;

	ret = sm5703_charger_set_field(chg, SM5703_REG_VBUSCNTL,
				       SM5703_VBUSCNTL_VBUSLIMIT, wanted,
				       SM5703_INPUT_CURRENT_MIN,
				       SM5703_INPUT_CURRENT_MAX,
				       SM5703_INPUT_CURRENT_STEP);
	if (ret)
		return ret;

	dev_dbg(chg->dev, "Input current limit now %u uA\n", wanted);

	return 1;
}

static int sm5703_charger_enable(struct sm5703_charger *chg, bool enable)
{
	int ret;

	if (enable) {
		ret = regmap_update_bits(chg->regmap, SM5703_REG_CNTL,
					 SM5703_CNTL_OPERATION_MODE_MASK,
					 SM5703_CNTL_OPERATION_MODE_CHARGING_ON);
		if (ret)
			return ret;
	}

	/* The nCHGEN pin gates the charging path on top of the mode bits. */
	gpiod_set_value_cansleep(chg->enable_gpio, enable);

	if (!enable)
		return regmap_update_bits(chg->regmap, SM5703_REG_CNTL,
					  SM5703_CNTL_OPERATION_MODE_MASK,
					  SM5703_CNTL_OPERATION_MODE_CHARGING_OFF);

	return 0;
}

/* Call with chg->lock held. */
static int sm5703_charger_apply_enable(struct sm5703_charger *chg)
{
	return sm5703_charger_enable(chg,
			chg->behaviour == POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO &&
			!chg->temp_blocked);
}

static bool sm5703_charger_online(struct sm5703_charger *chg)
{
	unsigned int status5;

	if (regmap_read(chg->regmap, SM5703_REG_STATUS5, &status5))
		return false;

	return status5 & SM5703_STATUS5_VBUSOK;
}

/*
 * The cell temperature comes from the thermistor on the SoC ADC, not
 * from this chip, and reaches us as a thermal zone; the fuel gauge
 * reads the same one.  Looked up on first use, as the zone may register
 * after this driver.
 */
static int sm5703_charger_battery_temp(struct sm5703_charger *chg, int *temp)
{
	struct thermal_zone_device *tz;

	if (!chg->battery_tz) {
		tz = thermal_zone_get_zone_by_name("battery-thermal");
		if (IS_ERR(tz))
			return PTR_ERR(tz);
		chg->battery_tz = tz;
	}

	return thermal_zone_get_temp(chg->battery_tz, temp);
}

static bool sm5703_charger_has_temp_range(struct sm5703_charger *chg)
{
	return chg->temp_min != INT_MIN || chg->temp_max != INT_MAX;
}

static void sm5703_charger_check_temp(struct sm5703_charger *chg)
{
	if (sm5703_charger_has_temp_range(chg))
		mod_delayed_work(system_freezable_power_efficient_wq,
				 &chg->temp_work, 0);
}

/*
 * Nothing in the chip watches the cell temperature: the thermal
 * regulation it has is for its own die.  Stop charging outside the
 * operating range of the cell and resume with some hysteresis, the way
 * the vendor driver did with its high and low block temperatures.
 */
static void sm5703_charger_temp_work(struct work_struct *work)
{
	struct sm5703_charger *chg = container_of(to_delayed_work(work),
						  struct sm5703_charger,
						  temp_work);
	bool blocked, changed;
	int temp = 0, ret;

	if (!sm5703_charger_online(chg)) {
		/* Nothing to protect; the next plug starts afresh. */
		blocked = false;
	} else {
		ret = sm5703_charger_battery_temp(chg, &temp);
		if (ret) {
			dev_warn_once(chg->dev,
				      "Cannot read the battery temperature: %d\n",
				      ret);
			blocked = chg->temp_blocked;
		} else if (chg->temp_blocked) {
			blocked = temp > chg->temp_max - SM5703_TEMP_HYSTERESIS_MC ||
				  temp < chg->temp_min + SM5703_TEMP_HYSTERESIS_MC;
		} else {
			blocked = temp > chg->temp_max || temp < chg->temp_min;
		}

		queue_delayed_work(system_freezable_power_efficient_wq,
				   &chg->temp_work,
				   msecs_to_jiffies(SM5703_TEMP_POLL_MS));
	}

	scoped_guard(mutex, &chg->lock) {
		changed = blocked != chg->temp_blocked;
		chg->temp_blocked = blocked;
		ret = changed ? sm5703_charger_apply_enable(chg) : 0;
	}

	if (!changed)
		return;

	if (ret)
		dev_err(chg->dev, "Failed to %s charging: %d\n",
			blocked ? "stop" : "resume", ret);
	else
		dev_info(chg->dev, "Battery at %d mC, charging %s\n", temp,
			 blocked ? "stopped" : "resumed");

	power_supply_changed(chg->psy);
}

/*
 * Push the whole charging configuration into the chip.  This runs at
 * probe and again whenever the charger reports that it has restarted,
 * because the hardware quietly reverts the input current limit to a
 * conservative value when a cable is plugged in.
 */
static int sm5703_charger_configure(struct sm5703_charger *chg)
{
	int ret;

	ret = sm5703_charger_apply_input_current(chg);
	if (ret < 0)
		return ret;

	ret = sm5703_charger_set_field(chg, SM5703_REG_CHGCNTL2,
				       SM5703_CHGCNTL2_FASTCHG,
				       chg->charge_current,
				       SM5703_CHARGE_CURRENT_MIN,
				       SM5703_CHARGE_CURRENT_MAX,
				       SM5703_CHARGE_CURRENT_STEP);
	if (ret)
		return ret;

	ret = sm5703_charger_set_field(chg, SM5703_REG_CHGCNTL3,
				       SM5703_CHGCNTL3_BATREG,
				       chg->battery_regulation_voltage,
				       SM5703_BATREG_MIN, SM5703_BATREG_MAX,
				       SM5703_BATREG_STEP);
	if (ret)
		return ret;

	ret = sm5703_charger_set_field(chg, SM5703_REG_CHGCNTL4,
				       SM5703_CHGCNTL4_TOPOFF,
				       chg->termination_current,
				       SM5703_TERM_CURRENT_MIN,
				       SM5703_TERM_CURRENT_MAX,
				       SM5703_TERM_CURRENT_STEP);
	if (ret)
		return ret;

	/* Stop charging on its own once the termination current is hit. */
	ret = regmap_set_bits(chg->regmap, SM5703_REG_CHGCNTL4,
			      SM5703_CHGCNTL4_AUTOSTOP);
	if (ret)
		return ret;

	/*
	 * AUTOSET lets the chip pick the input current limit from its own
	 * port detection.  It picks 450 mA for anything it is unsure about,
	 * which is most of the time, so drive the limit from here instead.
	 */
	ret = regmap_clear_bits(chg->regmap, SM5703_REG_CNTL,
				SM5703_CNTL_AUTOSET);
	if (ret)
		return ret;

	ret = regmap_update_bits(chg->regmap, SM5703_REG_CHGCNTL6,
				 SM5703_CHGCNTL6_FREQSEL,
				 FIELD_PREP(SM5703_CHGCNTL6_FREQSEL,
					    SM5703_CHGCNTL6_FREQSEL_1P5MHZ));
	if (ret)
		return ret;

	guard(mutex)(&chg->lock);

	return sm5703_charger_apply_enable(chg);
}

static int sm5703_charger_get_status(struct sm5703_charger *chg, int *val)
{
	unsigned int status3, status5;
	int ret;

	ret = regmap_read(chg->regmap, SM5703_REG_STATUS5, &status5);
	if (ret)
		return ret;

	if (!(status5 & SM5703_STATUS5_VBUSOK)) {
		*val = POWER_SUPPLY_STATUS_DISCHARGING;
		return 0;
	}

	ret = regmap_read(chg->regmap, SM5703_REG_STATUS3, &status3);
	if (ret)
		return ret;

	if (status3 & (SM5703_STATUS3_DONE | SM5703_STATUS3_TOPOFF))
		*val = POWER_SUPPLY_STATUS_FULL;
	else if (status3 & SM5703_STATUS3_CHGON)
		*val = POWER_SUPPLY_STATUS_CHARGING;
	else
		*val = POWER_SUPPLY_STATUS_NOT_CHARGING;

	return 0;
}

static int sm5703_charger_get_health(struct sm5703_charger *chg, int *val)
{
	unsigned int status2, status5;
	int ret;

	ret = regmap_read(chg->regmap, SM5703_REG_STATUS5, &status5);
	if (ret)
		return ret;

	if (status5 & SM5703_STATUS5_VBUSOVP) {
		*val = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		return 0;
	}

	ret = regmap_read(chg->regmap, SM5703_REG_STATUS2, &status2);
	if (ret)
		return ret;

	if (status2 & (SM5703_STATUS2_THEMREG | SM5703_STATUS2_THEMSHDN))
		*val = POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (status2 & SM5703_STATUS2_NOBAT)
		*val = POWER_SUPPLY_HEALTH_NO_BATTERY;
	else
		*val = POWER_SUPPLY_HEALTH_GOOD;

	return 0;
}

static int sm5703_charger_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct sm5703_charger *chg = power_supply_get_drvdata(psy);
	unsigned int tmp;
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		return sm5703_charger_get_status(chg, &val->intval);
	case POWER_SUPPLY_PROP_HEALTH:
		return sm5703_charger_get_health(chg, &val->intval);
	case POWER_SUPPLY_PROP_ONLINE:
		ret = regmap_read(chg->regmap, SM5703_REG_STATUS5, &tmp);
		if (ret)
			return ret;
		val->intval = !!(tmp & SM5703_STATUS5_VBUSOK);
		return 0;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		ret = regmap_read(chg->regmap, SM5703_REG_STATUS3, &tmp);
		if (ret)
			return ret;
		if (tmp & SM5703_STATUS3_TOPOFF)
			val->intval = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
		else if (tmp & SM5703_STATUS3_CHGON)
			val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST;
		else
			val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE;
		return 0;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		return sm5703_charger_get_field(chg, SM5703_REG_VBUSCNTL,
						SM5703_VBUSCNTL_VBUSLIMIT,
						SM5703_INPUT_CURRENT_MIN,
						SM5703_INPUT_CURRENT_STEP,
						&val->intval);
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		return sm5703_charger_get_field(chg, SM5703_REG_CHGCNTL2,
						SM5703_CHGCNTL2_FASTCHG,
						SM5703_CHARGE_CURRENT_MIN,
						SM5703_CHARGE_CURRENT_STEP,
						&val->intval);
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		return sm5703_charger_get_field(chg, SM5703_REG_CHGCNTL3,
						SM5703_CHGCNTL3_BATREG,
						SM5703_BATREG_MIN,
						SM5703_BATREG_STEP,
						&val->intval);
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		return sm5703_charger_get_field(chg, SM5703_REG_CHGCNTL4,
						SM5703_CHGCNTL4_TOPOFF,
						SM5703_TERM_CURRENT_MIN,
						SM5703_TERM_CURRENT_STEP,
						&val->intval);
	case POWER_SUPPLY_PROP_USB_TYPE:
		val->intval = sm5703_charger_usb_type(chg);
		return 0;
	case POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR:
		val->intval = chg->behaviour;
		return 0;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = "sm5703";
		return 0;
	default:
		return -EINVAL;
	}
}

static int sm5703_charger_set_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       const union power_supply_propval *val)
{
	struct sm5703_charger *chg = power_supply_get_drvdata(psy);
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		if (val->intval < 0)
			return -EINVAL;

		/* The board limit is a property of the wiring, not a
		 * preference: nothing may ask for more.
		 */
		if ((unsigned int)val->intval > chg->input_current_max)
			return -EINVAL;

		ret = sm5703_charger_set_field(chg, SM5703_REG_VBUSCNTL,
					       SM5703_VBUSCNTL_VBUSLIMIT,
					       val->intval,
					       SM5703_INPUT_CURRENT_MIN,
					       SM5703_INPUT_CURRENT_MAX,
					       SM5703_INPUT_CURRENT_STEP);
		if (ret)
			return ret;

		chg->input_current_limit = val->intval;
		return 0;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		if (val->intval < 0)
			return -EINVAL;

		ret = sm5703_charger_set_field(chg, SM5703_REG_CHGCNTL2,
					       SM5703_CHGCNTL2_FASTCHG,
					       val->intval,
					       SM5703_CHARGE_CURRENT_MIN,
					       SM5703_CHARGE_CURRENT_MAX,
					       SM5703_CHARGE_CURRENT_STEP);
		if (ret)
			return ret;

		chg->charge_current = val->intval;
		return 0;
	case POWER_SUPPLY_PROP_STATUS: {
		/* The older knob for the same thing as charge_behaviour. */
		guard(mutex)(&chg->lock);
		chg->behaviour = val->intval == POWER_SUPPLY_STATUS_DISCHARGING ?
				 POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE :
				 POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO;
		return sm5703_charger_apply_enable(chg);
	}
	case POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR: {
		if (val->intval != POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO &&
		    val->intval != POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE)
			return -EINVAL;

		guard(mutex)(&chg->lock);
		chg->behaviour = val->intval;
		return sm5703_charger_apply_enable(chg);
	}
	default:
		return -EINVAL;
	}
}

static int sm5703_charger_property_is_writeable(struct power_supply *psy,
						enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
	case POWER_SUPPLY_PROP_STATUS:
	case POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR:
		return 1;
	default:
		return 0;
	}
}

static enum power_supply_property sm5703_charger_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT,
	POWER_SUPPLY_PROP_USB_TYPE,
	POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR,
	POWER_SUPPLY_PROP_MODEL_NAME,
};

static const struct power_supply_desc sm5703_charger_desc = {
	.name			= "sm5703-charger",
	.type			= POWER_SUPPLY_TYPE_USB,
	.usb_types		= BIT(POWER_SUPPLY_USB_TYPE_UNKNOWN) |
				  BIT(POWER_SUPPLY_USB_TYPE_SDP) |
				  BIT(POWER_SUPPLY_USB_TYPE_CDP) |
				  BIT(POWER_SUPPLY_USB_TYPE_DCP) |
				  BIT(POWER_SUPPLY_USB_TYPE_ACA),
	.charge_behaviours	= BIT(POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO) |
				  BIT(POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE),
	.properties		= sm5703_charger_props,
	.num_properties		= ARRAY_SIZE(sm5703_charger_props),
	.get_property		= sm5703_charger_get_property,
	.set_property		= sm5703_charger_set_property,
	.property_is_writeable	= sm5703_charger_property_is_writeable,
};

/*
 * charging_enabled: the same state as charge_behaviour, as "1" or "0". The
 * standard attribute lists every behaviour and brackets the current one;
 * Android's charging control HALs (LineageOS's vendor.lineage.health among
 * them) toggle and read a plain boolean node instead.
 */
static ssize_t charging_enabled_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct sm5703_charger *chg = power_supply_get_drvdata(dev_to_psy(dev));

	guard(mutex)(&chg->lock);
	return sysfs_emit(buf, "%d\n",
			  chg->behaviour == POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO);
}

static ssize_t charging_enabled_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct sm5703_charger *chg = power_supply_get_drvdata(dev_to_psy(dev));
	bool enable;
	int ret;

	ret = kstrtobool(buf, &enable);
	if (ret)
		return ret;

	guard(mutex)(&chg->lock);
	chg->behaviour = enable ? POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO :
				  POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE;
	ret = sm5703_charger_apply_enable(chg);
	return ret ?: count;
}
static DEVICE_ATTR_RW(charging_enabled);

static struct attribute *sm5703_charger_attrs[] = {
	&dev_attr_charging_enabled.attr,
	NULL
};
ATTRIBUTE_GROUPS(sm5703_charger);

static void sm5703_charger_extcon_work(struct work_struct *work)
{
	struct sm5703_charger *chg = container_of(work, struct sm5703_charger,
						  extcon_work);
	int ret;

	ret = sm5703_charger_apply_input_current(chg);
	if (ret < 0) {
		dev_err(chg->dev, "Failed to set input current limit: %d\n",
			ret);
		return;
	}

	sm5703_charger_check_temp(chg);
	power_supply_changed(chg->psy);
}

static int sm5703_charger_extcon_notify(struct notifier_block *nb,
					unsigned long event, void *data)
{
	struct sm5703_charger *chg = container_of(nb, struct sm5703_charger,
						  extcon_nb);

	schedule_work(&chg->extcon_work);

	return NOTIFY_OK;
}

/*
 * A plugged in cable makes the chip restart charging, and that restart
 * is where it reapplies its own idea of the input current limit.  Push
 * the configuration back and tell userspace something changed.
 */
static irqreturn_t sm5703_charger_irq(int irq, void *data)
{
	struct sm5703_charger *chg = data;
	int ret;

	ret = sm5703_charger_configure(chg);
	if (ret)
		dev_err(chg->dev, "Failed to reconfigure charger: %d\n", ret);

	sm5703_charger_check_temp(chg);
	power_supply_changed(chg->psy);

	return IRQ_HANDLED;
}

/*
 * The interrupts only drive the automatic side of things: keeping the
 * reported state fresh, and putting the input current limit back after
 * the hardware has reset it.  A charger without them still charges at
 * whatever probe programmed, so a missing interrupt is worth a warning
 * rather than failing the probe and losing that too.
 */
static void sm5703_charger_request_irq(struct sm5703_charger *chg,
				       struct platform_device *pdev,
				       const char *name)
{
	int irq, ret;

	irq = platform_get_irq_byname_optional(pdev, name);
	if (irq < 0) {
		dev_warn(chg->dev,
			 "No %s interrupt, charger state will not update on its own\n",
			 name);
		return;
	}

	ret = devm_request_threaded_irq(chg->dev, irq, NULL,
					sm5703_charger_irq, IRQF_ONESHOT,
					name, chg);
	if (ret)
		dev_warn(chg->dev, "Failed to request %s interrupt: %d\n",
			 name, ret);
}

static void sm5703_charger_cancel_work(void *data)
{
	struct sm5703_charger *chg = data;

	cancel_work_sync(&chg->extcon_work);
}

static void sm5703_charger_cancel_temp_work(void *data)
{
	struct sm5703_charger *chg = data;

	cancel_delayed_work_sync(&chg->temp_work);
}

static int sm5703_charger_probe(struct platform_device *pdev)
{
	struct power_supply_battery_info *info;
	struct power_supply_config cfg = { };
	struct device *dev = &pdev->dev;
	struct sm5703_charger *chg;
	int ret;

	chg = devm_kzalloc(dev, sizeof(*chg), GFP_KERNEL);
	if (!chg)
		return -ENOMEM;

	chg->dev = dev;
	chg->behaviour = POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO;
	INIT_DELAYED_WORK(&chg->temp_work, sm5703_charger_temp_work);

	ret = devm_mutex_init(dev, &chg->lock);
	if (ret)
		return ret;

	chg->regmap = dev_get_regmap(dev->parent, NULL);
	if (!chg->regmap)
		return dev_err_probe(dev, -ENODEV, "Failed to get register map\n");

	/*
	 * Active low in hardware; the GPIO flags in the device tree carry
	 * the polarity, so asserting this descriptor enables charging.
	 */
	chg->enable_gpio = devm_gpiod_get_optional(dev, "enable", GPIOD_ASIS);
	if (IS_ERR(chg->enable_gpio))
		return dev_err_probe(dev, PTR_ERR(chg->enable_gpio),
				     "Cannot get charge enable GPIO\n");

	/*
	 * The device tree value is a ceiling, not a target: it says what
	 * the board can carry, and nothing raises the limit above it.
	 */
	chg->input_current_max = SM5703_INPUT_CURRENT_MAX;
	device_property_read_u32(dev, "input-current-limit-microamp",
				 &chg->input_current_max);
	chg->input_current_max = clamp_t(unsigned int, chg->input_current_max,
					 SM5703_INPUT_CURRENT_MIN,
					 SM5703_INPUT_CURRENT_MAX);

	/*
	 * With no MUIC there is no way to tell a wall adapter from a host
	 * port, and the board limit is all there is to go on.  With one,
	 * the limit follows what it reports, starting from the least a
	 * source can offer until it says otherwise.
	 */
	chg->edev = extcon_get_edev_by_phandle(dev, 0);
	if (IS_ERR(chg->edev)) {
		if (PTR_ERR(chg->edev) == -EPROBE_DEFER)
			return -EPROBE_DEFER;

		chg->edev = NULL;
	}

	cfg.drv_data = chg;
	cfg.fwnode = dev_fwnode(dev);
	cfg.attr_grp = sm5703_charger_groups;

	chg->psy = devm_power_supply_register(dev, &sm5703_charger_desc, &cfg);
	if (IS_ERR(chg->psy))
		return dev_err_probe(dev, PTR_ERR(chg->psy),
				     "Failed to register power supply\n");

	/*
	 * The cell decides how hard it may be charged, so these have no
	 * safe default: without a battery description leave whatever the
	 * bootloader programmed alone rather than guessing.
	 */
	ret = power_supply_get_battery_info(chg->psy, &info);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to get monitored-battery info\n");

	if (info->constant_charge_current_max_ua <= 0 ||
	    info->constant_charge_voltage_max_uv <= 0 ||
	    info->charge_term_current_ua <= 0) {
		power_supply_put_battery_info(chg->psy, info);
		return dev_err_probe(dev, -EINVAL,
				     "Incomplete monitored-battery description\n");
	}

	chg->charge_current = info->constant_charge_current_max_ua;
	chg->battery_regulation_voltage = info->constant_charge_voltage_max_uv;
	chg->termination_current = info->charge_term_current_ua;

	/* operating-range-celsius; the thermal zone reads millidegrees. */
	chg->temp_min = info->temp_min == INT_MIN ? INT_MIN :
			info->temp_min * 1000;
	chg->temp_max = info->temp_max == INT_MAX ? INT_MAX :
			info->temp_max * 1000;

	power_supply_put_battery_info(chg->psy, info);

	ret = devm_add_action_or_reset(dev, sm5703_charger_cancel_temp_work,
				       chg);
	if (ret)
		return ret;

	ret = sm5703_charger_configure(chg);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to configure charger\n");

	sm5703_charger_request_irq(chg, pdev, "CHGON");
	sm5703_charger_request_irq(chg, pdev, "TOPOFF");
	sm5703_charger_request_irq(chg, pdev, "DONE");

	if (chg->edev) {
		INIT_WORK(&chg->extcon_work, sm5703_charger_extcon_work);

		ret = devm_add_action_or_reset(dev, sm5703_charger_cancel_work,
					       chg);
		if (ret)
			return ret;

		chg->extcon_nb.notifier_call = sm5703_charger_extcon_notify;
		ret = devm_extcon_register_notifier_all(dev, chg->edev,
							&chg->extcon_nb);
		if (ret)
			return dev_err_probe(dev, ret,
					     "Failed to register extcon notifier\n");

		/* A cable may already be in when we get here. */
		schedule_work(&chg->extcon_work);
	}

	sm5703_charger_check_temp(chg);

	return 0;
}

static const struct of_device_id sm5703_charger_of_match[] = {
	{ .compatible = "siliconmitus,sm5703-charger" },
	{ }
};
MODULE_DEVICE_TABLE(of, sm5703_charger_of_match);

static struct platform_driver sm5703_charger_driver = {
	.driver = {
		.name		= "sm5703-charger",
		.of_match_table	= sm5703_charger_of_match,
	},
	.probe = sm5703_charger_probe,
};
module_platform_driver(sm5703_charger_driver);

MODULE_DESCRIPTION("Silicon Mitus SM5703 charger driver");
MODULE_LICENSE("GPL");
