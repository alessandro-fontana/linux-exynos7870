// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Silicon Mitus SM5703 fuel gauge driver
 *
 * The SM5703 is a PMIC combining a charger, a MUIC, regulators, a flash
 * LED driver and this voltage-tracking fuel gauge.  The fuel gauge sits
 * on its own I2C address (0x71), usually on a different bus than the
 * rest of the chip, and is fully independent from it: this driver does
 * not need the MFD part to work.
 *
 * All registers are 16 bit wide, little endian, addressed by a single
 * byte.  Voltages, currents and the OCV use a 3.8 fixed point format
 * (three integer bits, eight fractional bits, bit 15 = sign); the SOC
 * and the temperature use an 8.8 format.
 *
 * Derived from the vendor driver drivers/battery_v2/sm5703_fuelgauge.c:
 *   Copyright (C) 2013 Samsung Electronics, Dongik Sin <dongik.sin@samsung.com>
 *   Copyright (C) 2015 Silicon Mitus, SW Jung
 *
 * Copyright (C) 2026 The LineageOS Project
 */

#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/minmax.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/thermal.h>

#define SM5703_FG_REG_DEVICE_ID		0x00
#define SM5703_FG_REG_CNTL		0x01
#define SM5703_FG_REG_INTFG		0x02
#define SM5703_FG_REG_INTFG_MASK	0x03
#define SM5703_FG_REG_STATUS		0x04
#define SM5703_FG_REG_SOC		0x05
#define SM5703_FG_REG_OCV		0x06
#define SM5703_FG_REG_VOLTAGE		0x07
#define SM5703_FG_REG_CURRENT		0x08
#define SM5703_FG_REG_TEMPERATURE	0x09
#define SM5703_FG_REG_V_ALARM		0x0c
#define SM5703_FG_REG_T_ALARM		0x0d
#define SM5703_FG_REG_SOC_ALARM		0x0e
#define SM5703_FG_REG_OP_STATUS		0x10
#define SM5703_FG_REG_TOPOFF_SOC	0x12
#define SM5703_FG_REG_PARAM_CTRL	0x13
#define SM5703_FG_REG_PARAM_RUN_UPDATE	0x14
#define SM5703_FG_REG_VIT_PERIOD	0x1a
#define SM5703_FG_REG_MIX_RATE		0x1b
#define SM5703_FG_REG_MIX_INIT_BLANK	0x1c
#define SM5703_FG_REG_RCE0		0x20
#define SM5703_FG_REG_DTCD		0x23
#define SM5703_FG_REG_RS		0x24
#define SM5703_FG_REG_RS_MIX_FACTOR	0x25
#define SM5703_FG_REG_RS_MAX		0x26
#define SM5703_FG_REG_RS_MIN		0x27
#define SM5703_FG_REG_RS_MAN		0x29
#define SM5703_FG_REG_VOLT_CAL		0x2b
#define SM5703_FG_REG_CURR_CAL		0x2c
#define SM5703_FG_REG_IOCV_MAN		0x2e
#define SM5703_FG_REG_END_V_IDX		0x2f
#define SM5703_FG_REG_IOCV_B_L_MIN	0x30
#define SM5703_FG_REG_IOCV_B_L_MAX	0x37
#define SM5703_FG_REG_IOCV_B_S_MIN	0x40
#define SM5703_FG_REG_IOCV_B_S_MAX	0x43
#define SM5703_FG_REG_MISC		0x90
#define SM5703_FG_REG_TABLE_START	0xa0

/* SM5703_FG_REG_CNTL */
#define SM5703_FG_CNTL_MIX_MODE		BIT(15)
#define SM5703_FG_CNTL_TEMP_MEASURE	BIT(14)
#define SM5703_FG_CNTL_TOPOFF_SOC	BIT(13)
#define SM5703_FG_CNTL_RS_MAN_MODE	BIT(11)
#define SM5703_FG_CNTL_MANUAL_OCV	BIT(10)
#define SM5703_FG_CNTL_SOC_ALARM	BIT(3)
#define SM5703_FG_CNTL_T_H_ALARM	BIT(2)
#define SM5703_FG_CNTL_T_L_ALARM	BIT(1)
#define SM5703_FG_CNTL_V_ALARM		BIT(0)
#define SM5703_FG_CNTL_ALARM_MASK	GENMASK(3, 0)

/* SM5703_FG_REG_MISC */
#define SM5703_FG_MISC_SW_RESET		0x0008

/* SM5703_FG_REG_PARAM_CTRL */
#define SM5703_FG_PARAM_UNLOCK		0x3700
#define SM5703_FG_PARAM_LOCK		0x0000

/* Number of entries of each of the three calibration tables. */
#define SM5703_FG_TABLE_LEN		16
#define SM5703_FG_TABLE_COUNT		3

/*
 * The low byte of OP_STATUS reads back as 0x07 once the calibration
 * parameters have been programmed and the gauge is running.  Anything
 * else means the tables have to be written again.
 */
#define SM5703_FG_OP_STATUS_INITIALISED	0x0007

/* Value of CNTL seen on a gauge that came up from a surge reset. */
#define SM5703_FG_CNTL_SURGE_RESET	0x2008

/* Low voltage alarm level, 3.125 V in the register's 3.8 format. */
#define SM5703_FG_V_ALARM_DEFAULT	0x0320

/*
 * Raw SOC, in per mille, that the gauge reports on a full battery.  The
 * vendor tree carries this per device as "fuelgauge,capacity_max"; every
 * SM5703 device in that tree uses 990, and the value only decides where
 * the reported percentage saturates at 100%.
 */
#define SM5703_FG_SOC_FULL_PERMILLE	990

/* Current, in mA, below which the battery counts as neither charging nor
 * discharging when guessing POWER_SUPPLY_PROP_STATUS.
 */
#define SM5703_FG_CURRENT_IDLE_MA	30

enum sm5703_fg_table {
	SM5703_FG_TABLE_DISCHARGE = 0,
	SM5703_FG_TABLE_CHARGE,
	SM5703_FG_TABLE_SOC,
};

/*
 * Cell specific calibration data.  Everything here comes from the device
 * tree; without it the gauge still reports values, but they are only as
 * good as whatever the bootloader left in the chip.
 */
struct sm5703_fg_params {
	bool valid;
	u32 table[SM5703_FG_TABLE_COUNT][SM5703_FG_TABLE_LEN];
	u32 rce[3];
	u32 dtcd;
	u32 rs[4];		/* rs, mix factor, max, min */
	u32 vit_period;
	u32 mix[2];		/* mix rate, init blank */
	u32 topoff_soc[2];	/* enable, level */
	u32 volt_cal;
	u32 curr_cal;
	u32 temp_std;
	u32 temp_offset;
	u32 temp_offset_cal;
	s32 charge_offset_cal;
	u32 tem_poff[4];	/* normal level/offset, low level/offset */
};

struct sm5703_fg {
	struct device *dev;
	struct regmap *regmap;
	struct power_supply *psy;
	struct power_supply_battery_info *info;
	struct thermal_zone_device *tz;
	struct mutex lock;	/* serialises register sequences and cache */
	struct sm5703_fg_params params;

	unsigned int alert_soc;

	/* Last readings, in mV / mA, used by the IOCV correction. */
	int voltage;
	int prev_voltage;
	int current_ma;
	int ocv;
	int temp;		/* deci degrees Celsius */
	unsigned int iocv_error_count;
};

static int sm5703_fg_read(struct sm5703_fg *fg, u8 reg, u16 *val)
{
	unsigned int tmp;
	int ret;

	ret = regmap_read(fg->regmap, reg, &tmp);
	if (ret)
		return ret;

	*val = tmp;
	return 0;
}

static int sm5703_fg_write(struct sm5703_fg *fg, u8 reg, u16 val)
{
	return regmap_write(fg->regmap, reg, val);
}

/* Decode the 3.8 fixed point format used for voltages and currents. */
static int sm5703_fg_decode_38(u16 raw)
{
	int val;

	val = ((raw & 0x0700) >> 8) * 1000;
	val += ((raw & 0x00ff) * 1000) / 256;

	return val;
}

/* Decode the 8.8 fixed point format used for the SOC and the temperature. */
static int sm5703_fg_decode_88(u16 raw)
{
	int val;

	val = ((raw & 0x7f00) >> 8) * 10;
	val += ((raw & 0x00ff) * 10) / 256;

	return val;
}

static int sm5703_fg_get_voltage(struct sm5703_fg *fg, int *voltage)
{
	u16 raw;
	int ret;

	ret = sm5703_fg_read(fg, SM5703_FG_REG_VOLTAGE, &raw);
	if (ret)
		return ret;

	*voltage = sm5703_fg_decode_38(raw);
	fg->voltage = *voltage;

	return 0;
}

/*
 * The gauge has no averaging register: the vendor driver samples the
 * instantaneous voltage a few times and applies a first order filter.
 */
static int sm5703_fg_get_voltage_avg(struct sm5703_fg *fg, int *voltage)
{
	int ret, i, val, avg = 0;
	u16 raw;

	for (i = 0; i < 5; i++) {
		ret = sm5703_fg_read(fg, SM5703_FG_REG_VOLTAGE, &raw);
		if (ret)
			return ret;

		val = sm5703_fg_decode_38(raw);
		avg = i ? (val / 2 + avg / 2) : val;
	}

	*voltage = avg;

	return 0;
}

static int sm5703_fg_get_ocv(struct sm5703_fg *fg, int *ocv)
{
	u16 raw;
	int ret;

	ret = sm5703_fg_read(fg, SM5703_FG_REG_OCV, &raw);
	if (ret)
		return ret;

	*ocv = sm5703_fg_decode_38(raw);
	fg->ocv = *ocv;

	return 0;
}

/* Positive current means the battery is being charged. */
static int sm5703_fg_get_current(struct sm5703_fg *fg, int *curr)
{
	u16 raw;
	int ret;

	ret = sm5703_fg_read(fg, SM5703_FG_REG_CURRENT, &raw);
	if (ret)
		return ret;

	*curr = sm5703_fg_decode_38(raw);
	if (raw & BIT(15))
		*curr = -*curr;

	fg->current_ma = *curr;

	return 0;
}

static int sm5703_fg_get_current_avg(struct sm5703_fg *fg, int *curr)
{
	int ret, i, val, avg = 0;
	u16 raw;

	for (i = 0; i < 5; i++) {
		ret = sm5703_fg_read(fg, SM5703_FG_REG_CURRENT, &raw);
		if (ret)
			return ret;

		val = sm5703_fg_decode_38(raw);
		if (raw & BIT(15))
			val = -val;

		avg = i ? (val / 2 + avg / 2) : val;
	}

	*curr = avg;

	return 0;
}

/*
 * Temperature of the gauge's own die.  This is not the temperature of
 * the cell: the die sits next to the charging path and reads well above
 * the pack.  It is used for the current gain compensation, which is what
 * the chip measures it for.
 */
static int sm5703_fg_get_die_temp(struct sm5703_fg *fg, int *temp)
{
	u16 raw;
	int ret;

	ret = sm5703_fg_read(fg, SM5703_FG_REG_TEMPERATURE, &raw);
	if (ret)
		return ret;

	*temp = sm5703_fg_decode_88(raw);
	if (raw & BIT(15))
		*temp = -*temp;

	fg->temp = *temp;

	return 0;
}

/*
 * Temperature of the cell, from the pack thermistor.  It hangs off the
 * SoC ADC rather than off this chip, so it reaches us as a thermal zone.
 * Without one there is nothing to report: the die temperature is not a
 * stand-in for the battery temperature.
 */
static int sm5703_fg_get_temp(struct sm5703_fg *fg, int *temp)
{
	int ret, millicelsius;

	/*
	 * Looked up on first use rather than at probe: the thermal zone
	 * may well register after this driver, and deferring the probe to
	 * wait for one would hang a board that simply has no thermistor.
	 */
	if (!fg->tz) {
		fg->tz = thermal_zone_get_zone_by_name("battery-thermal");
		if (IS_ERR(fg->tz)) {
			fg->tz = NULL;
			return -ENODATA;
		}
	}

	ret = thermal_zone_get_temp(fg->tz, &millicelsius);
	if (ret)
		return ret;

	/* The thermal framework works in millidegrees, power supply in
	 * tenths of a degree.
	 */
	*temp = millicelsius / 100;

	return 0;
}

/*
 * Average a run of initial OCV samples, dropping the highest and the
 * lowest one, and report how far apart the extremes were.
 */
static int sm5703_fg_average_iocv(struct sm5703_fg *fg, u8 first, u8 last,
				  int *avg, int *spread)
{
	int lo = 0, hi = 0, sum = 0;
	int ret, i, val;
	u16 raw;

	for (i = first; i <= last; i++) {
		ret = sm5703_fg_read(fg, i, &raw);
		if (ret)
			return ret;

		val = raw;
		if (i == first) {
			lo = val;
			hi = val;
			sum = val;
			continue;
		}

		hi = max(hi, val);
		lo = min(lo, val);
		sum += val;
	}

	sum -= hi + lo;
	*avg = sum / (last - first - 1);
	if (spread)
		*spread = hi - lo;

	return 0;
}

/*
 * Work out the OCV the gauge should start from, out of the samples it
 * collected at power up.  Two sets are kept, a long one and a short one;
 * the short one is only valid when both status bits in END_V_IDX are set.
 */
static int sm5703_fg_calculate_iocv(struct sm5703_fg *fg, u16 *iocv)
{
	int l_avg, s_avg = 0, l_spread;
	int ret;
	u16 raw;

	ret = sm5703_fg_average_iocv(fg, SM5703_FG_REG_IOCV_B_L_MIN,
				     SM5703_FG_REG_IOCV_B_L_MAX,
				     &l_avg, &l_spread);
	if (ret)
		return ret;

	ret = sm5703_fg_read(fg, SM5703_FG_REG_END_V_IDX, &raw);
	if (ret)
		return ret;

	if ((raw & 0x0030) == 0x0030) {
		ret = sm5703_fg_average_iocv(fg, SM5703_FG_REG_IOCV_B_S_MIN,
					     SM5703_FG_REG_IOCV_B_S_MAX,
					     &s_avg, NULL);
		if (ret)
			return ret;
	}

	/* Prefer the long average when the two disagree, or when there is
	 * no short one at all.
	 */
	if ((abs(l_avg - s_avg) > 0x29 && l_spread < 0xcc) || !s_avg)
		*iocv = l_avg;
	else
		*iocv = s_avg;

	return 0;
}

/*
 * Program the cell parameters.  Called at probe when the gauge comes up
 * uninitialised, and again if it is found to have surge reset itself.
 * @manual_ocv selects whether the initial OCV is taken from the last
 * reading (after a reset, when the gauge was already tracking) or
 * recomputed from the power up samples.
 */
static int sm5703_fg_program_params(struct sm5703_fg *fg, bool manual_ocv)
{
	struct sm5703_fg_params *p = &fg->params;
	unsigned int topoff_soc;
	unsigned int i, j;
	u16 val, cntl, iocv;
	u8 reg;
	int ret;

	if (!p->valid)
		return -ENODATA;

	dev_dbg(fg->dev, "programming battery parameters\n");

	ret = sm5703_fg_write(fg, SM5703_FG_REG_PARAM_CTRL,
			      SM5703_FG_PARAM_UNLOCK);
	if (ret)
		return ret;

	for (i = 0; i < ARRAY_SIZE(p->rce); i++) {
		ret = sm5703_fg_write(fg, SM5703_FG_REG_RCE0 + i, p->rce[i]);
		if (ret)
			return ret;
	}

	ret = sm5703_fg_write(fg, SM5703_FG_REG_DTCD, p->dtcd);
	if (ret)
		return ret;

	ret = sm5703_fg_write(fg, SM5703_FG_REG_RS, p->rs[0]);
	if (ret)
		return ret;

	ret = sm5703_fg_write(fg, SM5703_FG_REG_VIT_PERIOD, p->vit_period);
	if (ret)
		return ret;

	/* Unlock again, this time telling the gauge how long the tables are. */
	ret = sm5703_fg_write(fg, SM5703_FG_REG_PARAM_CTRL,
			      SM5703_FG_PARAM_UNLOCK | (SM5703_FG_TABLE_LEN - 1));
	if (ret)
		return ret;

	for (i = 0; i < SM5703_FG_TABLE_COUNT; i++) {
		reg = SM5703_FG_REG_TABLE_START + (i << 4);

		for (j = 0; j < SM5703_FG_TABLE_LEN; j++) {
			ret = sm5703_fg_write(fg, reg + j, p->table[i][j]);
			if (ret)
				return ret;

			/* The table registers need a moment to settle, and
			 * occasionally swallow a write.  Read back and retry
			 * once, as the vendor driver does.
			 */
			usleep_range(10000, 11000);

			ret = sm5703_fg_read(fg, reg + j, &val);
			if (ret)
				return ret;

			if (val != p->table[i][j]) {
				dev_dbg(fg->dev,
					"table %u entry %u readback 0x%04x, retrying\n",
					i, j, val);
				ret = sm5703_fg_write(fg, reg + j, p->table[i][j]);
				if (ret)
					return ret;
			}
		}
	}

	ret = sm5703_fg_write(fg, SM5703_FG_REG_RS_MIX_FACTOR, p->rs[1]);
	if (!ret)
		ret = sm5703_fg_write(fg, SM5703_FG_REG_RS_MAX, p->rs[2]);
	if (!ret)
		ret = sm5703_fg_write(fg, SM5703_FG_REG_RS_MIN, p->rs[3]);
	if (!ret)
		ret = sm5703_fg_write(fg, SM5703_FG_REG_MIX_RATE, p->mix[0]);
	if (!ret)
		ret = sm5703_fg_write(fg, SM5703_FG_REG_MIX_INIT_BLANK, p->mix[1]);
	if (!ret)
		ret = sm5703_fg_write(fg, SM5703_FG_REG_VOLT_CAL, p->volt_cal);
	if (!ret)
		ret = sm5703_fg_write(fg, SM5703_FG_REG_CURR_CAL, p->curr_cal);
	if (ret)
		return ret;

	/* Early revisions number the top off levels differently. */
	topoff_soc = p->topoff_soc[1];
	ret = sm5703_fg_read(fg, SM5703_FG_REG_DEVICE_ID, &val);
	if (ret)
		return ret;

	if (val < 3) {
		if (topoff_soc >= 5)
			topoff_soc = 5;
		else if (topoff_soc >= 3)
			topoff_soc -= 3;
		else
			topoff_soc += 5;
	}

	ret = sm5703_fg_write(fg, SM5703_FG_REG_TOPOFF_SOC, topoff_soc);
	if (ret)
		return ret;

	ret = sm5703_fg_read(fg, SM5703_FG_REG_CNTL, &cntl);
	if (ret)
		return ret;

	cntl &= ~SM5703_FG_CNTL_TOPOFF_SOC;
	cntl |= SM5703_FG_CNTL_MIX_MODE | SM5703_FG_CNTL_TEMP_MEASURE;
	if (p->topoff_soc[0])
		cntl |= SM5703_FG_CNTL_TOPOFF_SOC;
	if (manual_ocv)
		cntl |= SM5703_FG_CNTL_MANUAL_OCV;

	ret = sm5703_fg_write(fg, SM5703_FG_REG_CNTL, cntl);
	if (ret)
		return ret;

	ret = sm5703_fg_write(fg, SM5703_FG_REG_PARAM_CTRL,
			      SM5703_FG_PARAM_LOCK | (SM5703_FG_TABLE_LEN - 1));
	if (ret)
		return ret;

	if (manual_ocv) {
		/* IOCV_MAN and the calibration tables share a format whose
		 * LSB is 1/2.048 mV, so a value in mV converts as
		 * (mV << 8) / 125.  0x2400 is 4500 mV, for a cross check.
		 */
		iocv = (fg->ocv << 8) / 125;
	} else {
		ret = sm5703_fg_calculate_iocv(fg, &iocv);
		if (ret)
			return ret;
	}

	return sm5703_fg_write(fg, SM5703_FG_REG_IOCV_MAN, iocv);
}

/*
 * The gauge normally tracks the internal resistance on its own.  When
 * the measured OCV drifts away from the measured terminal voltage while
 * barely any current flows, or when the pack is close to the cut off
 * voltage, the vendor driver pins Rs to the calibrated value instead.
 * Not doing this makes the SOC jump around at the bottom of the charge.
 */
static int sm5703_fg_check_iocv(struct sm5703_fg *fg)
{
	struct sm5703_fg_params *p = &fg->params;
	bool charging = fg->current_ma > 0;
	int poff, poff_offset;
	bool manual = false;
	u16 cntl, rs;
	int ret;

	if (!p->valid)
		return 0;

	if (abs(fg->current_ma) < 40 || (charging && abs(fg->current_ma) < 100)) {
		if (abs(fg->ocv - fg->voltage) > 30)
			fg->iocv_error_count++;

		fg->iocv_error_count = min(fg->iocv_error_count, 6u);
	} else {
		fg->iocv_error_count = 0;
	}

	if (fg->iocv_error_count > 5) {
		/* A moving voltage means the gauge is tracking after all. */
		if (abs(fg->prev_voltage - fg->voltage) > 15) {
			fg->iocv_error_count = 0;
		} else {
			manual = true;
			rs = p->rs[0];
		}
	} else if (!charging) {
		/* Below 15 degrees the pack needs the lower thresholds. */
		if (fg->temp / 10 > 15) {
			poff = p->tem_poff[0];
			poff_offset = p->tem_poff[1];
		} else {
			poff = p->tem_poff[2];
			poff_offset = p->tem_poff[3];
		}

		if (fg->prev_voltage < poff && fg->voltage < poff) {
			manual = true;
			rs = p->rs[0];

			if (fg->prev_voltage < poff - poff_offset &&
			    fg->voltage < poff - poff_offset)
				rs = p->rs[0] >> 1;
		}
	}

	if (manual) {
		ret = sm5703_fg_write(fg, SM5703_FG_REG_RS_MAN, rs);
		if (!ret)
			ret = sm5703_fg_write(fg, SM5703_FG_REG_PARAM_RUN_UPDATE, 0);
		if (!ret)
			ret = sm5703_fg_write(fg, SM5703_FG_REG_PARAM_RUN_UPDATE, 1);
		if (ret)
			return ret;
	}

	ret = sm5703_fg_read(fg, SM5703_FG_REG_CNTL, &cntl);
	if (ret)
		return ret;

	cntl |= SM5703_FG_CNTL_MIX_MODE;
	if (manual)
		cntl |= SM5703_FG_CNTL_RS_MAN_MODE;
	else
		cntl &= ~SM5703_FG_CNTL_RS_MAN_MODE;

	ret = sm5703_fg_write(fg, SM5703_FG_REG_CNTL, cntl);
	if (ret)
		return ret;

	fg->prev_voltage = fg->voltage;

	return 0;
}

/* Returns the raw state of charge in per mille. */
static int sm5703_fg_get_soc_raw(struct sm5703_fg *fg, int *soc)
{
	struct sm5703_fg_params *p = &fg->params;
	int ret, dummy, curr_cal, temp_cal;
	u16 raw, cntl;

	/* Refresh the values the IOCV correction works on. */
	ret = sm5703_fg_get_voltage(fg, &dummy);
	if (!ret)
		ret = sm5703_fg_get_current(fg, &dummy);
	if (!ret)
		ret = sm5703_fg_get_ocv(fg, &dummy);
	if (!ret)
		ret = sm5703_fg_get_die_temp(fg, &dummy);
	if (ret)
		return ret;

	ret = sm5703_fg_check_iocv(fg);
	if (ret)
		return ret;

	/* A gauge that surge reset itself reports a default control word and
	 * an uninitialised operating status.  Put the parameters back.
	 */
	ret = sm5703_fg_read(fg, SM5703_FG_REG_OP_STATUS, &raw);
	if (ret)
		return ret;

	if (p->valid && (raw & 0x00ff) != SM5703_FG_OP_STATUS_INITIALISED) {
		ret = sm5703_fg_read(fg, SM5703_FG_REG_CNTL, &cntl);
		if (ret)
			return ret;

		if (cntl == SM5703_FG_CNTL_SURGE_RESET) {
			dev_warn(fg->dev, "gauge reset itself, reprogramming\n");

			ret = sm5703_fg_write(fg, SM5703_FG_REG_MISC,
					      SM5703_FG_MISC_SW_RESET);
			if (ret)
				return ret;

			msleep(200);

			ret = sm5703_fg_program_params(fg, true);
			if (ret)
				return ret;
		}
	}

	/*
	 * Compensate the current gain for the die temperature.  The vendor
	 * driver adds a second correction below 25 degrees based on the pack
	 * thermistor, which lives outside this chip; it is left out here.
	 */
	if (p->valid) {
		curr_cal = p->curr_cal;
		if (fg->current_ma > 0)
			curr_cal += p->charge_offset_cal * 256;

		if (p->temp_offset) {
			temp_cal = (int)p->temp_std - fg->temp / 10;
			temp_cal /= (int)p->temp_offset;
			temp_cal *= (int)p->temp_offset_cal;
			curr_cal += temp_cal * 256;
		}

		ret = sm5703_fg_write(fg, SM5703_FG_REG_CURR_CAL, curr_cal);
		if (ret)
			return ret;
	}

	ret = sm5703_fg_read(fg, SM5703_FG_REG_SOC, &raw);
	if (ret)
		return ret;

	*soc = sm5703_fg_decode_88(raw);

	return 0;
}

static int sm5703_fg_get_capacity(struct sm5703_fg *fg, int *capacity)
{
	int ret, soc;

	ret = sm5703_fg_get_soc_raw(fg, &soc);
	if (ret)
		return ret;

	/* Scale so that a full pack reads 100%, then round to a percent. */
	soc = soc * 1000 / SM5703_FG_SOC_FULL_PERMILLE;
	*capacity = clamp(soc / 10, 0, 100);

	return 0;
}

static int sm5703_fg_set_alert_soc(struct sm5703_fg *fg, unsigned int soc)
{
	u16 cntl;
	int ret;

	ret = sm5703_fg_write(fg, SM5703_FG_REG_SOC_ALARM, soc << 8);
	if (!ret)
		ret = sm5703_fg_write(fg, SM5703_FG_REG_PARAM_RUN_UPDATE, 0);
	if (!ret)
		ret = sm5703_fg_write(fg, SM5703_FG_REG_PARAM_RUN_UPDATE, 1);
	if (ret)
		return ret;

	ret = sm5703_fg_read(fg, SM5703_FG_REG_CNTL, &cntl);
	if (ret)
		return ret;

	cntl &= ~SM5703_FG_CNTL_ALARM_MASK;
	cntl |= SM5703_FG_CNTL_SOC_ALARM | SM5703_FG_CNTL_V_ALARM;

	ret = sm5703_fg_write(fg, SM5703_FG_REG_CNTL, cntl);
	if (ret)
		return ret;

	fg->alert_soc = soc;

	return 0;
}

static int sm5703_fg_alert_init(struct sm5703_fg *fg)
{
	u16 dummy;
	int ret;

	/* Reading these two clears anything left pending. */
	ret = sm5703_fg_read(fg, SM5703_FG_REG_INTFG, &dummy);
	if (!ret)
		ret = sm5703_fg_read(fg, SM5703_FG_REG_STATUS, &dummy);
	if (!ret)
		ret = sm5703_fg_write(fg, SM5703_FG_REG_INTFG_MASK, 0);
	if (!ret)
		ret = sm5703_fg_write(fg, SM5703_FG_REG_V_ALARM,
				      SM5703_FG_V_ALARM_DEFAULT);
	if (ret)
		return ret;

	return sm5703_fg_set_alert_soc(fg, fg->alert_soc);
}

static irqreturn_t sm5703_fg_irq_thread(int irq, void *data)
{
	struct sm5703_fg *fg = data;
	u16 status;
	int ret;

	mutex_lock(&fg->lock);
	ret = sm5703_fg_read(fg, SM5703_FG_REG_INTFG, &status);
	mutex_unlock(&fg->lock);

	if (ret) {
		dev_err(fg->dev, "failed to read interrupt status: %d\n", ret);
		return IRQ_NONE;
	}

	dev_dbg(fg->dev, "fuel alert, status 0x%04x\n", status);
	power_supply_changed(fg->psy);

	return IRQ_HANDLED;
}

static int sm5703_fg_get_status(struct sm5703_fg *fg, int *status)
{
	int ret, curr, capacity;

	ret = sm5703_fg_get_current(fg, &curr);
	if (ret)
		return ret;

	/*
	 * Without the charger driver the sign of the current is all there
	 * is to go on.  This is a guess, and userspace treats it as one:
	 * it is replaced as soon as a supply that really knows shows up.
	 */
	if (curr > SM5703_FG_CURRENT_IDLE_MA) {
		*status = POWER_SUPPLY_STATUS_CHARGING;
		return 0;
	}

	if (curr < -SM5703_FG_CURRENT_IDLE_MA) {
		*status = POWER_SUPPLY_STATUS_DISCHARGING;
		return 0;
	}

	ret = sm5703_fg_get_capacity(fg, &capacity);
	if (ret)
		return ret;

	*status = capacity >= 100 ? POWER_SUPPLY_STATUS_FULL :
				    POWER_SUPPLY_STATUS_NOT_CHARGING;

	return 0;
}

/*
 * There is no health register on this gauge: its status register carries
 * only the alert flags.  What can be said comes from comparing the cell
 * against the limits the battery node declares, so a board without a
 * monitored-battery, or without a thermistor, gets an honest unknown
 * rather than a guess.
 */
static int sm5703_fg_get_health(struct sm5703_fg *fg, int *health)
{
	int ret, voltage, temp;

	*health = POWER_SUPPLY_HEALTH_UNKNOWN;

	if (!fg->info)
		return 0;

	ret = sm5703_fg_get_voltage(fg, &voltage);
	if (ret)
		return ret;

	if (fg->info->voltage_max_design_uv > 0 &&
	    voltage * 1000 > fg->info->voltage_max_design_uv) {
		*health = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		return 0;
	}

	ret = sm5703_fg_get_temp(fg, &temp);
	if (ret == -ENODATA)
		return 0;
	if (ret)
		return ret;

	/* power_supply_battery_info keeps these in whole degrees. */
	if (fg->info->temp_max != INT_MAX && temp / 10 > fg->info->temp_max)
		*health = POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (fg->info->temp_min != INT_MIN && temp / 10 < fg->info->temp_min)
		*health = POWER_SUPPLY_HEALTH_COLD;
	else
		*health = POWER_SUPPLY_HEALTH_GOOD;

	return 0;
}

static int sm5703_fg_get_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  union power_supply_propval *val)
{
	struct sm5703_fg *fg = power_supply_get_drvdata(psy);
	int ret = 0, tmp;

	mutex_lock(&fg->lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		ret = sm5703_fg_get_status(fg, &val->intval);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = fg->info ? fg->info->technology :
					 POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = sm5703_fg_get_capacity(fg, &val->intval);
		break;
	case POWER_SUPPLY_PROP_CAPACITY_ALERT_MIN:
		val->intval = fg->alert_soc;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = sm5703_fg_get_voltage(fg, &tmp);
		val->intval = tmp * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
		ret = sm5703_fg_get_voltage_avg(fg, &tmp);
		val->intval = tmp * 1000;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
		ret = sm5703_fg_get_ocv(fg, &tmp);
		val->intval = tmp * 1000;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = sm5703_fg_get_current(fg, &tmp);
		val->intval = tmp * 1000;
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		ret = sm5703_fg_get_current_avg(fg, &tmp);
		val->intval = tmp * 1000;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		ret = sm5703_fg_get_temp(fg, &val->intval);
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		ret = sm5703_fg_get_health(fg, &val->intval);
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		if (!fg->info || fg->info->charge_full_design_uah < 0) {
			ret = -ENODATA;
			break;
		}
		val->intval = fg->info->charge_full_design_uah;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		if (!fg->info || fg->info->charge_full_design_uah < 0) {
			ret = -ENODATA;
			break;
		}
		ret = sm5703_fg_get_capacity(fg, &tmp);
		val->intval = fg->info->charge_full_design_uah / 100 * tmp;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		if (!fg->info || fg->info->voltage_max_design_uv < 0) {
			ret = -ENODATA;
			break;
		}
		val->intval = fg->info->voltage_max_design_uv;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		if (!fg->info || fg->info->voltage_min_design_uv < 0) {
			ret = -ENODATA;
			break;
		}
		val->intval = fg->info->voltage_min_design_uv;
		break;
	default:
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&fg->lock);

	return ret;
}

static int sm5703_fg_set_property(struct power_supply *psy,
				  enum power_supply_property psp,
				  const union power_supply_propval *val)
{
	struct sm5703_fg *fg = power_supply_get_drvdata(psy);
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_CAPACITY_ALERT_MIN:
		if (val->intval < 0 || val->intval > 100)
			return -EINVAL;

		mutex_lock(&fg->lock);
		ret = sm5703_fg_set_alert_soc(fg, val->intval);
		mutex_unlock(&fg->lock);
		return ret;
	default:
		return -EINVAL;
	}
}

static int sm5703_fg_property_is_writeable(struct power_supply *psy,
					   enum power_supply_property psp)
{
	return psp == POWER_SUPPLY_PROP_CAPACITY_ALERT_MIN;
}

static enum power_supply_property sm5703_fg_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_ALERT_MIN,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_HEALTH,
};

static const struct power_supply_desc sm5703_fg_desc = {
	.name			= "sm5703-fuelgauge",
	.type			= POWER_SUPPLY_TYPE_BATTERY,
	.properties		= sm5703_fg_props,
	.num_properties		= ARRAY_SIZE(sm5703_fg_props),
	.get_property		= sm5703_fg_get_property,
	.set_property		= sm5703_fg_set_property,
	.property_is_writeable	= sm5703_fg_property_is_writeable,
};

static int sm5703_fg_parse_params(struct sm5703_fg *fg)
{
	struct sm5703_fg_params *p = &fg->params;
	struct device *dev = fg->dev;
	static const char * const table_names[SM5703_FG_TABLE_COUNT] = {
		"siliconmitus,battery-table-discharge",
		"siliconmitus,battery-table-charge",
		"siliconmitus,battery-table-soc",
	};
	u32 offset_cal = 0;
	int ret, i;

	for (i = 0; i < SM5703_FG_TABLE_COUNT; i++) {
		ret = device_property_read_u32_array(dev, table_names[i],
						     p->table[i],
						     SM5703_FG_TABLE_LEN);
		if (ret) {
			dev_dbg(dev, "missing %s\n", table_names[i]);
			return ret;
		}
	}

	ret = device_property_read_u32_array(dev, "siliconmitus,rce-values",
					     p->rce, ARRAY_SIZE(p->rce));
	if (!ret)
		ret = device_property_read_u32(dev, "siliconmitus,dtcd-value",
					       &p->dtcd);
	if (!ret)
		ret = device_property_read_u32_array(dev, "siliconmitus,rs-values",
						     p->rs, ARRAY_SIZE(p->rs));
	if (!ret)
		ret = device_property_read_u32(dev, "siliconmitus,vit-period",
					       &p->vit_period);
	if (!ret)
		ret = device_property_read_u32_array(dev, "siliconmitus,mix-values",
						     p->mix, ARRAY_SIZE(p->mix));
	if (!ret)
		ret = device_property_read_u32_array(dev, "siliconmitus,topoff-soc",
						     p->topoff_soc,
						     ARRAY_SIZE(p->topoff_soc));
	if (!ret)
		ret = device_property_read_u32(dev, "siliconmitus,volt-cal",
					       &p->volt_cal);
	if (!ret)
		ret = device_property_read_u32(dev, "siliconmitus,curr-cal",
					       &p->curr_cal);
	if (!ret)
		ret = device_property_read_u32(dev, "siliconmitus,temp-std",
					       &p->temp_std);
	if (!ret)
		ret = device_property_read_u32(dev, "siliconmitus,temp-offset",
					       &p->temp_offset);
	if (!ret)
		ret = device_property_read_u32(dev, "siliconmitus,temp-offset-cal",
					       &p->temp_offset_cal);
	if (!ret) {
		ret = device_property_read_u32(dev, "siliconmitus,charge-offset-cal",
					       &offset_cal);
		p->charge_offset_cal = (s32)offset_cal;
	}
	if (!ret)
		ret = device_property_read_u32_array(dev, "siliconmitus,tem-poff",
						     p->tem_poff,
						     ARRAY_SIZE(p->tem_poff));
	if (ret) {
		dev_dbg(dev, "incomplete battery calibration data\n");
		return ret;
	}

	p->valid = true;

	return 0;
}

static const struct regmap_config sm5703_fg_regmap_config = {
	.reg_bits		= 8,
	.val_bits		= 16,
	.max_register		= 0xff,
	.val_format_endian	= REGMAP_ENDIAN_LITTLE,
	.cache_type		= REGCACHE_NONE,
};

static int sm5703_fg_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct power_supply_config cfg = { };
	struct sm5703_fg *fg;
	u16 id, status;
	int ret;

	fg = devm_kzalloc(dev, sizeof(*fg), GFP_KERNEL);
	if (!fg)
		return -ENOMEM;

	fg->dev = dev;
	fg->alert_soc = 2;
	i2c_set_clientdata(client, fg);

	ret = devm_mutex_init(dev, &fg->lock);
	if (ret)
		return ret;

	fg->regmap = devm_regmap_init_i2c(client, &sm5703_fg_regmap_config);
	if (IS_ERR(fg->regmap))
		return dev_err_probe(dev, PTR_ERR(fg->regmap),
				     "failed to initialise regmap\n");

	ret = sm5703_fg_read(fg, SM5703_FG_REG_DEVICE_ID, &id);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read device ID\n");

	dev_dbg(dev, "SM5703 fuel gauge, device ID 0x%04x\n", id);

	/*
	 * The calibration data is cell specific and cannot be guessed.  A
	 * gauge without it still reports usable values as long as the
	 * bootloader or a previous kernel programmed it, so carry on with a
	 * warning rather than failing the probe.
	 */
	ret = sm5703_fg_parse_params(fg);
	if (ret)
		dev_warn(dev,
			 "no battery calibration data, readings may be inaccurate\n");

	device_property_read_u32(dev, "siliconmitus,alert-soc-percent",
				 &fg->alert_soc);
	if (fg->alert_soc > 100)
		return dev_err_probe(dev, -EINVAL,
				     "invalid siliconmitus,alert-soc-percent\n");

	ret = sm5703_fg_read(fg, SM5703_FG_REG_OP_STATUS, &status);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read operating status\n");

	if (fg->params.valid &&
	    (status & 0x00ff) != SM5703_FG_OP_STATUS_INITIALISED) {
		ret = sm5703_fg_program_params(fg, false);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to program battery parameters\n");
	} else {
		/* Already running: just make sure the modes this driver
		 * depends on are enabled.
		 */
		ret = regmap_update_bits(fg->regmap, SM5703_FG_REG_CNTL,
					 SM5703_FG_CNTL_MIX_MODE |
					 SM5703_FG_CNTL_TEMP_MEASURE,
					 SM5703_FG_CNTL_MIX_MODE |
					 SM5703_FG_CNTL_TEMP_MEASURE);
		if (ret)
			return dev_err_probe(dev, ret, "failed to set up gauge\n");
	}

	cfg.drv_data = fg;
	cfg.fwnode = dev_fwnode(dev);

	fg->psy = devm_power_supply_register(dev, &sm5703_fg_desc, &cfg);
	if (IS_ERR(fg->psy))
		return dev_err_probe(dev, PTR_ERR(fg->psy),
				     "failed to register power supply\n");

	ret = power_supply_get_battery_info(fg->psy, &fg->info);
	if (ret) {
		fg->info = NULL;

		if (ret != -ENODEV && ret != -ENOENT)
			return dev_err_probe(dev, ret,
					     "failed to get battery info\n");

		dev_dbg(dev, "no monitored-battery, design values unavailable\n");
	}

	ret = sm5703_fg_alert_init(fg);
	if (ret)
		return dev_err_probe(dev, ret, "failed to set up fuel alert\n");

	if (client->irq > 0) {
		ret = devm_request_threaded_irq(dev, client->irq, NULL,
						sm5703_fg_irq_thread,
						IRQF_ONESHOT,
						"sm5703-fuelgauge", fg);
		if (ret)
			return dev_err_probe(dev, ret, "failed to request IRQ\n");
	}

	return 0;
}

static void sm5703_fg_remove(struct i2c_client *client)
{
	struct sm5703_fg *fg = i2c_get_clientdata(client);

	if (fg->info)
		power_supply_put_battery_info(fg->psy, fg->info);
}

static const struct of_device_id sm5703_fg_of_match[] = {
	{ .compatible = "siliconmitus,sm5703-fuelgauge" },
	{ }
};
MODULE_DEVICE_TABLE(of, sm5703_fg_of_match);

static const struct i2c_device_id sm5703_fg_id[] = {
	{ "sm5703-fuelgauge" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sm5703_fg_id);

static struct i2c_driver sm5703_fg_driver = {
	.driver = {
		.name		= "sm5703-fuelgauge",
		.of_match_table	= sm5703_fg_of_match,
	},
	.probe		= sm5703_fg_probe,
	.remove		= sm5703_fg_remove,
	.id_table	= sm5703_fg_id,
};
module_i2c_driver(sm5703_fg_driver);

MODULE_DESCRIPTION("Silicon Mitus SM5703 fuel gauge driver");
MODULE_LICENSE("GPL");
