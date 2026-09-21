// SPDX-License-Identifier: GPL-2.0
// STMicroelectronics FTS Touchscreen device driver
//
// Copyright (c) 2017 Samsung Electronics Co., Ltd.
// Copyright (c) 2017 Andi Shyti <andi@etezian.org>
//
// Power-up and initialisation sequence derived from the Samsung downstream
// driver (drivers/input/touchscreen/stm/fts_ts.c, Exynos7870 kernel), as
// required by the FTS8 generation of controllers found on the Galaxy Tab A
// 10.1 (SM-T580) and relatives.

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>

/* single byte commands */
#define STMFTS_READ_STATUS			0x84
#define STMFTS_READ_ONE_EVENT			0x85
#define STMFTS_READ_ALL_EVENT			0x86
#define STMFTS_SLEEP_IN				0x90
#define STMFTS_SLEEP_OUT			0x91
#define STMFTS_MS_MT_SENSE_OFF			0x92
#define STMFTS_MS_MT_SENSE_ON			0x93
#define STMFTS_MS_KEY_SENSE_OFF			0x9a
#define STMFTS_MS_KEY_SENSE_ON			0x9b
#define STMFTS_CLEAR_EVENT_STACK		0xa1
#define STMFTS_FULL_FORCE_CALIBRATION		0xa2
#define STMFTS_RELEASE_INFO			0xaa

/*
 * Register writes.  The FTS8 generation does not implement the single byte
 * 0xa0 system reset; everything goes through the 0xb6 register window.
 *
 * This part is a hybrid of two register maps: the system reset is the newer
 * 0x28 / 0x80 pair, while the attention enable is still the older 0x1c /
 * 0x41 one.  The newer 0x2c / 0x48 pair is accepted and reads back on this
 * controller, but does not connect the event FIFO to the attention output:
 * events queue up and the line is never asserted.
 */
#define STMFTS_REG_WRITE			0xb6
#define STMFTS_REG_CHIP_ID			0x04	/* read, 6 bytes */
#define STMFTS_REG_CRC_STATUS			0x08	/* read, 2 bytes */
#define STMFTS_REG_WARMBOOT_CRC			0x1e	/* write 0x20 */
#define STMFTS_REG_FIFO_STATUS			0x23	/* read, 1 byte */
#define STMFTS_REG_SYSTEM_RESET			0x28	/* write 0x80 */
#define STMFTS_REG_INTERRUPT			0x1c	/* write 0x41/0x00 */

#define STMFTS_WARMBOOT_CRC_ENABLE		0x20
#define STMFTS_SYSTEM_RESET			0x80
#define STMFTS_INTERRUPT_ENABLE			0x41
#define STMFTS_INTERRUPT_DISABLE		0x00

/* events */
#define STMFTS_EV_NO_EVENT			0x00
#define STMFTS_EV_MULTI_TOUCH_ENTER		0x03
#define STMFTS_EV_MULTI_TOUCH_LEAVE		0x04
#define STMFTS_EV_MULTI_TOUCH_MOTION		0x05
#define STMFTS_EV_KEY_STATUS			0x0e
#define STMFTS_EV_ERROR				0x0f
#define STMFTS_EV_CONTROLLER_READY		0x10
#define STMFTS_EV_SLEEP_OUT_CONTROLLER_READY	0x11
#define STMFTS_EV_STATUS			0x16
#define STMFTS_EV_INTERNAL_RELEASE_INFO		0x19
#define STMFTS_EV_EXTERNAL_RELEASE_INFO		0x1a
#define STMFTS_EV_DEBUG				0xdb

/* error event sub-codes */
#define STMFTS_ERR_FLASH_CORRUPTION		0x03

/* multi touch related event masks */
#define STMFTS_MASK_EVENT_ID			0x0f
#define STMFTS_MASK_TOUCH_ID			0xf0
#define STMFTS_MASK_X_MSB			0x0f

/* key related event masks */
#define STMFTS_MASK_KEY_MENU			0x01
#define STMFTS_MASK_KEY_BACK			0x02

#define STMFTS_CHIP_ID0				0x36
#define STMFTS_CHIP_ID1				0x70

#define STMFTS_EVENT_SIZE	8
#define STMFTS_STACK_DEPTH	32
#define STMFTS_DATA_MAX_SIZE	(STMFTS_EVENT_SIZE * STMFTS_STACK_DEPTH)
#define STMFTS_MAX_FINGERS	10
#define STMFTS_DEV_NAME		"stmfts"

/* upper bound on how many events are drained in one go */
#define STMFTS_MAX_EVENTS	64

#define STMFTS_I2C_RETRIES	3	/* downstream FTS_TS_I2C_RETRY_CNT */
#define STMFTS_READY_ERRORS	32
#define STMFTS_INIT_RETRIES	3

/*
 * How long the polling loops keep asking before giving up.  This is a wall
 * clock deadline and not a retry count on purpose: a controller that has
 * been powered off pulls the bus down through its ESD diodes, and a single
 * transfer then costs the adapter its own multi second timeout, which would
 * make an iteration count look like a hang.
 */
#define STMFTS_READY_TIMEOUT_MS	2000

struct stmfts_data {
	struct i2c_client *client;
	struct input_dev *input;

	/* serialises access to the transfer buffers below */
	struct mutex mutex;

	/*
	 * Serialises the touch key configuration and the key state against the
	 * event handler.  Deliberately not @mutex: that one is taken inside
	 * every transfer helper, so a command issued while holding it would
	 * deadlock against itself.
	 */
	struct mutex key_mutex;

	struct touchscreen_properties prop;

	struct regulator *avdd;
	struct regulator *vdd;
	bool regulators_enabled;

	bool use_key;
	u8 key_state;
	/*
	 * Swap the keycodes the two touch keys report.  The controller only
	 * says "key 0" and "key 1"; which one is physically left of the home
	 * button is a property of the panel, not of the chip, and on some
	 * boards it is the other way round from the ST reference.  Exposed as
	 * the 'key_swap' sysfs attribute.
	 */
	bool key_swap;
	/*
	 * Stop scanning the touch keys altogether.  A device whose keys
	 * duplicate an on-screen navigation bar wants them silenced, and the
	 * controller can do it in hardware.  Exposed as 'key_disable'.
	 */
	bool key_disable;

	u16 chip_id;
	u16 fw_ver;
	u16 config_ver;
	u16 main_ver;

	/*
	 * Part of a kmalloc'ed structure, so usable as an I2C transfer
	 * buffer.  Protected by @mutex.
	 */
	u8 cmd[4];
	u8 reg[8];
	u8 data[STMFTS_DATA_MAX_SIZE];
};

static void stmfts_delay(unsigned int ms)
{
	fsleep(ms * USEC_PER_MSEC);
}

/*
 * The controller needs plain I2C transfers: the register writes are four
 * bytes long with no register/value split, which SMBus cannot express, and
 * the event FIFO is read back without a repeated command byte.  Both helpers
 * retry like the downstream driver does, because the controller is known to
 * NAK for a while right after a reset.
 */
static int stmfts_write(struct stmfts_data *sdata, const u8 *cmd, size_t clen)
{
	struct i2c_msg msg;
	int err = -EIO;
	int i;

	if (WARN_ON(clen > sizeof(sdata->cmd)))
		return -EINVAL;

	mutex_lock(&sdata->mutex);

	memcpy(sdata->cmd, cmd, clen);

	msg.addr = sdata->client->addr;
	msg.flags = 0;
	msg.len = clen;
	msg.buf = sdata->cmd;

	for (i = 0; i < STMFTS_I2C_RETRIES; i++) {
		err = i2c_transfer(sdata->client->adapter, &msg, 1);
		if (err == 1) {
			err = 0;
			break;
		}

		dev_dbg(&sdata->client->dev, "write %#02x failed: %d (try %d)\n",
			cmd[0], err, i);
		usleep_range(10000, 11000);
	}

	mutex_unlock(&sdata->mutex);

	return err > 0 ? -EIO : err;
}

static int stmfts_command(struct stmfts_data *sdata, u8 cmd)
{
	return stmfts_write(sdata, &cmd, 1);
}

/*
 * @buf must live inside struct stmfts_data (or be otherwise DMA safe) and is
 * only valid until the next transfer.
 */
static int stmfts_read(struct stmfts_data *sdata, const u8 *cmd, size_t clen,
		       u8 *buf, size_t len)
{
	struct i2c_msg msgs[2];
	int err = -EIO;
	int i;

	if (WARN_ON(clen > sizeof(sdata->cmd)))
		return -EINVAL;

	mutex_lock(&sdata->mutex);

	memcpy(sdata->cmd, cmd, clen);

	msgs[0].addr = sdata->client->addr;
	msgs[0].flags = 0;
	msgs[0].len = clen;
	msgs[0].buf = sdata->cmd;

	msgs[1].addr = sdata->client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = buf;

	for (i = 0; i < STMFTS_I2C_RETRIES; i++) {
		err = i2c_transfer(sdata->client->adapter, msgs,
				   ARRAY_SIZE(msgs));
		if (err == ARRAY_SIZE(msgs)) {
			err = 0;
			break;
		}

		dev_dbg(&sdata->client->dev, "read %#02x failed: %d (try %d)\n",
			cmd[0], err, i);
		usleep_range(10000, 11000);
	}

	mutex_unlock(&sdata->mutex);

	return err > 0 ? -EIO : err;
}

static int stmfts_write_reg(struct stmfts_data *sdata, u8 reg, u8 val)
{
	u8 cmd[4] = { STMFTS_REG_WRITE, 0x00, reg, val };

	return stmfts_write(sdata, cmd, sizeof(cmd));
}

/*
 * Register reads answer with one echo byte followed by the payload, so the
 * caller always gets @len bytes with the data starting at buf[1].
 */
static int stmfts_read_reg(struct stmfts_data *sdata, u8 reg,
			   u8 *buf, size_t len)
{
	u8 cmd[3] = { STMFTS_REG_WRITE, 0x00, reg };

	return stmfts_read(sdata, cmd, sizeof(cmd), buf, len);
}

/* power handling */

static int stmfts_power_on(struct stmfts_data *sdata)
{
	int err;

	if (sdata->regulators_enabled)
		return 0;

	err = regulator_enable(sdata->avdd);
	if (err) {
		dev_err(&sdata->client->dev,
			"failed to enable avdd: %d\n", err);
		return err;
	}

	/* the downstream driver waits 1ms between the two supplies */
	stmfts_delay(1);

	err = regulator_enable(sdata->vdd);
	if (err) {
		dev_err(&sdata->client->dev, "failed to enable vdd: %d\n", err);
		regulator_disable(sdata->avdd);
		return err;
	}

	sdata->regulators_enabled = true;

	/* time for the controller to come out of its own boot */
	stmfts_delay(5);

	return 0;
}

/*
 * The supplies are plain GPIO controlled fixed regulators and the bootloader
 * leaves them on, so an ordinary regulator_disable() would be a no-op and the
 * controller would never redo its boot sequence.  Drop our own reference
 * first, so use_count reaches zero, then force the hardware off.
 */
static void stmfts_power_off(struct stmfts_data *sdata)
{
	if (sdata->regulators_enabled) {
		regulator_disable(sdata->vdd);
		regulator_disable(sdata->avdd);
		sdata->regulators_enabled = false;
	}

	if (regulator_is_enabled(sdata->vdd))
		regulator_force_disable(sdata->vdd);

	if (regulator_is_enabled(sdata->avdd))
		regulator_force_disable(sdata->avdd);
}

/*
 * A full power cycle.  @ms is the time the controller is kept unpowered:
 * short for a plain retry, long when it has to discharge after ESD.
 *
 * While the controller is off it pulls the I2C lines down through its ESD
 * diodes and the whole bus becomes unusable, so this must never be left
 * half done: every caller has to reach the power on again.
 */
static int stmfts_power_cycle(struct stmfts_data *sdata, unsigned int ms)
{
	dev_dbg(&sdata->client->dev, "power cycle, off for %u ms\n", ms);

	stmfts_power_off(sdata);
	stmfts_delay(ms);

	return stmfts_power_on(sdata);
}

/* initialisation */

/*
 * Poll the event FIFO until the controller announces it is ready.  This is
 * deliberately not interrupt driven: the downstream driver only requests the
 * interrupt once the whole initialisation is over, and the controller does
 * not reliably assert the attention line before its first sense command.
 */
static int stmfts_wait_for_ready(struct stmfts_data *sdata)
{
	unsigned long deadline = jiffies +
				 msecs_to_jiffies(STMFTS_READY_TIMEOUT_MS);
	u8 cmd = STMFTS_READ_ONE_EVENT;
	unsigned int errors = 0;
	int err;

	for (;;) {
		err = stmfts_read(sdata, &cmd, 1, sdata->reg,
				  STMFTS_EVENT_SIZE);
		if (err) {
			/*
			 * The controller NAKs while it is still resetting.
			 * Treat it like an empty FIFO rather than giving up,
			 * but do not spin: fall through to the delay below.
			 */
			if (time_after(jiffies, deadline)) {
				dev_err(&sdata->client->dev,
					"no answer while waiting for ready\n");
				return err;
			}

			stmfts_delay(20);
			continue;
		}

		switch (sdata->reg[0]) {
		case STMFTS_EV_CONTROLLER_READY:
			dev_dbg(&sdata->client->dev, "controller ready\n");
			return 0;

		case STMFTS_EV_ERROR:
			if (sdata->reg[1] == STMFTS_ERR_FLASH_CORRUPTION) {
				dev_err(&sdata->client->dev,
					"flash corruption: %02x %02x %02x\n",
					sdata->reg[0], sdata->reg[1],
					sdata->reg[2]);
				return -ENODEV;
			}

			/*
			 * Any other error event is drained without delay:
			 * the controller queues a few of them on every boot.
			 */
			if (errors++ > STMFTS_READY_ERRORS) {
				dev_err(&sdata->client->dev,
					"too many error events\n");
				return -EIO;
			}

			if (!time_after(jiffies, deadline))
				continue;
		}

		if (time_after(jiffies, deadline)) {
			dev_err(&sdata->client->dev,
				"timeout waiting for ready, last event %02x\n",
				sdata->reg[0]);
			return -ETIMEDOUT;
		}

		stmfts_delay(20);
	}
}

static int stmfts_system_reset(struct stmfts_data *sdata)
{
	int err;

	err = stmfts_write_reg(sdata, STMFTS_REG_SYSTEM_RESET,
			       STMFTS_SYSTEM_RESET);
	if (err)
		return err;

	stmfts_delay(10);

	return stmfts_wait_for_ready(sdata);
}

static int stmfts_read_chip_id(struct stmfts_data *sdata)
{
	int err;

	err = stmfts_read_reg(sdata, STMFTS_REG_CHIP_ID, sdata->reg, 7);
	if (err)
		return err;

	/*
	 * Not every FingerTipS variant exposes the id in this register: the
	 * controller on the Galaxy Tab A 10.1 (SM-T580) returns zeroes here
	 * while answering everything else correctly.  Warn and carry on
	 * rather than refusing to drive a working panel.
	 */
	if (sdata->reg[1] != STMFTS_CHIP_ID0 ||
	    sdata->reg[2] != STMFTS_CHIP_ID1)
		dev_warn(&sdata->client->dev,
			 "unexpected chip id %02x%02x, continuing anyway\n",
			 sdata->reg[1], sdata->reg[2]);

	sdata->chip_id = (sdata->reg[1] << 8) | sdata->reg[2];

	return 0;
}

/*
 * Ask for the firmware versions.  Purely informative, so a failure here is
 * not fatal, but it is the first read that exercises the event FIFO end to
 * end and therefore a useful thing to see in the log.
 */
static int stmfts_read_version(struct stmfts_data *sdata)
{
	unsigned long deadline = jiffies +
				 msecs_to_jiffies(STMFTS_READY_TIMEOUT_MS);
	u8 cmd = STMFTS_READ_ONE_EVENT;
	int err;

	err = stmfts_command(sdata, STMFTS_RELEASE_INFO);
	if (err)
		return err;

	for (;;) {
		err = stmfts_read(sdata, &cmd, 1, sdata->reg,
				  STMFTS_EVENT_SIZE);
		if (err)
			return err;

		switch (sdata->reg[0]) {
		case STMFTS_EV_INTERNAL_RELEASE_INFO:
			sdata->fw_ver = (sdata->reg[3] << 8) | sdata->reg[4];
			sdata->config_ver = (sdata->reg[6] << 8) |
					     sdata->reg[5];
			break;

		case STMFTS_EV_EXTERNAL_RELEASE_INFO:
			sdata->main_ver = (sdata->reg[1] << 8) | sdata->reg[2];
			return 0;
		}

		if (time_after(jiffies, deadline))
			return -ETIMEDOUT;

		/*
		 * The bound is a deadline, not a retry count, so pace the
		 * loop: without this it would hammer the bus for two seconds
		 * whenever the FIFO comes back empty.
		 */
		stmfts_delay(2);
	}
}

/*
 * Bring the controller up.  The order matters and comes from the downstream
 * driver: the warm boot CRC check is disabled by default on FTS8 and has to
 * be turned on after every power on, before the reset that makes it effective.
 */
static int stmfts_initialise(struct stmfts_data *sdata)
{
	unsigned int i;
	int err = -ENODEV;

	for (i = 0; i < STMFTS_INIT_RETRIES; i++) {
		err = stmfts_write_reg(sdata, STMFTS_REG_WARMBOOT_CRC,
				       STMFTS_WARMBOOT_CRC_ENABLE);
		if (!err) {
			stmfts_delay(10);
			err = stmfts_system_reset(sdata);
		}

		if (!err)
			break;

		dev_warn(&sdata->client->dev,
			 "reset attempt %u failed: %d\n", i, err);

		err = stmfts_power_cycle(sdata, 20);
		if (err)
			return err;

		err = -ENODEV;
	}

	if (err) {
		dev_err(&sdata->client->dev, "failed to reset controller\n");
		return err;
	}

	/* an all zero CRC status means the firmware in flash is not usable */
	if (!stmfts_read_reg(sdata, STMFTS_REG_CRC_STATUS, sdata->reg, 3) &&
	    !sdata->reg[1] && !sdata->reg[2])
		dev_warn(&sdata->client->dev,
			 "boot CRC check failed, firmware may be corrupted\n");

	err = stmfts_read_version(sdata);
	if (err)
		dev_warn(&sdata->client->dev,
			 "failed to read version info: %d\n", err);
	else
		dev_dbg(&sdata->client->dev,
			"firmware %04x, config %04x, main %04x\n",
			sdata->fw_ver, sdata->config_ver, sdata->main_ver);

	err = stmfts_read_chip_id(sdata);
	if (err) {
		/* let the controller discharge before giving up on it */
		err = stmfts_power_cycle(sdata, 500);
		if (err)
			return err;

		err = stmfts_read_chip_id(sdata);
		if (err)
			return err;
	}

	dev_dbg(&sdata->client->dev, "chip id %04x\n", sdata->chip_id);

	/*
	 * Calibrate before the sensing is turned on and give the controller
	 * time to finish: a full force calibration takes a while and the
	 * status events it produces are then dropped by the flush below.
	 * The downstream driver does not calibrate at all on its runtime
	 * reinit path, so this is not needed to make the controller report
	 * touches, only to make them accurate.
	 */
	err = stmfts_command(sdata, STMFTS_FULL_FORCE_CALIBRATION);
	if (err)
		dev_warn(&sdata->client->dev,
			 "failed to calibrate: %d\n", err);
	else
		stmfts_delay(100);

	err = stmfts_command(sdata, STMFTS_CLEAR_EVENT_STACK);
	if (err)
		return err;

	stmfts_delay(10);

	err = stmfts_command(sdata, STMFTS_MS_MT_SENSE_ON);
	if (err)
		return err;

	/* the downstream reinit path waits here before touching anything */
	stmfts_delay(50);

	if (sdata->use_key && !sdata->key_disable) {
		err = stmfts_command(sdata, STMFTS_MS_KEY_SENSE_ON);
		if (err)
			/* the touchscreen is usable without the keys */
			dev_warn(&sdata->client->dev,
				 "failed to enable touchkeys: %d\n", err);
	}

	/*
	 * The attention output is left disabled by the boot firmware and is
	 * turned on separately, once the handler is in place: see
	 * stmfts_enable_interrupts().
	 */
	return 0;
}

/*
 * Turn the attention output on.  This is deliberately the very last thing
 * that happens, after the SoC side interrupt has been unmasked: if the event
 * FIFO is not empty the controller pulls the line down immediately and the
 * handler runs straight away.
 */
static int stmfts_enable_interrupts(struct stmfts_data *sdata)
{
	return stmfts_write_reg(sdata, STMFTS_REG_INTERRUPT,
				STMFTS_INTERRUPT_ENABLE);
}

/* event handling */

static void stmfts_report_contact_event(struct stmfts_data *sdata,
					const u8 event[])
{
	u8 slot_id = (event[0] & STMFTS_MASK_TOUCH_ID) >> 4;
	u16 x = event[1] | ((event[2] & STMFTS_MASK_X_MSB) << 8);
	u16 y = (event[2] >> 4) | (event[3] << 4);
	u8 bw = event[4];
	u8 bh = event[5];
	u8 area = event[7];

	if (slot_id >= STMFTS_MAX_FINGERS)
		return;

	input_mt_slot(sdata->input, slot_id);
	input_mt_report_slot_state(sdata->input, MT_TOOL_FINGER, true);

	touchscreen_report_pos(sdata->input, &sdata->prop, x, y, true);
	input_report_abs(sdata->input, ABS_MT_TOUCH_MAJOR, max(bw, bh));
	input_report_abs(sdata->input, ABS_MT_TOUCH_MINOR, min(bw, bh));
	input_report_abs(sdata->input, ABS_MT_PRESSURE, area);
}

static void stmfts_report_contact_release(struct stmfts_data *sdata,
					  const u8 event[])
{
	u8 slot_id = (event[0] & STMFTS_MASK_TOUCH_ID) >> 4;

	if (slot_id >= STMFTS_MAX_FINGERS)
		return;

	input_mt_slot(sdata->input, slot_id);
	input_mt_report_slot_inactive(sdata->input);
}

/*
 * The key event carries the state of every key as a bitmap, so more than one
 * key can change at a time.
 */
static void stmfts_report_key_event(struct stmfts_data *sdata, const u8 event[])
{
	u8 keys = event[2];
	u8 changed;

	mutex_lock(&sdata->key_mutex);

	changed = keys ^ sdata->key_state;

	if (changed & STMFTS_MASK_KEY_MENU)
		input_report_key(sdata->input,
				 sdata->key_swap ? KEY_BACK : KEY_MENU,
				 !!(keys & STMFTS_MASK_KEY_MENU));

	if (changed & STMFTS_MASK_KEY_BACK)
		input_report_key(sdata->input,
				 sdata->key_swap ? KEY_MENU : KEY_BACK,
				 !!(keys & STMFTS_MASK_KEY_BACK));

	sdata->key_state = keys;

	mutex_unlock(&sdata->key_mutex);
}

static void stmfts_parse_event(struct stmfts_data *sdata, const u8 *event)
{
	switch (event[0] & STMFTS_MASK_EVENT_ID) {
	case STMFTS_EV_MULTI_TOUCH_ENTER:
	case STMFTS_EV_MULTI_TOUCH_MOTION:
		stmfts_report_contact_event(sdata, event);
		return;

	case STMFTS_EV_MULTI_TOUCH_LEAVE:
		stmfts_report_contact_release(sdata, event);
		return;
	}

	switch (event[0]) {
	case STMFTS_EV_KEY_STATUS:
		stmfts_report_key_event(sdata, event);
		break;

	case STMFTS_EV_ERROR:
		dev_warn(&sdata->client->dev,
			 "error event: %02x %02x %02x\n",
			 event[1], event[2], event[3]);
		break;

	case STMFTS_EV_CONTROLLER_READY:
	case STMFTS_EV_SLEEP_OUT_CONTROLLER_READY:
	case STMFTS_EV_STATUS:
	case STMFTS_EV_DEBUG:
		dev_dbg(&sdata->client->dev,
			"status event: %02x %02x %02x\n",
			event[0], event[1], event[2]);
		break;

	default:
		dev_dbg(&sdata->client->dev,
			"unknown event %#02x\n", event[0]);
	}
}

/*
 * Drain the event FIFO one event at a time until it reports empty.
 *
 * The obvious alternative, reading the queued count from the FIFO status
 * register and fetching that many events in one go, is what the vendor driver
 * does, but the count this controller reports does not fit its own 32 event
 * stack: it comes back as 0x80, so the top bit is a flag of some sort rather
 * than part of the number.  Asking for more bytes than the controller has
 * makes it stop driving the bus, and the Exynos adapter then needs five
 * seconds to give up.  One transaction per event avoids that.
 */
static void stmfts_handle_events(struct stmfts_data *sdata)
{
	u8 cmd = STMFTS_READ_ONE_EVENT;
	bool reported = false;
	unsigned int i;
	int err;

	for (i = 0; i < STMFTS_MAX_EVENTS; i++) {
		err = stmfts_read(sdata, &cmd, 1, sdata->data,
				  STMFTS_EVENT_SIZE);
		if (err) {
			dev_err(&sdata->client->dev,
				"failed to read event: %d\n", err);
			break;
		}

		if (sdata->data[0] == STMFTS_EV_NO_EVENT)
			break;

		stmfts_parse_event(sdata, sdata->data);
		reported = true;
	}

	if (!reported)
		return;

	input_mt_sync_frame(sdata->input);
	input_sync(sdata->input);
}

static irqreturn_t stmfts_irq_handler(int irq, void *dev)
{
	stmfts_handle_events(dev);

	return IRQ_HANDLED;
}

/* input device */

static int stmfts_input_open(struct input_dev *dev)
{
	struct stmfts_data *sdata = input_get_drvdata(dev);

	return stmfts_command(sdata, STMFTS_MS_MT_SENSE_ON);
}

static void stmfts_input_close(struct input_dev *dev)
{
	struct stmfts_data *sdata = input_get_drvdata(dev);
	int err;

	err = stmfts_command(sdata, STMFTS_MS_MT_SENSE_OFF);
	if (err)
		dev_warn(&sdata->client->dev,
			 "failed to disable sensing: %d\n", err);
}

/* sysfs */

static ssize_t chip_id_show(struct device *dev,
			    struct device_attribute *attr, char *buf)
{
	struct stmfts_data *sdata = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%#x\n", sdata->chip_id);
}

static ssize_t fw_ver_show(struct device *dev,
			   struct device_attribute *attr, char *buf)
{
	struct stmfts_data *sdata = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%#x\n", sdata->fw_ver);
}

static ssize_t config_version_show(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct stmfts_data *sdata = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%#x\n", sdata->config_ver);
}

/*
 * Which of the two touch keys is which is a board property, and userspace is
 * the only place that knows what is silkscreened on the glass.  Rather than
 * hardcoding one board's wiring in the driver, let it be flipped at runtime.
 *
 * Releases are synthesised on the way out so a key cannot get stuck down
 * under the old keycode when the mapping changes while it is held.
 */
static ssize_t key_swap_show(struct device *dev,
			     struct device_attribute *attr, char *buf)
{
	struct stmfts_data *sdata = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u\n", sdata->key_swap ? 1 : 0);
}

static ssize_t key_swap_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct stmfts_data *sdata = dev_get_drvdata(dev);
	bool swap;
	int err;

	if (!sdata->use_key)
		return -EOPNOTSUPP;

	err = kstrtobool(buf, &swap);
	if (err)
		return err;

	mutex_lock(&sdata->key_mutex);

	if (sdata->key_swap != swap) {
		if (sdata->key_state) {
			input_report_key(sdata->input,
					 sdata->key_swap ? KEY_BACK : KEY_MENU,
					 0);
			input_report_key(sdata->input,
					 sdata->key_swap ? KEY_MENU : KEY_BACK,
					 0);
			input_sync(sdata->input);
			sdata->key_state = 0;
		}

		sdata->key_swap = swap;
	}

	mutex_unlock(&sdata->key_mutex);

	return count;
}

/*
 * Silence the two touch keys in the controller, which is what a device wants
 * when an on-screen navigation bar does their job instead.  The scan is
 * turned off in hardware rather than the events being dropped, so nothing is
 * spent scanning keys nobody reads.
 */
static ssize_t key_disable_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct stmfts_data *sdata = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%u\n", sdata->key_disable ? 1 : 0);
}

static ssize_t key_disable_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct stmfts_data *sdata = dev_get_drvdata(dev);
	bool disable;
	int err;

	if (!sdata->use_key)
		return -EOPNOTSUPP;

	err = kstrtobool(buf, &disable);
	if (err)
		return err;

	mutex_lock(&sdata->key_mutex);

	if (sdata->key_disable != disable) {
		err = stmfts_command(sdata, disable ? STMFTS_MS_KEY_SENSE_OFF :
						      STMFTS_MS_KEY_SENSE_ON);
		if (err) {
			mutex_unlock(&sdata->key_mutex);
			return err;
		}

		/*
		 * Turning the scan off while a key is held leaves the driver
		 * believing it is still down, and no release will ever come.
		 */
		if (disable && sdata->key_state) {
			input_report_key(sdata->input,
					 sdata->key_swap ? KEY_BACK : KEY_MENU,
					 0);
			input_report_key(sdata->input,
					 sdata->key_swap ? KEY_MENU : KEY_BACK,
					 0);
			input_sync(sdata->input);
			sdata->key_state = 0;
		}

		sdata->key_disable = disable;
	}

	mutex_unlock(&sdata->key_mutex);

	return count;
}

static DEVICE_ATTR_RW(key_disable);
static DEVICE_ATTR_RW(key_swap);
static DEVICE_ATTR_RO(chip_id);
static DEVICE_ATTR_RO(fw_ver);
static DEVICE_ATTR_RO(config_version);

static struct attribute *stmfts_sysfs_attrs[] = {
	&dev_attr_key_disable.attr,
	&dev_attr_key_swap.attr,
	&dev_attr_chip_id.attr,
	&dev_attr_fw_ver.attr,
	&dev_attr_config_version.attr,
	NULL
};
ATTRIBUTE_GROUPS(stmfts_sysfs);

/* probe */

static int stmfts_probe(struct i2c_client *client)
{
	struct stmfts_data *sdata;
	int err;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;

	sdata = devm_kzalloc(&client->dev, sizeof(*sdata), GFP_KERNEL);
	if (!sdata)
		return -ENOMEM;

	i2c_set_clientdata(client, sdata);
	sdata->client = client;
	mutex_init(&sdata->mutex);
	mutex_init(&sdata->key_mutex);

	sdata->avdd = devm_regulator_get(&client->dev, "avdd");
	if (IS_ERR(sdata->avdd))
		return dev_err_probe(&client->dev, PTR_ERR(sdata->avdd),
				     "failed to get avdd\n");

	sdata->vdd = devm_regulator_get(&client->dev, "vdd");
	if (IS_ERR(sdata->vdd))
		return dev_err_probe(&client->dev, PTR_ERR(sdata->vdd),
				     "failed to get vdd\n");

	sdata->input = devm_input_allocate_device(&client->dev);
	if (!sdata->input)
		return -ENOMEM;

	sdata->input->name = STMFTS_DEV_NAME;
	sdata->input->id.bustype = BUS_I2C;

	sdata->input->open = stmfts_input_open;
	sdata->input->close = stmfts_input_close;

	input_set_capability(sdata->input, EV_ABS, ABS_MT_POSITION_X);
	input_set_capability(sdata->input, EV_ABS, ABS_MT_POSITION_Y);
	touchscreen_parse_properties(sdata->input, true, &sdata->prop);

	input_set_abs_params(sdata->input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(sdata->input, ABS_MT_TOUCH_MINOR, 0, 255, 0, 0);
	input_set_abs_params(sdata->input, ABS_MT_PRESSURE, 0, 255, 0, 0);

	sdata->use_key = device_property_read_bool(&client->dev,
						   "touch-key-connected");
	if (sdata->use_key) {
		input_set_capability(sdata->input, EV_KEY, KEY_MENU);
		input_set_capability(sdata->input, EV_KEY, KEY_BACK);
	}

	/*
	 * No INPUT_MT_DROP_UNUSED: the controller only reports the contacts
	 * that changed, so slots left untouched in a frame are still down.
	 */
	err = input_mt_init_slots(sdata->input, STMFTS_MAX_FINGERS,
				  INPUT_MT_DIRECT);
	if (err)
		return err;

	input_set_drvdata(sdata->input, sdata);

	/*
	 * The interrupt is requested disabled and only enabled once the
	 * controller has been initialised: the initialisation is polled, and
	 * an interrupt arriving in the middle of it would read the event FIFO
	 * from under the polling loop.
	 */
	err = devm_request_threaded_irq(&client->dev, client->irq,
					NULL, stmfts_irq_handler,
					IRQF_ONESHOT | IRQF_NO_AUTOEN,
					"stmfts_irq", sdata);
	if (err)
		return err;

	/*
	 * The supplies are already on at this point, either from the
	 * bootloader or because regulator_ignore_unused kept them up, so
	 * start with a real power cycle to get a controller in a known state.
	 */
	err = stmfts_power_cycle(sdata, 100);
	if (err)
		return err;

	err = stmfts_initialise(sdata);
	if (err) {
		/*
		 * Never leave the bus with an unpowered controller on it:
		 * it would take down every other device on the same adapter.
		 */
		stmfts_power_on(sdata);
		return err;
	}

	err = input_register_device(sdata->input);
	if (err)
		return err;

	enable_irq(client->irq);

	err = stmfts_enable_interrupts(sdata);
	if (err)
		dev_err(&client->dev,
			"failed to enable the attention output: %d\n", err);

	return 0;
}

/*
 * Leave the controller in a state the next probe can start from.  Without
 * this it keeps sensing and keeps the attention output enabled after the
 * handler is gone: the FIFO fills, the line stays asserted, and the module
 * cannot simply be loaded again.
 */
static void stmfts_remove(struct i2c_client *client)
{
	struct stmfts_data *sdata = i2c_get_clientdata(client);
	int err;

	err = stmfts_write_reg(sdata, STMFTS_REG_INTERRUPT,
			       STMFTS_INTERRUPT_DISABLE);
	if (err)
		dev_warn(&client->dev,
			 "failed to disable the attention output: %d\n", err);

	disable_irq(client->irq);

	err = stmfts_command(sdata, STMFTS_MS_MT_SENSE_OFF);
	if (err)
		dev_warn(&client->dev, "failed to disable sensing: %d\n", err);

	if (sdata->use_key) {
		err = stmfts_command(sdata, STMFTS_MS_KEY_SENSE_OFF);
		if (err)
			dev_warn(&client->dev,
				 "failed to disable touchkeys: %d\n", err);
	}
}

static int stmfts_suspend(struct device *dev)
{
	struct stmfts_data *sdata = dev_get_drvdata(dev);
	int err;

	disable_irq(sdata->client->irq);

	err = stmfts_command(sdata, STMFTS_MS_MT_SENSE_OFF);
	if (err)
		dev_warn(dev, "failed to disable sensing: %d\n", err);

	if (sdata->use_key) {
		err = stmfts_command(sdata, STMFTS_MS_KEY_SENSE_OFF);
		if (err)
			dev_warn(dev, "failed to disable keys: %d\n", err);
	}

	return stmfts_command(sdata, STMFTS_SLEEP_IN);
}

static int stmfts_resume(struct device *dev)
{
	struct stmfts_data *sdata = dev_get_drvdata(dev);
	int err;

	err = stmfts_command(sdata, STMFTS_SLEEP_OUT);
	if (err) {
		dev_err(dev, "failed to wake the controller: %d\n", err);
		return err;
	}

	/*
	 * Sleep-out is not acknowledged with a ready event on this
	 * generation: the controller was never powered down, so it has no
	 * boot to announce.  Polling for one only delays re-enabling the
	 * interrupt and sensing below, which costs the first touch.
	 */
	stmfts_delay(20);

	stmfts_command(sdata, STMFTS_CLEAR_EVENT_STACK);
	stmfts_command(sdata, STMFTS_MS_MT_SENSE_ON);

	if (sdata->use_key)
		stmfts_command(sdata, STMFTS_MS_KEY_SENSE_ON);

	enable_irq(sdata->client->irq);

	err = stmfts_enable_interrupts(sdata);
	if (err)
		dev_warn(dev, "failed to re-enable interrupts: %d\n", err);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(stmfts_pm_ops, stmfts_suspend, stmfts_resume);

static const struct of_device_id stmfts_of_match[] = {
	{ .compatible = "st,stmfts", },
	{ },
};
MODULE_DEVICE_TABLE(of, stmfts_of_match);

static const struct i2c_device_id stmfts_id[] = {
	{ "stmfts" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, stmfts_id);

static struct i2c_driver stmfts_driver = {
	.driver = {
		.name = STMFTS_DEV_NAME,
		.dev_groups = stmfts_sysfs_groups,
		.of_match_table = stmfts_of_match,
		.pm = pm_sleep_ptr(&stmfts_pm_ops),
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.probe = stmfts_probe,
	.remove = stmfts_remove,
	.id_table = stmfts_id,
};

module_i2c_driver(stmfts_driver);

MODULE_AUTHOR("Andi Shyti <andi.shyti@samsung.com>");
MODULE_DESCRIPTION("STMicroelectronics FTS Touch Screen");
MODULE_LICENSE("GPL v2");
