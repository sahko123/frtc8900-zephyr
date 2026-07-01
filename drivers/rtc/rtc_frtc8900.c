/*
 * Copyright (c) 2024 Every Watch
 * SPDX-License-Identifier: Apache-2.0
 *
 * Driver for the Nyfea FRTC8900 I2C real-time clock module.
 *
 * Datasheet: Nyfea FRTC8900 Product Specification v1.3 (2024-11-01)
 *
 * Key hardware notes:
 *  - All time/calendar values are in BCD format.
 *  - The WEEK register uses a bitmask (bit 0 = Sunday, bit 6 = Saturday),
 *    not a sequential 0–6 integer.
 *  - The /INT pin is open-drain, active low, shared by alarm, timer, and
 *    time-update interrupts. Read the flag register to identify the source.
 *  - Alarm AE bits are low-active: AE=0 means the field participates in
 *    comparison, AE=1 means the field is ignored.
 *  - VLF=1 after power loss — all registers must be reinitialised and time
 *    must be re-set before the RTC is usable.
 *  - Wait 30 ms after VDD reaches 2.5 V before first I2C access (tCL).
 *    On initial crystal startup, wait tSTA (≤3 s) before relying on time.
 */

#define DT_DRV_COMPAT nyfea_frtc8900

#include <zephyr/drivers/rtc.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(frtc8900, CONFIG_RTC_LOG_LEVEL);

/* ── Register addresses ────────────────────────────────────────────────── */
#define FRTC8900_REG_SEC         0x00
#define FRTC8900_REG_MIN         0x01
#define FRTC8900_REG_HOUR        0x02
#define FRTC8900_REG_WEEK        0x03
#define FRTC8900_REG_DAY         0x04
#define FRTC8900_REG_MONTH       0x05
#define FRTC8900_REG_YEAR        0x06
#define FRTC8900_REG_ALARM_MIN   0x08
#define FRTC8900_REG_ALARM_HOUR  0x09
#define FRTC8900_REG_ALARM_WDAY  0x0A
#define FRTC8900_REG_EXT         0x0D
#define FRTC8900_REG_FLAG        0x0E
#define FRTC8900_REG_CTRL        0x0F

/* ── Extension register (0x0D) bits ───────────────────────────────────── */
/* TEST (bit7) — always write 0; omitted here to prevent accidental use   */
#define FRTC8900_EXT_WADA        BIT(6) /* 0=week alarm, 1=day alarm      */
#define FRTC8900_EXT_USEL        BIT(5) /* 0=second update irq, 1=minute  */
#define FRTC8900_EXT_TE          BIT(4) /* fixed-cycle timer enable        */
#define FRTC8900_EXT_FSEL1       BIT(3) /* FOUT frequency select [1]       */
#define FRTC8900_EXT_FSEL0       BIT(2) /* FOUT frequency select [0]       */
#define FRTC8900_EXT_TSEL1       BIT(1) /* timer source clock select [1]   */
#define FRTC8900_EXT_TSEL0       BIT(0) /* timer source clock select [0]   */

/* ── Flag register (0x0E) bits ────────────────────────────────────────── */
#define FRTC8900_FLAG_UF         BIT(5) /* time update flag                */
#define FRTC8900_FLAG_TF         BIT(4) /* fixed-cycle timer flag          */
#define FRTC8900_FLAG_AF         BIT(3) /* alarm flag                      */
#define FRTC8900_FLAG_VLF        BIT(1) /* voltage low — time unreliable   */
#define FRTC8900_FLAG_VDET       BIT(0) /* voltage drop detected           */

/* ── Control register (0x0F) bits ────────────────────────────────────── */
#define FRTC8900_CTRL_CSEL1      BIT(7) /* compensation interval [1]       */
#define FRTC8900_CTRL_CSEL0      BIT(6) /* compensation interval [0]       */
#define FRTC8900_CTRL_UIE        BIT(5) /* time-update interrupt enable    */
#define FRTC8900_CTRL_TIE        BIT(4) /* timer interrupt enable          */
#define FRTC8900_CTRL_AIE        BIT(3) /* alarm interrupt enable          */
#define FRTC8900_CTRL_RESET      BIT(0) /* synchronised counter reset      */

/* CSEL=01 → 2 s compensation interval (datasheet default) */
#define FRTC8900_CTRL_CSEL_2S    FRTC8900_CTRL_CSEL0

/* ── Alarm register AE bit ────────────────────────────────────────────── */
/* AE=0 → field participates in comparison; AE=1 → field ignored          */
#define FRTC8900_ALARM_AE        BIT(7)

/* ── Alarm fields exposed via Zephyr RTC alarm API ───────────────────── */
#define FRTC8900_ALARM_SUPPORTED                                            \
	(RTC_ALARM_TIME_MASK_MINUTE   |                                     \
	 RTC_ALARM_TIME_MASK_HOUR     |                                     \
	 RTC_ALARM_TIME_MASK_MONTHDAY |                                     \
	 RTC_ALARM_TIME_MASK_WEEKDAY)

/* ── Driver structures ────────────────────────────────────────────────── */

struct frtc8900_config {
	struct i2c_dt_spec i2c;
#ifdef CONFIG_RTC_ALARM
	struct gpio_dt_spec int_gpio;
#endif
};

struct frtc8900_data {
	struct k_mutex lock;
#ifdef CONFIG_RTC_ALARM
	const struct device *dev; /* back-pointer for work handler */
	rtc_alarm_callback alarm_cb;
	void *alarm_cb_data;
	struct gpio_callback int_gpio_cb;
	struct k_work alarm_work;
#endif
};

/* ── BCD helpers ──────────────────────────────────────────────────────── */

static inline uint8_t bcd_to_bin(uint8_t bcd)
{
	return ((bcd >> 4) & 0x0f) * 10u + (bcd & 0x0fu);
}

static inline uint8_t bin_to_bcd(uint8_t bin)
{
	return ((bin / 10u) << 4) | (bin % 10u);
}

/* Convert the WEEK bitmask register to tm_wday (0=Sunday … 6=Saturday).
 * Returns the lowest set bit position, or 0 on empty mask.               */
static int week_bitmask_to_wday(uint8_t mask)
{
	mask &= 0x7f;
	for (int i = 0; i < 7; i++) {
		if (mask & BIT(i)) {
			return i;
		}
	}
	return 0;
}

/* ── Low-level I2C helpers ────────────────────────────────────────────── */

static int frtc8900_write_reg(const struct device *dev, uint8_t reg, uint8_t val)
{
	const struct frtc8900_config *cfg = dev->config;
	uint8_t buf[2] = {reg, val};

	return i2c_write_dt(&cfg->i2c, buf, sizeof(buf));
}

static int frtc8900_read_reg(const struct device *dev, uint8_t reg, uint8_t *val)
{
	const struct frtc8900_config *cfg = dev->config;

	return i2c_write_read_dt(&cfg->i2c, &reg, 1, val, 1);
}

static int frtc8900_read_regs(const struct device *dev, uint8_t reg,
			       uint8_t *buf, size_t len)
{
	const struct frtc8900_config *cfg = dev->config;

	return i2c_write_read_dt(&cfg->i2c, &reg, 1, buf, len);
}

/* Write a block starting at reg; buf[0] must be the register address.    */
static int frtc8900_write_block(const struct device *dev,
				uint8_t *buf, size_t len)
{
	const struct frtc8900_config *cfg = dev->config;

	return i2c_write_dt(&cfg->i2c, buf, len);
}

/* ── RTC API: set_time ────────────────────────────────────────────────── */

static int frtc8900_set_time(const struct device *dev,
			     const struct rtc_time *timeptr)
{
	struct frtc8900_data *data = dev->data;
	int ret;
	/* reg + SEC MIN HOUR WEEK DAY MONTH YEAR = 8 bytes */
	uint8_t buf[8];

	/* RTC stores years 00–99 relative to 2000; tm_year is since 1900 */
	if (timeptr->tm_year < 100 || timeptr->tm_year > 199) {
		return -EINVAL;
	}
	if (timeptr->tm_wday < 0 || timeptr->tm_wday > 6) {
		return -EINVAL;
	}

	buf[0] = FRTC8900_REG_SEC;
	buf[1] = bin_to_bcd(timeptr->tm_sec);
	buf[2] = bin_to_bcd(timeptr->tm_min);
	buf[3] = bin_to_bcd(timeptr->tm_hour);
	buf[4] = BIT(timeptr->tm_wday); /* WEEK bitmask, not sequential */
	buf[5] = bin_to_bcd(timeptr->tm_mday);
	buf[6] = bin_to_bcd(timeptr->tm_mon + 1); /* tm_mon 0-11 → RTC 1-12 */
	buf[7] = bin_to_bcd(timeptr->tm_year - 100);

	k_mutex_lock(&data->lock, K_FOREVER);
	ret = frtc8900_write_block(dev, buf, sizeof(buf));
	if (ret == 0) {
		/* Clear VLF (and stale flags) now that a valid time has been set */
		ret = frtc8900_write_reg(dev, FRTC8900_REG_FLAG, 0x00);
	}
	k_mutex_unlock(&data->lock);

	return ret;
}

/* ── RTC API: get_time ────────────────────────────────────────────────── */

static int frtc8900_get_time(const struct device *dev, struct rtc_time *timeptr)
{
	struct frtc8900_data *data = dev->data;
	uint8_t regs[7];
	uint8_t flags;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);

	ret = frtc8900_read_reg(dev, FRTC8900_REG_FLAG, &flags);
	if (ret < 0) {
		goto out;
	}

	/* VLF=1: oscillator stopped or supply dipped below 1.6 V — time invalid */
	if (flags & FRTC8900_FLAG_VLF) {
		LOG_WRN("VLF set: time data invalid, set time before use");
		ret = -ENODATA;
		goto out;
	}

	ret = frtc8900_read_regs(dev, FRTC8900_REG_SEC, regs, sizeof(regs));
	if (ret < 0) {
		goto out;
	}

	timeptr->tm_sec   = bcd_to_bin(regs[0] & 0x7fu);
	timeptr->tm_min   = bcd_to_bin(regs[1] & 0x7fu);
	timeptr->tm_hour  = bcd_to_bin(regs[2] & 0x3fu);
	timeptr->tm_wday  = week_bitmask_to_wday(regs[3]);
	timeptr->tm_mday  = bcd_to_bin(regs[4] & 0x3fu);
	timeptr->tm_mon   = bcd_to_bin(regs[5] & 0x1fu) - 1;
	timeptr->tm_year  = bcd_to_bin(regs[6]) + 100;
	timeptr->tm_yday  = -1;
	timeptr->tm_isdst = -1;
	timeptr->tm_nsec  = 0;

out:
	k_mutex_unlock(&data->lock);
	return ret;
}

/* ── RTC ALARM API ────────────────────────────────────────────────────── */

#ifdef CONFIG_RTC_ALARM

static int frtc8900_alarm_get_supported_fields(const struct device *dev,
					       uint16_t id, uint16_t *mask)
{
	ARG_UNUSED(dev);

	if (id != 0) {
		return -EINVAL;
	}

	*mask = FRTC8900_ALARM_SUPPORTED;
	return 0;
}

static int frtc8900_alarm_set_time(const struct device *dev, uint16_t id,
				   uint16_t mask, const struct rtc_time *timeptr)
{
	struct frtc8900_data *data = dev->data;
	uint8_t min_alarm, hour_alarm, wday_alarm;
	uint8_t ext;
	uint8_t buf[4];
	int ret;

	if (id != 0) {
		return -EINVAL;
	}
	if (mask & ~FRTC8900_ALARM_SUPPORTED) {
		return -EINVAL;
	}
	/* MONTHDAY and WEEKDAY are mutually exclusive on this hardware */
	if ((mask & RTC_ALARM_TIME_MASK_MONTHDAY) &&
	    (mask & RTC_ALARM_TIME_MASK_WEEKDAY)) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	ret = frtc8900_read_reg(dev, FRTC8900_REG_EXT, &ext);
	if (ret < 0) {
		goto out;
	}

	/* AE=0 → field active in comparison; AE=1 → field ignored */
	min_alarm = (mask & RTC_ALARM_TIME_MASK_MINUTE)
		? bin_to_bcd(timeptr->tm_min) & 0x7fu
		: FRTC8900_ALARM_AE;

	hour_alarm = (mask & RTC_ALARM_TIME_MASK_HOUR)
		? bin_to_bcd(timeptr->tm_hour) & 0x3fu
		: FRTC8900_ALARM_AE;

	if (mask & RTC_ALARM_TIME_MASK_MONTHDAY) {
		wday_alarm = bin_to_bcd(timeptr->tm_mday) & 0x3fu;
		ext |= FRTC8900_EXT_WADA; /* day-of-month alarm mode */
	} else if (mask & RTC_ALARM_TIME_MASK_WEEKDAY) {
		wday_alarm = BIT(timeptr->tm_wday) & 0x7fu;
		ext &= ~FRTC8900_EXT_WADA; /* day-of-week alarm mode */
	} else {
		wday_alarm = FRTC8900_ALARM_AE;
	}

	buf[0] = FRTC8900_REG_ALARM_MIN;
	buf[1] = min_alarm;
	buf[2] = hour_alarm;
	buf[3] = wday_alarm;

	ret = frtc8900_write_block(dev, buf, sizeof(buf));
	if (ret < 0) {
		goto out;
	}

	/* TEST bit must always be 0 */
	ret = frtc8900_write_reg(dev, FRTC8900_REG_EXT, ext & ~BIT(7));

out:
	k_mutex_unlock(&data->lock);
	return ret;
}

static int frtc8900_alarm_get_time(const struct device *dev, uint16_t id,
				   uint16_t *mask, struct rtc_time *timeptr)
{
	struct frtc8900_data *data = dev->data;
	uint8_t regs[3];
	uint8_t ext;
	int ret;

	if (id != 0) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	ret = frtc8900_read_regs(dev, FRTC8900_REG_ALARM_MIN, regs, sizeof(regs));
	if (ret < 0) {
		goto out;
	}

	ret = frtc8900_read_reg(dev, FRTC8900_REG_EXT, &ext);
	if (ret < 0) {
		goto out;
	}

	*mask = 0;
	memset(timeptr, 0, sizeof(*timeptr));

	if (!(regs[0] & FRTC8900_ALARM_AE)) {
		timeptr->tm_min = bcd_to_bin(regs[0] & 0x7fu);
		*mask |= RTC_ALARM_TIME_MASK_MINUTE;
	}

	if (!(regs[1] & FRTC8900_ALARM_AE)) {
		timeptr->tm_hour = bcd_to_bin(regs[1] & 0x3fu);
		*mask |= RTC_ALARM_TIME_MASK_HOUR;
	}

	if (!(regs[2] & FRTC8900_ALARM_AE)) {
		if (ext & FRTC8900_EXT_WADA) {
			timeptr->tm_mday = bcd_to_bin(regs[2] & 0x3fu);
			*mask |= RTC_ALARM_TIME_MASK_MONTHDAY;
		} else {
			timeptr->tm_wday = week_bitmask_to_wday(regs[2]);
			*mask |= RTC_ALARM_TIME_MASK_WEEKDAY;
		}
	}

out:
	k_mutex_unlock(&data->lock);
	return ret;
}

static int frtc8900_alarm_is_pending(const struct device *dev, uint16_t id)
{
	struct frtc8900_data *data = dev->data;
	uint8_t flags;
	int ret;

	if (id != 0) {
		return -EINVAL;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	ret = frtc8900_read_reg(dev, FRTC8900_REG_FLAG, &flags);
	if (ret < 0) {
		goto out;
	}

	if (!(flags & FRTC8900_FLAG_AF)) {
		ret = 0;
		goto out;
	}

	/* Clear AF — only 0 can be written to flag bits */
	ret = frtc8900_write_reg(dev, FRTC8900_REG_FLAG,
				 flags & ~FRTC8900_FLAG_AF);
	if (ret == 0) {
		ret = 1;
	}

out:
	k_mutex_unlock(&data->lock);
	return ret;
}

static void frtc8900_alarm_work_handler(struct k_work *work)
{
	struct frtc8900_data *data =
		CONTAINER_OF(work, struct frtc8900_data, alarm_work);
	const struct device *dev = data->dev;
	rtc_alarm_callback cb;
	void *cb_data;
	uint8_t flags;
	int ret;

	k_mutex_lock(&data->lock, K_FOREVER);

	ret = frtc8900_read_reg(dev, FRTC8900_REG_FLAG, &flags);
	if (ret < 0 || !(flags & FRTC8900_FLAG_AF)) {
		k_mutex_unlock(&data->lock);
		return;
	}

	/* Clear AF before releasing the lock */
	frtc8900_write_reg(dev, FRTC8900_REG_FLAG, flags & ~FRTC8900_FLAG_AF);

	/* Snapshot callback under the lock, invoke after — avoids deadlock if
	 * the callback itself calls back into the RTC API */
	cb      = data->alarm_cb;
	cb_data = data->alarm_cb_data;

	k_mutex_unlock(&data->lock);

	if (cb) {
		cb(dev, 0, cb_data);
	}
}

static void frtc8900_int_gpio_cb(const struct device *gpio_dev,
				 struct gpio_callback *cb, uint32_t pins)
{
	struct frtc8900_data *data =
		CONTAINER_OF(cb, struct frtc8900_data, int_gpio_cb);

	ARG_UNUSED(gpio_dev);
	ARG_UNUSED(pins);

	/* Defer to work queue — I2C must not be called from ISR context */
	k_work_submit(&data->alarm_work);
}

static int frtc8900_alarm_set_callback(const struct device *dev, uint16_t id,
				       rtc_alarm_callback cb, void *user_data)
{
	const struct frtc8900_config *cfg = dev->config;
	struct frtc8900_data *data = dev->data;
	uint8_t ctrl;
	int ret;

	if (id != 0) {
		return -EINVAL;
	}

	if (!cfg->int_gpio.port) {
		return -ENOTSUP;
	}

	k_mutex_lock(&data->lock, K_FOREVER);

	data->alarm_cb      = cb;
	data->alarm_cb_data = user_data;

	ret = frtc8900_read_reg(dev, FRTC8900_REG_CTRL, &ctrl);
	if (ret < 0) {
		goto out;
	}

	if (cb) {
		ctrl |= FRTC8900_CTRL_AIE;
	} else {
		ctrl &= ~FRTC8900_CTRL_AIE;
	}

	ret = frtc8900_write_reg(dev, FRTC8900_REG_CTRL, ctrl);

out:
	k_mutex_unlock(&data->lock);
	return ret;
}

#endif /* CONFIG_RTC_ALARM */

/* ── Driver init ──────────────────────────────────────────────────────── */

static int frtc8900_init(const struct device *dev)
{
	const struct frtc8900_config *cfg = dev->config;
	struct frtc8900_data *data = dev->data;
	uint8_t flags;
	int ret;

	k_mutex_init(&data->lock);

	if (!i2c_is_ready_dt(&cfg->i2c)) {
		LOG_ERR("I2C bus not ready");
		return -ENODEV;
	}

	ret = frtc8900_read_reg(dev, FRTC8900_REG_FLAG, &flags);
	if (ret < 0) {
		LOG_ERR("Failed to read flag register: %d", ret);
		return ret;
	}

	if (flags & FRTC8900_FLAG_VLF) {
		LOG_WRN("VLF set after power loss — call rtc_set_time() before use");
	}

	/* Clear all flag bits (datasheet: only 0 can be written to these bits) */
	ret = frtc8900_write_reg(dev, FRTC8900_REG_FLAG, 0x00);
	if (ret < 0) {
		return ret;
	}

	/* Extension register defaults:
	 *   TEST=0  (always), WADA=1 (day alarm), USEL=0 (second update irq),
	 *   TE=0    (timer off), FSEL=00 (32768 Hz FOUT), TSEL=00            */
	ret = frtc8900_write_reg(dev, FRTC8900_REG_EXT, FRTC8900_EXT_WADA);
	if (ret < 0) {
		return ret;
	}

	/* Control register: CSEL=01 (2 s compensation), all interrupts off */
	ret = frtc8900_write_reg(dev, FRTC8900_REG_CTRL, FRTC8900_CTRL_CSEL_2S);
	if (ret < 0) {
		return ret;
	}

#ifdef CONFIG_RTC_ALARM
	data->dev = dev;
	k_work_init(&data->alarm_work, frtc8900_alarm_work_handler);

	if (cfg->int_gpio.port) {
		if (!gpio_is_ready_dt(&cfg->int_gpio)) {
			LOG_ERR("INT GPIO not ready");
			return -ENODEV;
		}

		ret = gpio_pin_configure_dt(&cfg->int_gpio, GPIO_INPUT);
		if (ret < 0) {
			return ret;
		}

		gpio_init_callback(&data->int_gpio_cb, frtc8900_int_gpio_cb,
				   BIT(cfg->int_gpio.pin));

		ret = gpio_add_callback(cfg->int_gpio.port, &data->int_gpio_cb);
		if (ret < 0) {
			return ret;
		}

		ret = gpio_pin_interrupt_configure_dt(&cfg->int_gpio,
						      GPIO_INT_EDGE_FALLING);
		if (ret < 0) {
			return ret;
		}
	}
#endif

	return 0;
}

/* ── Driver API table ─────────────────────────────────────────────────── */

static const struct rtc_driver_api frtc8900_api = {
	.set_time = frtc8900_set_time,
	.get_time = frtc8900_get_time,
#ifdef CONFIG_RTC_ALARM
	.alarm_get_supported_fields = frtc8900_alarm_get_supported_fields,
	.alarm_set_time             = frtc8900_alarm_set_time,
	.alarm_get_time             = frtc8900_alarm_get_time,
	.alarm_is_pending           = frtc8900_alarm_is_pending,
	.alarm_set_callback         = frtc8900_alarm_set_callback,
#endif
};

/* ── Instance registration ────────────────────────────────────────────── */

#ifdef CONFIG_RTC_ALARM
#define FRTC8900_INT_GPIO_INIT(n) \
	.int_gpio = GPIO_DT_SPEC_INST_GET_OR(n, int_gpios, {0}),
#else
#define FRTC8900_INT_GPIO_INIT(n)
#endif

#define FRTC8900_INIT(n)                                                    \
	static struct frtc8900_data frtc8900_data_##n;                      \
                                                                            \
	static const struct frtc8900_config frtc8900_config_##n = {        \
		.i2c = I2C_DT_SPEC_INST_GET(n),                            \
		FRTC8900_INT_GPIO_INIT(n)                                   \
	};                                                                  \
                                                                            \
	DEVICE_DT_INST_DEFINE(n, frtc8900_init, NULL,                       \
			      &frtc8900_data_##n,                           \
			      &frtc8900_config_##n,                         \
			      POST_KERNEL,                                  \
			      CONFIG_RTC_INIT_PRIORITY,                     \
			      &frtc8900_api);

DT_INST_FOREACH_STATUS_OKAY(FRTC8900_INIT)
