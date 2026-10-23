/*
 * ADXL362 motion switch over raw SPI registers.
 *
 * The chip runs in wake-up mode (~270 nA, ~6 samples/s) with referenced
 * activity and inactivity detection in loop mode, and the AWAKE bit mapped to
 * INT1: INT1 goes high when the fob starts moving and low a few seconds after
 * it stops. Loop mode acknowledges the interrupts itself, so nothing has to
 * read STATUS, and the chip keeps doing this while the nRF52840 is in System OFF.
 */
#include <errno.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/poweroff.h>

#include "motion.h"

LOG_MODULE_REGISTER(motion, LOG_LEVEL_INF);

#define ADXL_CMD_WRITE 0x0A
#define ADXL_CMD_READ 0x0B

#define ADXL_REG_DEVID_AD 0x00
#define ADXL_REG_PARTID 0x02
#define ADXL_REG_XDATA_L 0x0E
#define ADXL_REG_SOFT_RESET 0x1F
#define ADXL_REG_THRESH_ACT_L 0x20
#define ADXL_REG_THRESH_INACT_L 0x23
#define ADXL_REG_TIME_INACT_L 0x25
#define ADXL_REG_ACT_INACT_CTL 0x27
#define ADXL_REG_INTMAP1 0x2A
#define ADXL_REG_POWER_CTL 0x2D

#define ADXL_DEVID_AD 0xAD
#define ADXL_PARTID 0xF2
#define ADXL_RESET_KEY 0x52

/* ACT_EN | ACT_REF | INACT_EN | INACT_REF | LINKLOOP = loop. */
#define ADXL_ACT_INACT_LOOP 0x3F
#define ADXL_INTMAP_AWAKE 0x40
/* WAKEUP | MEASURE = measurement. */
#define ADXL_POWER_WAKEUP_MEASURE 0x0A

/* In mg at the default ±2 g range (1 mg/LSB): above 250 mg is motion, under 150 mg is rest. */
#define ACT_THRESHOLD_MG 250
#define INACT_THRESHOLD_MG 150
/* Samples of rest before INT1 drops, ~5 s at the wake-up mode rate. */
#define INACT_SAMPLES 30

#define ADXL_NODE DT_NODELABEL(adxl362)

static const struct spi_dt_spec adxl_spi =
	SPI_DT_SPEC_GET(ADXL_NODE, SPI_OP_MODE_MASTER | SPI_WORD_SET(8) | SPI_TRANSFER_MSB);
static const struct gpio_dt_spec int1 = GPIO_DT_SPEC_GET(ADXL_NODE, int1_gpios);

static struct gpio_callback int1_cb;
static void (*idle_handler)(void);

static void idle_expired(struct k_work *work)
{
	idle_handler();
}

static K_WORK_DELAYABLE_DEFINE(idle_work, idle_expired);

static int adxl_write(uint8_t reg, const uint8_t *val, size_t len)
{
	uint8_t hdr[2] = {ADXL_CMD_WRITE, reg};
	const struct spi_buf bufs[] = {
		{.buf = hdr, .len = sizeof(hdr)},
		{.buf = (void *)val, .len = len},
	};
	const struct spi_buf_set tx = {.buffers = bufs, .count = ARRAY_SIZE(bufs)};

	return spi_write_dt(&adxl_spi, &tx);
}

static int adxl_write_u8(uint8_t reg, uint8_t val)
{
	return adxl_write(reg, &val, 1);
}

/* 11-bit threshold or 16-bit time, low byte first. */
static int adxl_write_u16(uint8_t reg, uint16_t val)
{
	uint8_t le[2] = {val & 0xff, val >> 8};

	return adxl_write(reg, le, sizeof(le));
}

static int adxl_read(uint8_t reg, uint8_t *val, size_t len)
{
	uint8_t hdr[2] = {ADXL_CMD_READ, reg};
	const struct spi_buf tx_buf = {.buf = hdr, .len = sizeof(hdr)};
	const struct spi_buf rx_bufs[] = {
		{.buf = NULL, .len = sizeof(hdr)},
		{.buf = val, .len = len},
	};
	const struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};
	const struct spi_buf_set rx = {.buffers = rx_bufs, .count = ARRAY_SIZE(rx_bufs)};

	return spi_transceive_dt(&adxl_spi, &tx, &rx);
}

static int adxl_read_u8(uint8_t reg, uint8_t *val)
{
	return adxl_read(reg, val, 1);
}

static int adxl_configure(void)
{
	uint8_t devid, partid;
	int err;

	/* Start from reset values whether this is a cold boot or a wake from System OFF. */
	err = adxl_write_u8(ADXL_REG_SOFT_RESET, ADXL_RESET_KEY);
	if (err) {
		return err;
	}
	k_msleep(1);

	err = adxl_read_u8(ADXL_REG_DEVID_AD, &devid);
	if (!err) {
		err = adxl_read_u8(ADXL_REG_PARTID, &partid);
	}
	if (err) {
		return err;
	}
	if (devid != ADXL_DEVID_AD || partid != ADXL_PARTID) {
		LOG_ERR("no ADXL362 (id %02x/%02x)", devid, partid);
		return -ENODEV;
	}

	err = adxl_write_u16(ADXL_REG_THRESH_ACT_L, ACT_THRESHOLD_MG);
	if (!err) {
		err = adxl_write_u16(ADXL_REG_THRESH_INACT_L, INACT_THRESHOLD_MG);
	}
	if (!err) {
		err = adxl_write_u16(ADXL_REG_TIME_INACT_L, INACT_SAMPLES);
	}
	if (!err) {
		err = adxl_write_u8(ADXL_REG_ACT_INACT_CTL, ADXL_ACT_INACT_LOOP);
	}
	if (!err) {
		err = adxl_write_u8(ADXL_REG_INTMAP1, ADXL_INTMAP_AWAKE);
	}
	if (!err) {
		/* Last: measurement starts with everything above in place. */
		err = adxl_write_u8(ADXL_REG_POWER_CTL, ADXL_POWER_WAKEUP_MEASURE);
	}
	return err;
}

/*
 * Debug: while INT1 stays high, reads XYZ every 50 ms (faster than the ~6 Hz
 * wake-up rate, so every chip sample is seen) and logs each axis' min/max once
 * a second. A span above INACT_THRESHOLD_MG restarts the chip's inactivity count.
 */
#define DUMP_PERIOD_MS 50
#define DUMP_READS 20

static void dump_expired(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(dump_work, dump_expired);

static void dump_expired(struct k_work *work)
{
	static int16_t lo[3], hi[3];
	static int reads;
	uint8_t raw[6];

	if (gpio_pin_get_dt(&int1) <= 0) {
		reads = 0;
		return;
	}
	if (!adxl_read(ADXL_REG_XDATA_L, raw, sizeof(raw))) {
		for (int i = 0; i < 3; i++) {
			int16_t v = sys_get_le16(&raw[2 * i]);

			if (reads == 0 || v < lo[i]) {
				lo[i] = v;
			}
			if (reads == 0 || v > hi[i]) {
				hi[i] = v;
			}
		}
		if (++reads == DUMP_READS) {
			LOG_INF("x %d..%d (%d) y %d..%d (%d) z %d..%d (%d) mg",
				lo[0], hi[0], hi[0] - lo[0], lo[1], hi[1], hi[1] - lo[1],
				lo[2], hi[2], hi[2] - lo[2]);
			reads = 0;
		}
	}
	k_work_reschedule(&dump_work, K_MSEC(DUMP_PERIOD_MS));
}

/* Both edges count: the fob starting to move, and coming to rest. */
static void int1_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	LOG_INF("ADXL362 INT1: %s", gpio_pin_get_dt(&int1) > 0 ? "moving" : "at rest");
	k_work_reschedule(&dump_work, K_NO_WAIT);
	k_work_reschedule(&idle_work, K_SECONDS(MOTION_IDLE_TIMEOUT_S));
}

int motion_init(void (*on_idle)(void))
{
	int err;

	if (!spi_is_ready_dt(&adxl_spi) || !gpio_is_ready_dt(&int1)) {
		return -ENODEV;
	}
	err = adxl_configure();
	if (err) {
		return err;
	}

	err = gpio_pin_configure_dt(&int1, GPIO_INPUT);
	if (err) {
		return err;
	}
	gpio_init_callback(&int1_cb, int1_isr, BIT(int1.pin));
	err = gpio_add_callback_dt(&int1, &int1_cb);
	if (!err) {
		err = gpio_pin_interrupt_configure_dt(&int1, GPIO_INT_EDGE_BOTH);
	}
	if (err) {
		return err;
	}

	idle_handler = on_idle;
	k_work_reschedule(&idle_work, K_SECONDS(MOTION_IDLE_TIMEOUT_S));
	LOG_INF("ADXL362 motion switch on, System OFF after %d s at rest",
		MOTION_IDLE_TIMEOUT_S);
	return 0;
}

bool motion_active(void)
{
	return gpio_pin_get_dt(&int1) > 0;
}

void motion_defer(void)
{
	k_work_reschedule(&idle_work, K_SECONDS(MOTION_IDLE_TIMEOUT_S));
}

void motion_poweroff(void)
{
	/* A level interrupt sets the pin's SENSE, which is what wakes the chip from System OFF. */
	gpio_pin_interrupt_configure_dt(&int1, GPIO_INT_LEVEL_ACTIVE);
	LOG_INF("at rest for %d s, entering System OFF",
		MOTION_IDLE_TIMEOUT_S);
	LOG_PANIC();
	sys_poweroff();
}
