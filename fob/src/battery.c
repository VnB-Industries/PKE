/*
 * The XIAO's BQ25101 pulls CHG low while it charges the cell. Its STAT line
 * stays high both when charging is done and when there is no USB power, so a
 * high pin alone can't tell those apart; the voltage gives the rest.
 *
 * BAT+ reaches AIN7 through a 1M/510k divider, which P0.14 switches on.
 */
#include <zephyr/kernel.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include "battery.h"

LOG_MODULE_REGISTER(battery, LOG_LEVEL_INF);

#define USER_NODE DT_PATH(zephyr_user)

/* Divider ratio (1M + 510k) / 510k, as numerator and denominator. */
#define DIVIDER_NUM 1510
#define DIVIDER_DEN 510

static const struct gpio_dt_spec chg = GPIO_DT_SPEC_GET(USER_NODE, chg_gpios);
static const struct gpio_dt_spec vbat_en = GPIO_DT_SPEC_GET(USER_NODE, vbat_en_gpios);
static const struct adc_dt_spec vbat = ADC_DT_SPEC_GET(USER_NODE);

static int read_vbat_mv(int32_t *mv)
{
	int16_t raw;
	struct adc_sequence seq = {
		.buffer = &raw,
		.buffer_size = sizeof(raw),
	};
	int err;

	err = adc_channel_setup_dt(&vbat);
	if (err) {
		return err;
	}
	adc_sequence_init_dt(&vbat, &seq);

	err = gpio_pin_configure_dt(&vbat_en, GPIO_OUTPUT_ACTIVE);
	if (err) {
		return err;
	}
	/* Let the high-impedance divider settle on the sampling cap. */
	k_msleep(1);
	err = adc_read_dt(&vbat, &seq);
	gpio_pin_configure_dt(&vbat_en, GPIO_INPUT);
	if (err) {
		return err;
	}

	*mv = raw;
	err = adc_raw_to_millivolts_dt(&vbat, mv);
	if (err) {
		return err;
	}
	*mv = *mv * DIVIDER_NUM / DIVIDER_DEN;
	return 0;
}

void battery_log_state(void)
{
	int32_t mv;
	int charging;
	int err;

	err = gpio_pin_configure_dt(&chg, GPIO_INPUT);
	charging = err ? err : gpio_pin_get_dt(&chg);
	gpio_pin_configure_dt(&chg, GPIO_DISCONNECTED);

	err = read_vbat_mv(&mv);
	if (err) {
		LOG_ERR("battery read failed (err %d), charging %d", err, charging);
		return;
	}
	LOG_INF("battery %d mV, %s", mv, charging > 0 ? "charging" : "not charging");
}
