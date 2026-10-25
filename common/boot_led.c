/*
 * The XIAO's RGB LED is three active-low GPIOs (led0 red, led1 green, led2
 * blue in the board devicetree). The animation is a red-green-blue chase run
 * twice, then a white flash, ~0.7 s in all. Each colour draws a few mA only
 * while it is lit; the pins are left driven inactive (high), which draws
 * nothing, including in System OFF.
 */
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include "boot_led.h"

LOG_MODULE_REGISTER(boot_led, LOG_LEVEL_INF);

#define FRAME_MS 70

#define R BIT(0)
#define G BIT(1)
#define B BIT(2)

static const struct gpio_dt_spec leds[] = {
	GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios),
	GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios),
	GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios),
};

/* One colour mask per frame; the last frame turns the LED off. */
static const uint8_t frames[] = {R, G, B, R, G, B, 0, R | G | B, 0};

static void frame_work_fn(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(frame_work, frame_work_fn);
static size_t frame;

static void show(uint8_t mask)
{
	for (size_t i = 0; i < ARRAY_SIZE(leds); i++) {
		gpio_pin_set_dt(&leds[i], (mask >> i) & 1);
	}
}

static void frame_work_fn(struct k_work *work)
{
	show(frames[frame]);
	if (++frame < ARRAY_SIZE(frames)) {
		k_work_reschedule(&frame_work, K_MSEC(FRAME_MS));
	}
}

void boot_led_play(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(leds); i++) {
		if (!gpio_is_ready_dt(&leds[i]) ||
		    gpio_pin_configure_dt(&leds[i], GPIO_OUTPUT_INACTIVE)) {
			LOG_WRN("LED %zu not available, no boot animation", i);
			return;
		}
	}
	frame = 0;
	k_work_reschedule(&frame_work, K_NO_WAIT);
}
