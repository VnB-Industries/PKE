/*
 * Idle clock and System OFF for the car unit.
 *
 * The wake input (wake-gpios, D14 on the carte_uwb board; active low with the
 * internal pull-up) takes a push button to GND, or the driver's jamb-switch
 * line through a diode: opening the door grounds the line and pulls the input
 * low, while the diode blocks the line's battery voltage when the door is
 * closed. CAN bus activity can wake it too (BUS_WAKE_ENABLED,
 * door_arm_bus_wake()). Outside a session the MCP2515 is already stopped, the
 * mic unclocked and the relay released, so System OFF only has to stop the
 * radio and the CPU.
 */
#include <errno.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/sys/poweroff.h>

#include "door.h"
#include "power.h"

LOG_MODULE_REGISTER(power, LOG_LEVEL_INF);

static const struct gpio_dt_spec wake = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), wake_gpios);

static struct gpio_callback wake_cb;
static K_SEM_DEFINE(wake_sem, 0, 1);
/* Something can bring the unit back from System OFF: the wake input, the CAN bus, or both. */
static bool wake_ok;
static bool input_ok;
static bool bus_wake;
static int64_t last_activity_ms;

static void wake_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	k_sem_give(&wake_sem);
}

int power_init(bool can_ok)
{
	int err;

	last_activity_ms = k_uptime_get();
	bus_wake = BUS_WAKE_ENABLED && can_ok;
	wake_ok = bus_wake;
	if (!gpio_is_ready_dt(&wake)) {
		return -ENODEV;
	}
	err = gpio_pin_configure_dt(&wake, GPIO_INPUT);
	if (err) {
		return err;
	}
	gpio_init_callback(&wake_cb, wake_isr, BIT(wake.pin));
	err = gpio_add_callback_dt(&wake, &wake_cb);
	if (!err) {
		err = gpio_pin_interrupt_configure_dt(&wake, GPIO_INT_EDGE_TO_ACTIVE);
	}
	if (err) {
		return err;
	}
	input_ok = true;
	wake_ok = true;
	LOG_INF("deep idle after %d h without a session, System OFF after %d h",
		POWER_DEEP_IDLE_AFTER_S / 3600, POWER_OFF_AFTER_S / 3600);
	return 0;
}

void power_note_activity(void)
{
	last_activity_ms = k_uptime_get();
}

enum power_stage power_stage(void)
{
	int64_t idle_ms;

	if (k_sem_take(&wake_sem, K_NO_WAIT) == 0) {
		LOG_INF("wake input pressed");
		power_note_activity();
	}
	idle_ms = k_uptime_get() - last_activity_ms;
	if (wake_ok && idle_ms >= (int64_t)POWER_OFF_AFTER_S * MSEC_PER_SEC) {
		return POWER_OFF;
	}
	if (idle_ms >= (int64_t)POWER_DEEP_IDLE_AFTER_S * MSEC_PER_SEC) {
		return POWER_DEEP_IDLE;
	}
	return POWER_NORMAL;
}

k_timeout_t power_until_deep_idle(void)
{
	int64_t left_ms = last_activity_ms + (int64_t)POWER_DEEP_IDLE_AFTER_S * MSEC_PER_SEC -
			  k_uptime_get();

	return K_MSEC(MAX(left_ms, 0));
}

bool power_wait_wake(k_timeout_t timeout)
{
	if (k_sem_take(&wake_sem, timeout)) {
		return false;
	}
	LOG_INF("wake input pressed");
	power_note_activity();
	return true;
}

void power_off(void)
{
	if (bus_wake) {
		int err = door_arm_bus_wake();

		if (err) {
			LOG_ERR("CAN bus wake failed (err %d)", err);
			bus_wake = false;
			wake_ok = input_ok;
			if (!wake_ok) {
				LOG_ERR("nothing could wake the unit, staying in deep idle");
				return;
			}
		}
	}
	if (input_ok) {
		/* A level interrupt sets the pin's SENSE, which is what wakes the chip from System OFF. */
		gpio_pin_interrupt_configure_dt(&wake, GPIO_INT_LEVEL_ACTIVE);
	}
	LOG_INF("no fob session for %d h, entering System OFF until %s", POWER_OFF_AFTER_S / 3600,
		input_ok && bus_wake ? "the wake input or CAN activity"
		: bus_wake	     ? "CAN activity"
				     : "the wake input");
	LOG_PANIC();
	sys_poweroff();
}
