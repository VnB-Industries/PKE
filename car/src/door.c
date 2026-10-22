/*
 * Door frames over CAN through the MCP2515, ported from the ESP32 PKE.ino
 * (branch ESP32/main). The measurements behind each choice are in the README,
 * "Waking a sleeping bus".
 *
 * The controller stays in normal mode and receives every standard frame from
 * its interrupt thread: ACKing the gateway's NM frames keeps the responsive
 * window usable, and the last frame seen tells whether the bus is awake
 * without having to transmit and fail.
 */
#include <errno.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/can.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

#include "door.h"
#include "door_config.h"

LOG_MODULE_REGISTER(door, LOG_LEVEL_INF);

static const struct device *const can_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));
#if RELAY_WAKE_ENABLED
static const struct gpio_dt_spec relay = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), relay_gpios);
#endif

static K_MUTEX_DEFINE(door_mutex);
static K_SEM_DEFINE(tx_sem, 0, 1);
static K_SEM_DEFINE(reply_sem, 0, 1);
static K_SEM_DEFINE(rx_sem, 0, 1);

static bool can_ready;
static int filter_id = -1;
static int tx_err;
static uint8_t reply[4];

/* Uptime of the last frame received from any other node; rx_seen stays 0 until the first one. */
static atomic_t last_rx_ms;
static atomic_t rx_seen;

static void rx_cb(const struct device *dev, struct can_frame *frame, void *user_data)
{
	atomic_set(&last_rx_ms, (atomic_val_t)k_uptime_get_32());
	atomic_set(&rx_seen, 1);
	k_sem_give(&rx_sem);

	/* Positive response to service 0x30 (0x30 + 0x40), echoing LID 0x11. */
	if (frame->id == DOOR_RESPONSE_ID && can_dlc_to_bytes(frame->dlc) >= 4 &&
	    frame->data[2] == 0x70) {
		memcpy(reply, frame->data, sizeof(reply));
		k_sem_give(&reply_sem);
	}
}

static void tx_cb(const struct device *dev, int error, void *user_data)
{
	tx_err = error;
	k_sem_give(&tx_sem);
}

static bool bus_awake(void)
{
	return atomic_get(&rx_seen) &&
	       (k_uptime_get_32() - (uint32_t)atomic_get(&last_rx_ms)) < BUS_AWAKE_TIMEOUT_MS;
}

static int start_can(void)
{
	/* Standard IDs only, mask 0: every frame counts as bus activity. */
	const struct can_filter all = {.id = 0, .mask = 0, .flags = 0};
	int err;

	if (!device_is_ready(can_dev)) {
		LOG_ERR("MCP2515 not ready");
		return -ENODEV;
	}
	err = can_set_mode(can_dev, CAN_MODE_NORMAL);
	if (!err) {
		err = can_start(can_dev);
	}
	if (err && err != -EALREADY) {
		LOG_ERR("CAN start failed (err %d)", err);
		return err;
	}
	/* Filters survive a stop/start, so a retried start must not add a second one. */
	if (filter_id < 0) {
		filter_id = can_add_rx_filter(can_dev, rx_cb, NULL, &all);
		if (filter_id < 0) {
			LOG_ERR("CAN filter failed (err %d)", filter_id);
			return filter_id;
		}
	}
	LOG_INF("MCP2515 on-bus");
	return 0;
}

/* Stopping the controller aborts every pending frame, and starting it again returns to normal mode. */
static void restart_can(void)
{
	can_stop(can_dev);
	if (can_start(can_dev)) {
		LOG_ERR("CAN restart failed");
		can_ready = false;
	}
}

static int send_frame(const uint8_t data[8])
{
	struct can_frame frame = {.id = DOOR_CMD_ID, .dlc = 8};
	int err;

	memcpy(frame.data, data, 8);
	k_sem_reset(&tx_sem);
	err = can_send(can_dev, &frame, K_MSEC(DOOR_TX_TIMEOUT_MS), tx_cb, NULL);
	if (err) {
		return err;
	}
	if (k_sem_take(&tx_sem, K_MSEC(DOOR_TX_TIMEOUT_MS))) {
		/*
		 * Not ACKed. Left alone, the MCP2515 would keep retransmitting it and fire it the
		 * instant the ECU wakes - a stale lock/unlock. Aborting it here prevents that.
		 */
		restart_can();
		return -ETIMEDOUT;
	}
	return tx_err;
}

#if RELAY_WAKE_ENABLED
/* Waits up to ms, returning as soon as any frame arrives. */
static bool wait_for_traffic(uint32_t ms)
{
	return k_sem_take(&rx_sem, K_MSEC(ms)) == 0;
}

/*
 * Wakes a body ECU in deep sleep by pulsing the relay across the driver's jamb switch as a train of
 * open/close cycles - the way Toyota's service procedure wakes a sleeping main body ECU (open the
 * door, then open and close several times at ~1.5 s intervals). A single pulse isn't enough.
 * Returns as soon as the bus comes alive, so a wake short-circuits the rest of the train.
 */
static bool relay_wake(void)
{
	LOG_INF("wake: pulsing relay on the jamb-switch line...");
	k_sem_reset(&rx_sem);
	for (int i = 0; i < RELAY_PULSE_COUNT; i++) {
		gpio_pin_set_dt(&relay, 1); /* "door open" - ground the line */
		bool awake = wait_for_traffic(RELAY_PULSE_ON_MS);

		gpio_pin_set_dt(&relay, 0); /* "door closed" - release to the pull-up */
		if (awake || wait_for_traffic(RELAY_PULSE_OFF_MS)) {
			LOG_INF("wake: bus came alive after %d pulse(s)", i + 1);
			return true;
		}
	}
	/* Train done: give the gateway/ECU a last moment to bring the bus up before giving up. */
	if (wait_for_traffic(RELAY_WAKE_TIMEOUT_MS)) {
		LOG_INF("wake: bus came alive after the pulse train");
		return true;
	}
	LOG_WRN("wake: no traffic after relay pulses; ECU may still be asleep");
	return false;
}
#endif

/*
 * Retrying matters even when the bus reads silent: measured on the CT200h, a door frame sent after
 * 42 s of silence was both ACKed and positively answered. A quiet bus is not necessarily a sleeping
 * one - as long as this node stays in normal mode and ACKs the gateway's NM frames, those quiet
 * stretches are gaps in the NM cycle, not deep sleep. Lock and unlock are idempotent, so a repeat
 * is harmless.
 */
static bool send_with_retries(const uint8_t *data)
{
	for (int attempt = 1; attempt <= DOOR_MAX_RETRIES && can_ready; attempt++) {
		int err;

		k_sem_reset(&reply_sem);
		err = send_frame(data);
		if (!err) {
			LOG_INF("sent OK (attempt %d)", attempt);
			if (k_sem_take(&reply_sem, K_MSEC(DOOR_REPLY_WAIT_MS)) == 0) {
				LOG_INF("body ECU confirmed (758: %02X %02X %02X %02X)", reply[0],
					reply[1], reply[2], reply[3]);
			}
			return true;
		}
		LOG_WRN("door frame failed (err %d)", err);
		if (attempt < DOOR_MAX_RETRIES) {
			k_sleep(K_MSEC(DOOR_RETRY_GAP_MS));
		}
	}

	if (bus_awake()) {
		/* traffic is flowing but our frame still wasn't ACKed: not a sleep problem */
		LOG_WRN("NOT delivered although the bus is active - check bitrate/wiring");
	} else {
		LOG_WRN("NOT delivered after retries: bus unresponsive");
	}
	return false;
}

bool door_send(enum door_cmd cmd)
{
	const uint8_t *data = cmd == DOOR_UNLOCK ? UNLOCK_CMD_DATA : LOCK_CMD_DATA;
	bool ok = false;

	k_mutex_lock(&door_mutex, K_FOREVER);
	if (!can_ready) {
		can_ready = start_can() == 0;
	}
	if (!can_ready) {
		LOG_WRN("CAN not ready, command dropped");
		goto out;
	}

	/*
	 * Don't launch a door frame into a bus in deep sleep: nothing would ACK it. The NM frame
	 * ticks ~1 Hz in the responsive window, so bus_awake() stays true there and the quiet-stretch
	 * sends above still happen.
	 */
	if (!bus_awake()) {
#if RELAY_WAKE_ENABLED
		/*
		 * Only unlock-on-return has to defeat deep sleep; lock happens in the responsive
		 * window just after walking away, where the bus is still awake.
		 */
		if (cmd != DOOR_UNLOCK || !relay_wake()) {
			LOG_WRN("bus in deep sleep, command dropped");
			goto out;
		}
		LOG_INF("bus woke after relay pulse, sending command");
#else
		LOG_WRN("bus in deep sleep, command dropped");
		goto out;
#endif
	}

	ok = send_with_retries(data);
out:
	k_mutex_unlock(&door_mutex);
	return ok;
}

int door_init(void)
{
#if RELAY_WAKE_ENABLED
	/* Start with the relay released ("door closed"). */
	int err = gpio_pin_configure_dt(&relay, GPIO_OUTPUT_INACTIVE);

	if (err) {
		LOG_ERR("relay GPIO failed (err %d)", err);
		return err;
	}
#endif
	can_ready = start_can() == 0;
	return can_ready ? 0 : -ENODEV;
}
