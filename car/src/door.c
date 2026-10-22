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
#include <zephyr/drivers/can/transceiver.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

#include "door.h"
#include "door_config.h"

LOG_MODULE_REGISTER(door, LOG_LEVEL_INF);

static const struct device *const can_dev = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));
/* Raw access to the same chip, for the Sleep mode the Zephyr driver doesn't offer. */
static const struct spi_dt_spec mcp_spi =
	SPI_DT_SPEC_GET(DT_CHOSEN(zephyr_canbus), SPI_OP_MODE_MASTER | SPI_WORD_SET(8));
#if BUS_WAKE_ENABLED
static const struct gpio_dt_spec can_int = GPIO_DT_SPEC_GET(DT_CHOSEN(zephyr_canbus), int_gpios);
#if !MCP_SLEEP_ENABLED
static const struct device *const can_phy = DEVICE_DT_GET(DT_PHANDLE(DT_CHOSEN(zephyr_canbus), phys));
#endif
#endif
#if RELAY_WAKE_ENABLED
static const struct gpio_dt_spec relay = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), relay_gpios);
#endif

static K_MUTEX_DEFINE(door_mutex);
static K_SEM_DEFINE(tx_sem, 0, 1);
static K_SEM_DEFINE(reply_sem, 0, 1);
static K_SEM_DEFINE(rx_sem, 0, 1);
static K_SEM_DEFINE(query_sem, 0, 1);

static bool can_ready;
static bool mcp_asleep;
static int filter_id = -1;
static int tx_err;
static uint8_t reply[4];
/* door_query(): replies are captured from [query_id, query_id + query_span), span 0 when none is due. */
static uint16_t query_id;
static atomic_t query_span;
/* Uptime before which door_query() refuses to send, 0 when no hold-off is running. */
static uint32_t query_holdoff_until;
static uint8_t query_rsp[8];

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

	if (frame->id - query_id < (uint32_t)atomic_get(&query_span)) {
		memcpy(query_rsp, frame->data, MIN(can_dlc_to_bytes(frame->dlc), sizeof(query_rsp)));
		atomic_set(&query_span, 0);
		k_sem_give(&query_sem);
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

static int mcp_bit_modify(uint8_t addr, uint8_t mask, uint8_t value)
{
	uint8_t cmd[] = {MCP_OPCODE_BIT_MODIFY, addr, mask, value};
	const struct spi_buf buf = {.buf = cmd, .len = sizeof(cmd)};
	const struct spi_buf_set tx = {.buffers = &buf, .count = 1};

	return spi_write_dt(&mcp_spi, &tx);
}

static int mcp_read(uint8_t addr, uint8_t *value)
{
	uint8_t cmd[3] = {MCP_OPCODE_READ, addr, 0};
	uint8_t rsp[3];
	const struct spi_buf tx_buf = {.buf = cmd, .len = sizeof(cmd)};
	const struct spi_buf rx_buf = {.buf = rsp, .len = sizeof(rsp)};
	const struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};
	const struct spi_buf_set rx = {.buffers = &rx_buf, .count = 1};
	int err = spi_transceive_dt(&mcp_spi, &tx, &rx);

	*value = rsp[2];
	return err;
}

/*
 * Puts the MCP2515 in Sleep mode, a few uA. can_stop() first aborts any pending frame and tells the
 * driver it is stopped. WAKIE stays off outside mcp_wake() and door_arm_bus_wake(), so bus activity
 * doesn't wake the chip.
 * The chip only enters Sleep once the bus is idle, which can take longer than the check below, so it
 * counts as asleep either way: mcp_wake() on a chip that is already awake does no harm, while a
 * mode request to a sleeping chip times out. Call with door_mutex held.
 */
static void mcp_sleep(void)
{
	uint8_t canstat = 0;

	can_stop(can_dev);
	can_ready = false;
	if (!MCP_SLEEP_ENABLED) {
		/* Configuration mode: off the bus, and the next start needs no wake. */
		return;
	}
	mcp_bit_modify(MCP_ADDR_CANCTRL, MCP_MODE_MASK, MCP_MODE_SLEEP);
	for (int i = 0; i < 10; i++) {
		if (!mcp_read(MCP_ADDR_CANSTAT, &canstat) &&
		    (canstat & MCP_MODE_MASK) == MCP_MODE_SLEEP) {
			mcp_asleep = true;
			LOG_INF("MCP2515 asleep");
			return;
		}
		k_sleep(K_MSEC(1));
	}
	mcp_asleep = true;
	LOG_WRN("MCP2515 Sleep not confirmed yet (CANSTAT 0x%02X)", canstat);
}

/*
 * Setting WAKIF by hand wakes the chip as bus activity would, but only with WAKIE set: without it the
 * chip stays asleep and every mode request times out. WAKIE is on just for the wake, and the
 * driver's interrupt handler clears the WAKIF interrupt it doesn't use. The chip restarts its
 * oscillator and comes up in Listen-Only mode; can_start() then returns it to normal mode. Since nothing was received while
 * it slept, it listens for a moment so bus_awake() reflects the bus as it is now. Call with
 * door_mutex held.
 */
/* The registers that say which mode the chip is in and why a mode change may stall. */
static void mcp_dump(const char *when)
{
	uint8_t canstat = 0, canctrl = 0, inte = 0, intf = 0, eflg = 0, tec = 0;
	uint8_t cnf1 = 0, cnf2 = 0, cnf3 = 0;

	mcp_read(MCP_ADDR_CNF1, &cnf1);
	mcp_read(MCP_ADDR_CNF2, &cnf2);
	mcp_read(MCP_ADDR_CNF3, &cnf3);
	mcp_read(MCP_ADDR_CANSTAT, &canstat);
	mcp_read(MCP_ADDR_CANCTRL, &canctrl);
	mcp_read(MCP_ADDR_CANINTE, &inte);
	mcp_read(MCP_ADDR_CANINTF, &intf);
	mcp_read(MCP_ADDR_EFLG, &eflg);
	mcp_read(MCP_ADDR_TEC, &tec);
	LOG_INF("MCP2515 %s: CANSTAT %02X CANCTRL %02X CANINTE %02X CANINTF %02X EFLG %02X TEC %u "
		"CNF %02X %02X %02X", when, canstat, canctrl, inte, intf, eflg, tec, cnf1, cnf2, cnf3);
}

/*
 * The driver writes CNF1-3 and CANINTE once, at init. A brown-out or reset of the MCP2515 alone
 * zeroes them while the driver still believes the chip is configured: no interrupt ever fires, so
 * nothing is received and every send times out, and the bitrate is wrong. CANINTE is never 0 once
 * configured, so a 0 there means the chip lost its setup; setting the bitrate again rewrites it all.
 * Call with door_mutex held, after the chip has been started.
 */
static void mcp_restore_config(void)
{
	uint8_t inte = 0;

	if (mcp_read(MCP_ADDR_CANINTE, &inte) || inte != 0) {
		return;
	}
	mcp_dump("lost its configuration");
	/*
	 * Not can_set_bitrate(): its default 87.5 % sample point is out of reach with the 8 MHz
	 * crystal (8 TQ per bit at 500 kbit/s, PS2 >= 2), so it fails with -ERANGE. 75 % fits exactly.
	 */
	struct can_timing timing;
	int stop_err = can_stop(can_dev);
	int rate_err = can_calc_timing(can_dev, &timing, DT_PROP(DT_CHOSEN(zephyr_canbus), bitrate),
				       CAN_SAMPLE_POINT_PERMILLE);

	rate_err = rate_err < 0 ? rate_err : can_set_timing(can_dev, &timing);
	if (!rate_err) {
		/*
		 * Zephyr always clears SAM; the ESP32 tool, which transmits fine on this bus with the
		 * same module, samples each bit three times (mcp_can: CNF2 0xD1). Match it.
		 */
		mcp_bit_modify(MCP_ADDR_CNF2, MCP_CNF2_SAM, MCP_CNF2_SAM);
	}
	int start_err = rate_err ? 0 : can_start(can_dev);

	if (rate_err || start_err) {
		LOG_ERR("MCP2515 reconfiguration failed (stop %d, bitrate %d, start %d)", stop_err,
			rate_err, start_err);
		mcp_dump("after failed reconfiguration");
		can_ready = false;
		return;
	}
	LOG_WRN("MCP2515 reconfigured after a reset, listening for bus traffic...");
	k_sem_reset(&rx_sem);
	k_sem_take(&rx_sem, K_MSEC(CAN_WAKE_LISTEN_MS));
}

static void mcp_wake(void)
{
	mcp_bit_modify(MCP_ADDR_CANINTE, MCP_CANINTE_WAKIE, MCP_CANINTE_WAKIE);
	mcp_bit_modify(MCP_ADDR_CANINTF, MCP_CANINTF_WAKIF, MCP_CANINTF_WAKIF);
	k_sleep(K_MSEC(MCP_OSC_START_MS));
	mcp_bit_modify(MCP_ADDR_CANINTF, MCP_CANINTF_WAKIF, 0);
	mcp_bit_modify(MCP_ADDR_CANINTE, MCP_CANINTE_WAKIE, 0);
	mcp_asleep = false;

	if (can_start(can_dev)) {
		LOG_ERR("MCP2515 wake failed");
		mcp_dump("after failed start");
		return;
	}
	can_ready = true;
	LOG_INF("MCP2515 awake, listening for bus traffic...");
	k_sem_reset(&rx_sem);
	k_sem_take(&rx_sem, K_MSEC(CAN_WAKE_LISTEN_MS));
}

static int send_frame(uint16_t id, const uint8_t data[8])
{
	struct can_frame frame = {.id = id, .dlc = 8};
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
		 * TXBnCTRL first: TXREQ 0x08 still pending, TXERR 0x10 bus error (no ACK), MLOA 0x20
		 * arbitration lost. Pending with neither means it never got a slot on the bus.
		 */
		uint8_t txb[3] = {0};

		for (int i = 0; i < 3; i++) {
			mcp_read(MCP_ADDR_TXB0CTRL + 0x10 * i, &txb[i]);
		}
		LOG_WRN("TX timeout: TXB0CTRL %02X TXB1CTRL %02X TXB2CTRL %02X", txb[0], txb[1],
			txb[2]);
		mcp_dump("at TX timeout");
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
	mcp_dump("after failed wake");
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
		err = send_frame(DOOR_CMD_ID, data);
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
	mcp_dump("after failed send");
	return false;
}

bool door_send(enum door_cmd cmd)
{
	const uint8_t *data = cmd == DOOR_UNLOCK ? UNLOCK_CMD_DATA : LOCK_CMD_DATA;
	bool ok = false;

	k_mutex_lock(&door_mutex, K_FOREVER);
	if (mcp_asleep) {
		mcp_wake();
	}
	if (!can_ready) {
		can_ready = start_can() == 0;
	}
	if (!can_ready) {
		LOG_WRN("CAN not ready, command dropped");
		goto out;
	}
	mcp_restore_config();
	if (!can_ready) {
		goto out;
	}

	/*
	 * A silent bus is not necessarily a sleeping one (see send_with_retries()), so try the frame
	 * first and only wake the ECU if it isn't delivered. Unanswered frames can't drive the
	 * controller bus-off: ACK errors stop raising TEC once it is error-passive.
	 */
	if (!bus_awake()) {
		LOG_INF("bus quiet, trying the frame anyway");
		ok = send_with_retries(data);
		if (ok) {
			goto out;
		}
	}

	if (!bus_awake()) {
#if RELAY_WAKE_ENABLED
		if (!relay_wake()) {
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
	if (!can_ready) {
		return -ENODEV;
	}
	/* No fob yet: the controller sleeps until the first command. */
	k_mutex_lock(&door_mutex, K_FOREVER);
	mcp_sleep();
	k_mutex_unlock(&door_mutex);
	return 0;
}

void door_sleep(void)
{
	k_mutex_lock(&door_mutex, K_FOREVER);
	if (!mcp_asleep) {
		mcp_sleep();
	}
	k_mutex_unlock(&door_mutex);
}

int door_query(uint16_t req_id, const uint8_t req[8], uint16_t rsp_id, uint8_t rsp_span,
	       uint8_t rsp[8], k_timeout_t timeout)
{
	int err;

	k_mutex_lock(&door_mutex, K_FOREVER);
	if (mcp_asleep) {
		mcp_wake();
	}
	if (!can_ready) {
		can_ready = start_can() == 0;
	}
	if (!can_ready) {
		err = -ENODEV;
		goto out;
	}
	mcp_restore_config();
	if (!can_ready) {
		err = -ENODEV;
		goto out;
	}
	/* A query never wakes the bus: only a door command is worth the relay. */
	if (!bus_awake()) {
		err = -ENETDOWN;
		goto out;
	}
	/*
	 * Queries are polled about once a second. While frames can't get out, every attempt drives
	 * the controller bus-off and puts error frames on the car's bus, so hold off after a failure.
	 */
	if (query_holdoff_until && (int32_t)(k_uptime_get_32() - query_holdoff_until) < 0) {
		err = -ENETDOWN;
		goto out;
	}

	query_id = rsp_id;
	memset(query_rsp, 0, sizeof(query_rsp));
	k_sem_reset(&query_sem);
	atomic_set(&query_span, rsp_span);
	err = send_frame(req_id, req);
	if (err == -ETIMEDOUT) {
		query_holdoff_until = k_uptime_get_32() + QUERY_HOLDOFF_MS;
		LOG_WRN("query frame not delivered, pausing queries for %d s", QUERY_HOLDOFF_MS / 1000);
	}
	if (!err && k_sem_take(&query_sem, timeout)) {
		err = -ETIMEDOUT;
	}
	atomic_set(&query_span, 0);
	if (!err) {
		memcpy(rsp, query_rsp, sizeof(query_rsp));
	}
out:
	k_mutex_unlock(&door_mutex);
	return err;
}

#if BUS_WAKE_ENABLED
int door_arm_bus_wake(void)
{
	int err;

	k_mutex_lock(&door_mutex, K_FOREVER);
#if MCP_SLEEP_ENABLED
	if (!mcp_asleep) {
		mcp_sleep();
	}
	/*
	 * Bus activity now wakes the chip, which sets WAKIF and pulls INT low. The driver resets the
	 * MCP2515 at the next boot, which clears WAKIE and WAKIF again.
	 */
	err = mcp_bit_modify(MCP_ADDR_CANINTE, MCP_CANINTE_WAKIE, MCP_CANINTE_WAKIE);
#else
	/*
	 * Sleep is off on this module (MCP_SLEEP_ENABLED), so the chip listens instead: Listen-Only
	 * never ACKs or transmits, and the first standard frame raises the receive interrupt the driver
	 * enabled at init, which pulls INT low. The transceiver then goes back to standby, where its
	 * receiver stays on. A few mA, about what Configuration mode already draws between sessions.
	 */
	can_stop(can_dev);
	can_ready = false;
	err = can_set_mode(can_dev, CAN_MODE_LISTENONLY);
	if (!err) {
		err = can_start(can_dev);
	}
	if (!err) {
		err = can_transceiver_disable(can_phy);
	}
#endif
	if (!err) {
		/* A level interrupt sets the pin's SENSE, which is what wakes the chip from System OFF. */
		err = gpio_pin_interrupt_configure_dt(&can_int, GPIO_INT_LEVEL_ACTIVE);
	}
	k_mutex_unlock(&door_mutex);
	return err;
}
#else
int door_arm_bus_wake(void)
{
	return -ENOTSUP;
}
#endif
