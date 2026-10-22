/*
 * Door frames over CAN through the MCP2515, ported from car/src/door.c on
 * nrf52840/main. The measurements behind each choice are in the README,
 * "Waking a sleeping bus".
 *
 * The controller stays in normal mode and receives every frame in its own
 * task, woken by the INT pin: ACKing the gateway's NM frames keeps the
 * responsive window usable, and the last frame seen tells whether the bus is
 * awake without having to transmit and fail.
 */
#include <errno.h>
#include <string.h>

#include <Arduino.h>
#include <mcp_can.h>

#include "board.h"
#include "door.h"
#include "door_config.h"
#include "log.h"

LOG_MODULE("door");

/* Above the tap processing task and the loop task, which both send door frames. */
#define RX_TASK_PRIORITY 3
#define RX_TASK_STACK    3072

static MCP_CAN can0(MCP_CS_PIN);

/* door_mutex serialises whole commands; mcp_mutex every SPI exchange with the chip. */
static SemaphoreHandle_t door_mutex;
static SemaphoreHandle_t mcp_mutex;
static SemaphoreHandle_t irq_sem;
static SemaphoreHandle_t rx_sem;
static SemaphoreHandle_t reply_sem;

/* Written with mcp_mutex held, so the receive task never talks to a sleeping or absent chip. */
static bool can_ready;
static bool mcp_asleep;
static uint8_t reply[4];

/* millis() of the last frame received from any other node; rx_seen stays false until the first one. */
static volatile uint32_t last_rx_ms;
static volatile bool rx_seen;

static void IRAM_ATTR mcp_int_isr(void)
{
	BaseType_t woken = pdFALSE;

	xSemaphoreGiveFromISR(irq_sem, &woken);
	if (woken) {
		portYIELD_FROM_ISR();
	}
}

/* Empties both receive buffers. Call with mcp_mutex held. */
static void drain_rx(void)
{
	unsigned long id; /* INT32U in mcp_can */
	uint8_t len;
	uint8_t buf[8];

	while (can0.checkReceive() == CAN_MSGAVAIL) {
		if (can0.readMsgBuf(&id, &len, buf) != CAN_OK) {
			break;
		}
		last_rx_ms = millis();
		rx_seen = true;
		xSemaphoreGive(rx_sem);

		/* Positive response to service 0x30 (0x30 + 0x40), echoing LID 0x11. */
		if (id == DOOR_RESPONSE_ID && len >= 4 && buf[2] == 0x70) {
			memcpy(reply, buf, sizeof(reply));
			xSemaphoreGive(reply_sem);
		}
	}
}

static void rx_task(void *arg)
{
	for (;;) {
		xSemaphoreTake(irq_sem, portMAX_DELAY);
		/*
		 * A frame that lands while the buffers are being drained keeps INT low without a new
		 * falling edge, so drain until the line is released.
		 */
		do {
			xSemaphoreTake(mcp_mutex, portMAX_DELAY);
			if (can_ready) {
				drain_rx();
			}
			xSemaphoreGive(mcp_mutex);
		} while (can_ready && digitalRead(MCP_INT_PIN) == LOW);
	}
}

static bool bus_awake(void)
{
	return rx_seen && (millis() - last_rx_ms) < BUS_AWAKE_TIMEOUT_MS;
}

/* Resets the chip and puts it on-bus in normal mode, accepting every frame. Call with mcp_mutex held. */
static bool reset_can(void)
{
	can_ready = can0.begin(MCP_ANY, MCP_CAN_SPEED, MCP_OSC_FREQ) == CAN_OK &&
		    can0.setMode(MCP_NORMAL) == MCP2515_OK;
	return can_ready;
}

static bool start_can(void)
{
	bool ok;

	xSemaphoreTake(mcp_mutex, portMAX_DELAY);
	ok = reset_can();
	xSemaphoreGive(mcp_mutex);
	if (!ok) {
		LOG_ERR("MCP2515 init failed");
		return false;
	}
	LOG_INF("MCP2515 on-bus");
	return true;
}

/*
 * Puts the MCP2515 in Sleep mode, a few uA. WAKIE stays off (mcp_can leaves it off unless asked),
 * so bus activity doesn't wake the chip: only mcp_wake() does. Call with door_mutex held.
 */
static void mcp_sleep(void)
{
	uint8_t err;

	xSemaphoreTake(mcp_mutex, portMAX_DELAY);
	can_ready = false;
	err = can0.setMode(MCP_SLEEP);
	xSemaphoreGive(mcp_mutex);
	if (err != MCP2515_OK) {
		LOG_WRN("MCP2515 did not enter Sleep");
		return;
	}
	mcp_asleep = true;
	LOG_INF("MCP2515 asleep");
}

/*
 * Leaving Sleep is done by mcp_can's setMode(): it sets WAKIF by hand, as bus activity would, waits
 * for the chip to restart its oscillator in Listen-Only mode, then switches it to normal mode. Since
 * nothing was received while it slept, it listens for a moment so bus_awake() reflects the bus as it
 * is now. Call with door_mutex held.
 */
static void mcp_wake(void)
{
	uint8_t err;

	xSemaphoreTake(mcp_mutex, portMAX_DELAY);
	err = can0.setMode(MCP_NORMAL);
	can_ready = err == MCP2515_OK;
	xSemaphoreGive(mcp_mutex);
	mcp_asleep = false;

	if (err != MCP2515_OK) {
		LOG_ERR("MCP2515 wake failed");
		return;
	}
	LOG_INF("MCP2515 awake, listening for bus traffic...");
	xSemaphoreTake(rx_sem, 0);
	xSemaphoreTake(rx_sem, pdMS_TO_TICKS(CAN_WAKE_LISTEN_MS));
}

/* Returns CAN_OK, or the mcp_can error code (7 = CAN_SENDMSGTIMEOUT: not ACKed). */
static uint8_t send_frame(const uint8_t data[8])
{
	uint8_t buf[8];
	uint8_t eflg, tec, rec;
	uint8_t res;

	memcpy(buf, data, sizeof(buf));
	xSemaphoreTake(mcp_mutex, portMAX_DELAY);
	/* mcp_can waits up to TIMEOUTVALUE (2.5 ms) for the frame to leave the TX buffer. */
	res = can0.sendMsgBuf(DOOR_CMD_ID, 0, sizeof(buf), buf);
	if (res == CAN_OK) {
		xSemaphoreGive(mcp_mutex);
		return CAN_OK;
	}

	/* EFLG: bit0 EWARN, bit2 TXWAR, bit4 TXEP (error-passive), bit5 TXBO (bus-off) */
	eflg = can0.getError();
	tec = can0.errorCountTX();
	rec = can0.errorCountRX();
	/*
	 * Not ACKed. Left alone, the MCP2515 would keep retransmitting it and fire it the instant the
	 * ECU wakes - a stale lock/unlock. Resetting the chip drops it and clears the error counters.
	 * (mcp_can's abortTX() leaves CANCTRL.ABAT set, which would abort every later send too.)
	 */
	if (!reset_can()) {
		LOG_ERR("CAN restart failed");
	}
	xSemaphoreGive(mcp_mutex);
	LOG_WRN("door frame failed: code=%u EFLG=0x%02X TEC=%u REC=%u", res, eflg, tec, rec);
	return res;
}

static void relay_set(bool energised)
{
	digitalWrite(RELAY_PIN, energised == RELAY_ACTIVE_HIGH ? HIGH : LOW);
}

#if RELAY_WAKE_ENABLED
/* Waits up to ms, returning as soon as any frame arrives. */
static bool wait_for_traffic(uint32_t ms)
{
	return xSemaphoreTake(rx_sem, pdMS_TO_TICKS(ms)) == pdTRUE;
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
	xSemaphoreTake(rx_sem, 0);
	for (int i = 0; i < RELAY_PULSE_COUNT; i++) {
		relay_set(true); /* "door open" - ground the line */
		bool awake = wait_for_traffic(RELAY_PULSE_ON_MS);

		relay_set(false); /* "door closed" - release to the pull-up */
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
		xSemaphoreTake(reply_sem, 0);
		if (send_frame(data) == CAN_OK) {
			LOG_INF("sent OK (attempt %d)", attempt);
			if (xSemaphoreTake(reply_sem, pdMS_TO_TICKS(DOOR_REPLY_WAIT_MS)) == pdTRUE) {
				LOG_INF("body ECU confirmed (758: %02X %02X %02X %02X)", reply[0], reply[1],
					reply[2], reply[3]);
			}
			return true;
		}
		if (attempt < DOOR_MAX_RETRIES) {
			vTaskDelay(pdMS_TO_TICKS(DOOR_RETRY_GAP_MS));
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

	xSemaphoreTake(door_mutex, portMAX_DELAY);
	if (mcp_asleep) {
		mcp_wake();
	}
	if (!can_ready) {
		start_can();
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
	xSemaphoreGive(door_mutex);
	return ok;
}

int door_init(void)
{
	door_mutex = xSemaphoreCreateMutex();
	mcp_mutex = xSemaphoreCreateMutex();
	irq_sem = xSemaphoreCreateBinary();
	rx_sem = xSemaphoreCreateBinary();
	reply_sem = xSemaphoreCreateBinary();

	/* Start with the relay released ("door closed"). */
	pinMode(RELAY_PIN, OUTPUT);
	relay_set(false);

	pinMode(MCP_INT_PIN, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(MCP_INT_PIN), mcp_int_isr, FALLING);
	xTaskCreatePinnedToCore(rx_task, "can_rx", RX_TASK_STACK, NULL, RX_TASK_PRIORITY, NULL, 1);

	if (!start_can()) {
		return -ENODEV;
	}
	/* No fob yet: the controller sleeps until the first command. */
	xSemaphoreTake(door_mutex, portMAX_DELAY);
	mcp_sleep();
	xSemaphoreGive(door_mutex);
	return 0;
}

void door_sleep(void)
{
	xSemaphoreTake(door_mutex, portMAX_DELAY);
	if (!mcp_asleep) {
		mcp_sleep();
	}
	xSemaphoreGive(door_mutex);
}
