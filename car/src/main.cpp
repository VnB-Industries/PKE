/*
 * PKE car unit on an ESP32 DevKit, ported from car/src/main.c on nrf52840/main.
 *
 * Simplified authentication: the car never connects to the fob. It unlocks
 * once an advert whose RPA resolves with the enrolled fob IRK holds a strong
 * enough filtered RSSI, and locks once the fob's adverts fade or stop. See
 * README "Safety" for what that gives up.
 *
 * The loop task runs the whole sequence; the NimBLE scan callback only queues adverts.
 */
#include <string.h>

#include <Arduino.h>
#include <NimBLEDevice.h>

#include "actions.h"
#include "door.h"
#include "log.h"
#include "rpa.h"
#include "tap_listener.h"
#include "vehicle.h"

LOG_MODULE("car");

/* Step 2: unlock once the filtered advert RSSI holds this level. */
#define DETECT_RSSI_DBM -82
#define DETECT_SAMPLES  1

/*
 * Step 3: the exit starts once the filtered advert RSSI stays below this level.
 * Only 4 dB below DETECT, so the fob locks after a short walk away. A narrow
 * dead band risks flapping at the boundary; EXIT_SAMPLES + the parked check guard it.
 */
#define EXIT_RSSI_DBM -86
#define EXIT_SAMPLES  5
/* The fob advertises every ~250 ms: this long without an advert means it is out of range. */
#define FOB_LOST_MS 3000

/*
 * Pause before searching again after a session ends, including after a lock:
 * the gap between DETECT_RSSI_DBM and EXIT_RSSI_DBM already keeps a fob at
 * the boundary from flapping between lock and unlock.
 */
#define RETRY_DELAY_MS 500

/* Same as Zephyr's BT_GAP_SCAN_FAST_INTERVAL / BT_GAP_SCAN_FAST_WINDOW. */
#define SCAN_INTERVAL_MS 60
#define SCAN_WINDOW_MS   30

#define ADV_QUEUE_LEN 16

struct scan_report {
	uint8_t addr[6]; /* little-endian */
	int8_t rssi;
};

static QueueHandle_t adv_q;

/* Exponential moving average, alpha = 1/2: reacts fast so a fob walking in is detected sooner. */
static int rssi_filter(int ema, int sample, bool first)
{
	return first ? sample : ema + (sample - ema) / 2;
}

class ScanCallbacks : public NimBLEScanCallbacks {
	void onResult(const NimBLEAdvertisedDevice *dev) override
	{
		const NimBLEAddress &addr = dev->getAddress();
		struct scan_report report;

		/* Cheap pre-filter here; the AES check runs in the loop task. */
		if (dev->getAdvType() != BLE_HCI_ADV_RPT_EVTYPE_ADV_IND || !addr.isRpa()) {
			return;
		}
		memcpy(report.addr, addr.getVal(), sizeof(report.addr));
		report.rssi = dev->getRSSI();
		xQueueSend(adv_q, &report, 0);
	}
};

static ScanCallbacks scan_callbacks;

static void halt(void)
{
	vTaskSuspend(NULL);
}

/*
 * Waits up to timeout for an advert from the fob, skipping every other RPA.
 * True with that advert in report, false on timeout.
 */
static bool next_fob_advert(TickType_t timeout, struct scan_report *report)
{
	TickType_t start = xTaskGetTickCount();

	for (;;) {
		TickType_t elapsed = xTaskGetTickCount() - start;

		if (timeout != portMAX_DELAY && elapsed >= timeout) {
			return false;
		}
		if (xQueueReceive(adv_q, report,
				  timeout == portMAX_DELAY ? portMAX_DELAY : timeout - elapsed) != pdTRUE) {
			return false;
		}
		if (rpa_is_fob(report->addr)) {
			return true;
		}
	}
}

/* Step 2: waits until the fob's RPA resolves with a strong enough filtered RSSI. Returns that RSSI. */
static int wait_for_fob(void)
{
	struct scan_report report;
	int ema = 0;
	int above = 0;
	bool first = true;
	unsigned int samples = 0;

	xQueueReset(adv_q);
	LOG_INF("scanning for fob");

	for (;;) {
		next_fob_advert(portMAX_DELAY, &report);
		ema = rssi_filter(ema, report.rssi, first);
		first = false;
		above = ema >= DETECT_RSSI_DBM ? above + 1 : 0;

		/* One advert per address arrives often; log every 4th to keep the line rate sane. */
		if (++samples % 4 == 0) {
			LOG_INF("fob resolved %02X:%02X:%02X:%02X:%02X:%02X rssi=%d ema=%d", report.addr[5],
				report.addr[4], report.addr[3], report.addr[2], report.addr[1], report.addr[0],
				report.rssi, ema);
		}
		if (above >= DETECT_SAMPLES) {
			LOG_INF("fob detected");
			return ema;
		}
	}
}

/*
 * Step 3: follows the fob's advert RSSI until the driver walks away with the car parked.
 * Returns true on an exit, false when the fob stopped being heard first.
 */
static bool wait_for_exit(void)
{
	struct scan_report report;
	unsigned int samples = 0;
	int ema = 0;
	int below = 0;

	LOG_INF("exit detection armed (advert RSSI < %d dBm for %d samples, or none for %d ms)",
		EXIT_RSSI_DBM, EXIT_SAMPLES, FOB_LOST_MS);
	for (;;) {
		if (!next_fob_advert(pdMS_TO_TICKS(FOB_LOST_MS), &report)) {
			LOG_WRN("fob lost while unlocked, treating as exit");
			return false;
		}
		ema = rssi_filter(ema, report.rssi, samples == 0);
		below = ema < EXIT_RSSI_DBM ? below + 1 : 0;
		if (++samples % 4 == 0) {
			LOG_INF("fob rssi=%d ema=%d", report.rssi, ema);
		}
		if (below < EXIT_SAMPLES) {
			continue;
		}

		LOG_INF("exit started: advert ema=%d dBm below %d dBm", ema, EXIT_RSSI_DBM);
		if (vehicle_is_parked()) {
			return true;
		}
		below = 0;
	}
}

/* Steps 2-4 for one approach of the fob. */
static void run_session(int rssi)
{
	bool locked = false;
	bool seen;

	(void)rssi;

	/*
	 * Proximity only arms the session: the mic listens while the fob is detected and the
	 * driver must double tap to unlock (tap_listener.cpp handle_sequence -> door_send).
	 * No automatic unlock on approach.
	 */
	tap_listener_start();

	/* A fob no longer heard is out of BLE range, so beyond any lock radius. */
	do {
		seen = wait_for_exit();
		locked = lock_decide();
	} while (!locked && seen);

	tap_listener_stop();
	door_sleep();
}

void setup()
{
	NimBLEScan *scan;

	Serial.begin(115200);
	delay(300);

	adv_q = xQueueCreate(ADV_QUEUE_LEN, sizeof(struct scan_report));
	if (rpa_init()) {
		LOG_ERR("crypto init failed");
		halt();
	}
	/* Neither is fatal: BLE still runs, and door_send() retries the CAN start. */
	door_init();
	tap_listener_init();

	if (!NimBLEDevice::init("PKE car")) {
		LOG_ERR("bluetooth init failed");
		halt();
	}
	/*
	 * Passive and continuous, with no duplicate filtering and no stored results:
	 * every advert feeds the RSSI filter.
	 */
	scan = NimBLEDevice::getScan();
	scan->setScanCallbacks(&scan_callbacks, true);
	scan->setActiveScan(false);
	scan->setInterval(SCAN_INTERVAL_MS);
	scan->setWindow(SCAN_WINDOW_MS);
	scan->setMaxResults(0);
	if (!scan->start(0, false, true)) {
		LOG_ERR("scan start failed");
		halt();
	}
	LOG_INF("car ready");
}

void loop()
{
	run_session(wait_for_fob());
	delay(RETRY_DELAY_MS);
}
