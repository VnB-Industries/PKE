/*
 * PKE car unit: scans for the enrolled fob, connects once it is close,
 * authenticates it, then watches the live link for the driver walking away.
 *
 * The main thread runs the whole sequence and blocks on car_events; the
 * Bluetooth callbacks only post events or queue adverts.
 */
#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>

#include "actions.h"
#include "auth_client.h"
#include "boot_led.h"
#include "car.h"
#include "door.h"
#include "pke_crypto.h"
#include "power.h"
#include "ranging.h"
#include "rpa.h"
#include "tap_listener.h"
#include "vehicle.h"

LOG_MODULE_REGISTER(car, LOG_LEVEL_INF);

/* Step 2: connect once the filtered advert RSSI holds this level. */
#define CONNECT_RSSI_DBM -70
#define CONNECT_SAMPLES  3

/* Step 6: the exit starts once the filtered link RSSI stays below this level. */
#define EXIT_RSSI_DBM -80
#define EXIT_SAMPLES  5
#define EXIT_POLL_MS  250

/*
 * Step 5: unlock as soon as the fob is authenticated. Off: the doors open with a double tap or the
 * original key, and the session only watches for the exit and locks.
 */
#define AUTO_UNLOCK_ENABLED 0

/* Driving mode: the vehicle state is read every this many exit polls, so once a second. */
#define VEHICLE_POLL_EVERY 4
/*
 * Slow link while driving: 1 s interval, and the fob may skip 4 events, so it listens every 5 s.
 * 16 s supervision timeout. Units: 1.25 ms for the interval, 10 ms for the timeout.
 */
#define DRIVE_CONN_PARAM BT_LE_CONN_PARAM(800, 800, 4, 1600)

/* Step 7: rounds per ranging session, compared with the lock radius. */
#define RANGING_ROUNDS 3
#define LOCK_RADIUS_CM 300

#define LINK_TIMEOUT K_SECONDS(5)
/*
 * Pause before scanning again after a session ends, including after a lock:
 * the gap between CONNECT_RSSI_DBM and EXIT_RSSI_DBM already keeps a fob at
 * the boundary from flapping between lock and unlock.
 */
#define RETRY_DELAY K_SECONDS(2)

/* Deep idle (power.h): listen this long, once per period, for the fob. */
#define BURST_MS       1000
#define BURST_PERIOD_S 60

/* Why exit detection stopped watching the link. */
enum exit_cause {
	EXIT_WALKED_AWAY, /* link RSSI dropped with the car parked */
	EXIT_LINK_LOST,   /* supervision timeout or any other loss: fob out of range */
	EXIT_FOB_CLOSED,  /* the fob ended the link itself, e.g. to sleep at rest */
};

/* Reason of the last disconnection, set before EV_DISCONNECTED is posted. */
static uint8_t disc_reason;

struct adv_report {
	bt_addr_le_t addr;
	int8_t rssi;
};

K_EVENT_DEFINE(car_events);
K_MSGQ_DEFINE(adv_q, sizeof(struct adv_report), 16, 4);

static struct bt_conn *conn;

/* Exponential moving average, alpha = 1/4. */
static int rssi_filter(int ema, int sample, bool first)
{
	return first ? sample : ema + (sample - ema) / 4;
}

static void scan_cb(const bt_addr_le_t *addr, int8_t rssi, uint8_t adv_type,
		    struct net_buf_simple *ad)
{
	struct adv_report report = {.addr = *addr, .rssi = rssi};

	/* Cheap pre-filter here; the AES check runs in the main thread. */
	if (adv_type != BT_GAP_ADV_TYPE_ADV_IND || addr->type != BT_ADDR_LE_RANDOM ||
	    !BT_ADDR_IS_RPA(&addr->a)) {
		return;
	}
	k_msgq_put(&adv_q, &report, K_NO_WAIT);
}

static void connected(struct bt_conn *c, uint8_t err)
{
	k_event_post(&car_events, err ? EV_CONN_FAILED : EV_CONNECTED);
}

static void disconnected(struct bt_conn *c, uint8_t reason)
{
	LOG_INF("disconnected (reason 0x%02x)", reason);
	disc_reason = reason;
	k_event_post(&car_events, EV_DISCONNECTED);
}

static void security_changed(struct bt_conn *c, bt_security_t level, enum bt_security_err err)
{
	k_event_post(&car_events, err ? EV_SEC_FAILED : EV_SECURED);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.security_changed = security_changed,
};

/*
 * Step 2: scan until the fob's RPA resolves with a strong enough filtered RSSI.
 * Returns -EAGAIN once timeout runs out first.
 */
static int scan_for_fob(bt_addr_le_t *fob, k_timeout_t timeout)
{
	/* No duplicate filtering: every advert feeds the RSSI filter. */
	struct bt_le_scan_param param = {
		.type = BT_LE_SCAN_TYPE_PASSIVE,
		.options = BT_LE_SCAN_OPT_NONE,
		.interval = BT_GAP_SCAN_FAST_INTERVAL,
		.window = BT_GAP_SCAN_FAST_WINDOW,
	};
	k_timepoint_t end = sys_timepoint_calc(timeout);
	char addr_str[BT_ADDR_LE_STR_LEN];
	struct adv_report report;
	int ema = 0;
	int above = 0;
	bool first = true;
	int err;

	k_msgq_purge(&adv_q);
	err = bt_le_scan_start(&param, scan_cb);
	if (err) {
		LOG_ERR("scan start failed (err %d)", err);
		return err;
	}
	LOG_INF("scanning for fob");

	for (;;) {
		if (k_msgq_get(&adv_q, &report, sys_timepoint_timeout(end))) {
			err = -EAGAIN;
			break;
		}
		if (!rpa_is_fob(&report.addr)) {
			continue;
		}
		ema = rssi_filter(ema, report.rssi, first);
		first = false;
		above = ema >= CONNECT_RSSI_DBM ? above + 1 : 0;

		bt_addr_le_to_str(&report.addr, addr_str, sizeof(addr_str));
		LOG_INF("fob resolved %s rssi=%d ema=%d", addr_str, report.rssi, ema);
		if (above >= CONNECT_SAMPLES) {
			*fob = report.addr;
			break;
		}
	}

	bt_le_scan_stop();
	return err;
}

/* Deep idle: one burst of continuous scanning. True if the fob was heard, at any RSSI. */
static bool fob_heard_in_burst(void)
{
	struct bt_le_scan_param param = {
		.type = BT_LE_SCAN_TYPE_PASSIVE,
		.options = BT_LE_SCAN_OPT_NONE,
		.interval = BT_GAP_SCAN_FAST_INTERVAL,
		.window = BT_GAP_SCAN_FAST_INTERVAL,
	};
	k_timepoint_t end = sys_timepoint_calc(K_MSEC(BURST_MS));
	struct adv_report report;
	bool heard = false;

	k_msgq_purge(&adv_q);
	if (bt_le_scan_start(&param, scan_cb)) {
		return false;
	}
	while (!heard && k_msgq_get(&adv_q, &report, sys_timepoint_timeout(end)) == 0) {
		heard = rpa_is_fob(&report.addr);
	}
	bt_le_scan_stop();
	return heard;
}

/*
 * Step 2 with long-parking protection: scans normally until POWER_DEEP_IDLE,
 * then in bursts, back to normal as soon as the fob is heard or the wake input
 * is pressed, and enters System OFF at POWER_OFF.
 */
static int wait_for_fob(bt_addr_le_t *fob)
{
	enum power_stage stage, prev = POWER_NORMAL;
	int err;

	for (;;) {
		stage = power_stage();
		if (stage != prev) {
			LOG_INF("power stage %d -> %d", prev, stage);
			prev = stage;
		}

		switch (stage) {
		case POWER_NORMAL:
			err = scan_for_fob(fob, power_until_deep_idle());
			if (err != -EAGAIN) {
				return err;
			}
			break;
		case POWER_DEEP_IDLE:
			if (fob_heard_in_burst()) {
				LOG_INF("fob heard, back to normal scanning");
				power_note_activity();
			} else {
				power_wait_wake(K_MSEC(BURST_PERIOD_S * MSEC_PER_SEC - BURST_MS));
			}
			break;
		case POWER_OFF:
			power_off();
			break;
		}
	}
}

/* Steps 2-3: connect and encrypt the link (LE Secure Connections, Just Works). */
static int connect_secure(const bt_addr_le_t *fob)
{
	uint32_t ev;
	int err;

	k_event_clear(&car_events, UINT32_MAX);
	LOG_INF("fob in range, connecting");
	err = bt_conn_le_create(fob, BT_CONN_LE_CREATE_CONN, BT_LE_CONN_PARAM_DEFAULT, &conn);
	if (err) {
		LOG_WRN("connection create failed (err %d)", err);
		return err;
	}
	ev = car_wait(EV_CONNECTED | EV_CONN_FAILED, LINK_TIMEOUT);
	if (!(ev & EV_CONNECTED)) {
		LOG_WRN("connection failed");
		return -ENOTCONN;
	}

	LOG_INF("connected, encrypting");
	err = bt_conn_set_security(conn, BT_SECURITY_L2);
	if (err) {
		LOG_WRN("set security failed (err %d)", err);
		return err;
	}
	ev = car_wait(EV_SECURED | EV_SEC_FAILED, LINK_TIMEOUT);
	if (!(ev & EV_SECURED)) {
		LOG_WRN("encryption failed");
		return -EACCES;
	}
	LOG_INF("link encrypted (L%d)", bt_conn_get_security(conn));
	return 0;
}

static int read_conn_rssi(int8_t *rssi)
{
	struct bt_hci_cp_read_rssi *cp;
	struct bt_hci_rp_read_rssi *rp;
	struct net_buf *buf;
	struct net_buf *rsp = NULL;
	uint16_t handle;
	int err;

	err = bt_hci_get_conn_handle(conn, &handle);
	if (err) {
		return err;
	}
	buf = bt_hci_cmd_alloc(K_SECONDS(1));
	if (!buf) {
		return -ENOBUFS;
	}
	cp = net_buf_add(buf, sizeof(*cp));
	cp->handle = sys_cpu_to_le16(handle);

	err = bt_hci_cmd_send_sync(BT_HCI_OP_READ_RSSI, buf, &rsp);
	if (err) {
		return err;
	}
	rp = (void *)rsp->data;
	*rssi = rp->rssi;
	net_buf_unref(rsp);
	return 0;
}

/*
 * Driving mode: a slow link the fob barely wakes for, and the fob told not to sleep at rest. The fob
 * hears it while the link is fast: before slowing it, after speeding it up. Whatever fails is retried
 * on the next vehicle poll.
 */
static void sync_driving(bool driving, bool *link_slow, bool *fob_told)
{
	if (driving && !*fob_told) {
		*fob_told = auth_client_drive(conn, true) == 0;
	}
	if (driving != *link_slow) {
		int err = bt_conn_le_param_update(conn, driving ? DRIVE_CONN_PARAM
								: BT_LE_CONN_PARAM_DEFAULT);

		if (err) {
			LOG_WRN("conn param update failed (err %d)", err);
		} else {
			*link_slow = driving;
			LOG_INF("%s link", driving ? "driving: slow" : "parked: fast");
		}
	}
	if (!driving && *fob_told) {
		*fob_told = auth_client_drive(conn, false) != 0;
	}
}

/*
 * Step 6: polls the link RSSI until the driver walks away with the car parked, or the link ends.
 * While the car is driven, the RSSI is ignored and the link runs slow (sync_driving()).
 */
static enum exit_cause wait_for_exit(void)
{
	unsigned int ticks = 0;
	unsigned int samples = 0;
	bool driving = false;
	bool link_slow = false;
	bool fob_told = false;
	bool read_failed = false;
	int ema = 0;
	int below = 0;
	int8_t rssi;
	int err;

	LOG_INF("exit detection armed (link RSSI < %d dBm for %d samples)", EXIT_RSSI_DBM,
		EXIT_SAMPLES);
	for (;;) {
		if (car_wait(0, K_MSEC(EXIT_POLL_MS))) {
			/*
			 * A clean terminate came from the fob over the encrypted link, so it was
			 * still in range: nothing says the driver left.
			 */
			if (disc_reason == BT_HCI_ERR_REMOTE_USER_TERM_CONN) {
				LOG_INF("fob closed the link while unlocked, not locking");
				return EXIT_FOB_CLOSED;
			}
			LOG_WRN("link lost while unlocked, treating as exit");
			return EXIT_LINK_LOST;
		}
		if (ticks++ % VEHICLE_POLL_EVERY == 0) {
			driving = vehicle_poll() == VEHICLE_DRIVING;
			sync_driving(driving, &link_slow, &fob_told);
		}
		if (driving || link_slow || fob_told) {
			/* No exit while driven, nor before the link and the fob are back to parked. */
			samples = 0;
			below = 0;
			continue;
		}
		err = read_conn_rssi(&rssi);
		if (err) {
			if (!read_failed) {
				LOG_WRN("link RSSI read failed (err %d)", err);
				read_failed = true;
			}
			continue;
		}
		ema = rssi_filter(ema, rssi, samples == 0);
		below = ema < EXIT_RSSI_DBM ? below + 1 : 0;
		if (++samples % 4 == 0) {
			LOG_INF("link rssi=%d ema=%d", rssi, ema);
		}
		if (below < EXIT_SAMPLES) {
			continue;
		}

		LOG_INF("exit started: link ema=%d dBm below %d dBm", ema, EXIT_RSSI_DBM);
		if (vehicle_is_parked()) {
			return EXIT_WALKED_AWAY;
		}
		below = 0;
	}
}

/* Step 7: one ranging session. Returns the longest run of rounds beyond the lock radius. */
static int ranging_session(const uint8_t session_key[PKE_KEY_LEN])
{
	uint8_t sts_key[PKE_KEY_LEN];
	uint32_t distance_cm;
	int consecutive = 0;
	int best = 0;

	if (auth_client_range(conn, session_key, sts_key)) {
		return 0;
	}
	for (int i = 0; i < RANGING_ROUNDS; i++) {
		int err = ranging_measure(sts_key, &distance_cm);

		/* -ENOTSUP: the UWB dummy, which always passes. */
		if (err == -ENOTSUP || (err == 0 && distance_cm > LOCK_RADIUS_CM)) {
			consecutive++;
		} else {
			consecutive = 0;
		}
		best = MAX(best, consecutive);
	}
	memset(sts_key, 0, sizeof(sts_key));
	return best;
}

/* Steps 2-8 for one approach of the fob. */
static void run_session(const bt_addr_le_t *fob)
{
	uint8_t session_key[PKE_KEY_LEN];
	enum exit_cause cause;
	bool locked = false;
	bool armed = true;

	if (connect_secure(fob) || auth_client_run(conn, session_key)) {
		return;
	}

	/* The mic only listens while this authenticated fob is connected. */
	tap_listener_start();

#if AUTO_UNLOCK_ENABLED
	uint32_t distance_cm;
	/* Step 4 */
	int err = ranging_measure(session_key, &distance_cm);

	/* Step 5, never while driving: a fob reconnecting mid-drive must not open the doors. */
	if (vehicle_poll() == VEHICLE_DRIVING) {
		LOG_INF("car is being driven, unlock skipped");
	} else {
		armed = unlock_decide(pke_key_id(session_key), err ? -1 : (int32_t)distance_cm);
	}
#else
	LOG_INF("auto-unlock off, double tap to unlock");
#endif
	if (armed) {
		/*
		 * Steps 6-8: stay connected so exit detection can read the link RSSI.
		 * A lost link means the fob is out of BLE range, so beyond any lock radius.
		 * A link the fob closed itself leaves the doors as they are.
		 */
		do {
			cause = wait_for_exit();
			if (cause == EXIT_FOB_CLOSED) {
				break;
			}
			locked = lock_decide(cause == EXIT_WALKED_AWAY ? ranging_session(session_key)
								 : LOCK_K_ROUNDS);
		} while (!locked && cause == EXIT_WALKED_AWAY);
	}

	tap_listener_stop();
	door_sleep();
	memset(session_key, 0, sizeof(session_key));
}

static void drop_conn(void)
{
	if (!conn) {
		return;
	}
	if (bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN) == 0) {
		car_wait(0, K_SECONDS(2));
	}
	bt_conn_unref(conn);
	conn = NULL;
}

int main(void)
{
	bt_addr_le_t fob;
	int err;

	boot_led_play();
	err = pke_crypto_init();
	if (!err) {
		err = rpa_init();
	}
	if (err) {
		LOG_ERR("crypto init failed (err %d)", err);
		return 0;
	}
	/* Neither is fatal: BLE still runs, and door_send() retries the CAN start. */
	door_init();
	tap_listener_init();
	err = power_init();
	if (err) {
		LOG_ERR("wake input init failed (err %d)", err);
	}

	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("bluetooth init failed (err %d)", err);
		return 0;
	}
	LOG_INF("car ready");

	for (;;) {
		if (wait_for_fob(&fob)) {
			k_sleep(RETRY_DELAY);
			continue;
		}
		run_session(&fob);
		drop_conn();
		power_note_activity();
		k_sleep(RETRY_DELAY);
	}
}
