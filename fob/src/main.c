/*
 * PKE fob: advertises with a resolvable private address built from the
 * enrolled IRK and answers the car's challenge-response over an encrypted link.
 *
 * Bluetooth callbacks only queue events; the crypto and the replies run in the
 * main thread, which owns all the state below.
 */
#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/version.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>

#include "keys.h"
#include "pke_crypto.h"
#include "pke_protocol.h"

LOG_MODULE_REGISTER(fob, LOG_LEVEL_INF);

/* A car that has not finished mutual auth this long after connecting is dropped. */
#define AUTH_TIMEOUT_MS 5000

/* ~250 ms advertising interval, in 0.625 ms units. */
#define ADV_INTERVAL_MIN 0x0190
#define ADV_INTERVAL_MAX 0x01a0

#if ZEPHYR_VERSION_CODE >= ZEPHYR_VERSION(4, 0, 0)
#define ADV_OPT_CONN BT_LE_ADV_OPT_CONN
#else
#define ADV_OPT_CONN BT_LE_ADV_OPT_CONNECTABLE
#endif

enum fob_state {
	FOB_IDLE,
	FOB_WAIT_HELLO,
	FOB_WAIT_CAR_PROOF,
	FOB_AUTHED,
};

enum fob_event {
	FOB_EV_CONNECTED,
	FOB_EV_DISCONNECTED,
	FOB_EV_RECYCLED,
	FOB_EV_WRITE,
};

struct fob_msg {
	uint8_t event;
	uint8_t len;
	uint8_t data[PKE_MSG_MAX_LEN];
};

K_MSGQ_DEFINE(fob_q, sizeof(struct fob_msg), 8, 4);

static struct bt_conn *cur_conn;
static enum fob_state state;
static int64_t auth_deadline;
static uint8_t nonce_c[PKE_NONCE_LEN];
static uint8_t nonce_f[PKE_NONCE_LEN];
static uint8_t session_key[PKE_KEY_LEN];

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
};

static void post_event(uint8_t event)
{
	struct fob_msg msg = {.event = event};

	if (k_msgq_put(&fob_q, &msg, K_NO_WAIT)) {
		LOG_ERR("event queue full, event %u lost", event);
	}
}

static ssize_t auth_write(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
			  uint16_t len, uint16_t offset, uint8_t flags)
{
	struct fob_msg msg = {.event = FOB_EV_WRITE, .len = len};

	if (offset) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}
	if (len == 0 || len > sizeof(msg.data)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);
	}
	memcpy(msg.data, buf, len);
	if (k_msgq_put(&fob_q, &msg, K_NO_WAIT)) {
		return BT_GATT_ERR(BT_ATT_ERR_UNLIKELY);
	}
	return len;
}

/* Writes and the CCC both require an encrypted link. */
BT_GATT_SERVICE_DEFINE(pke_svc,
	BT_GATT_PRIMARY_SERVICE(PKE_SVC_UUID),
	BT_GATT_CHARACTERISTIC(PKE_AUTH_UUID, BT_GATT_CHRC_WRITE | BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_WRITE_ENCRYPT, NULL, auth_write, NULL),
	BT_GATT_CCC(NULL, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE_ENCRYPT));

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		return;
	}
	cur_conn = bt_conn_ref(conn);
	post_event(FOB_EV_CONNECTED);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("disconnected (reason 0x%02x)", reason);
	post_event(FOB_EV_DISCONNECTED);
}

static void recycled(void)
{
	post_event(FOB_EV_RECYCLED);
}

static void security_changed(struct bt_conn *conn, bt_security_t level, enum bt_security_err err)
{
	if (err) {
		LOG_WRN("encryption failed (err %d)", err);
	} else {
		LOG_INF("link encrypted (L%d)", level);
	}
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.recycled = recycled,
	.security_changed = security_changed,
};

static void advertise(void)
{
	int err = bt_le_adv_start(BT_LE_ADV_PARAM(ADV_OPT_CONN, ADV_INTERVAL_MIN, ADV_INTERVAL_MAX,
						  NULL),
				  ad, ARRAY_SIZE(ad), NULL, 0);

	if (err && err != -EALREADY) {
		LOG_ERR("advertising failed (err %d)", err);
		return;
	}
	LOG_INF("advertising (RPA, rotates every %d s)", CONFIG_BT_RPA_TIMEOUT);
}

static void notify(const uint8_t *msg, uint16_t len)
{
	int err = bt_gatt_notify(cur_conn, &pke_svc.attrs[1], msg, len);

	if (err) {
		LOG_WRN("notify failed (err %d)", err);
	}
}

static void drop_link(void)
{
	state = FOB_IDLE;
	bt_conn_disconnect(cur_conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
}

static void on_hello(const uint8_t *nonce)
{
	uint8_t rsp[PKE_FOB_PROOF_LEN] = {PKE_MSG_FOB_PROOF};

	memcpy(nonce_c, nonce, PKE_NONCE_LEN);
	if (pke_random(nonce_f, PKE_NONCE_LEN) ||
	    pke_proof(PKE_LABEL_FOB, nonce_c, nonce_f, rsp + 1 + PKE_NONCE_LEN)) {
		LOG_ERR("crypto failure building FOB_PROOF");
		drop_link();
		return;
	}
	memcpy(rsp + 1, nonce_f, PKE_NONCE_LEN);

	LOG_INF("HELLO received, sending FOB_PROOF");
	state = FOB_WAIT_CAR_PROOF;
	notify(rsp, sizeof(rsp));
}

static void on_car_proof(const uint8_t *mac)
{
	uint8_t ack[PKE_ACK_LEN] = {PKE_MSG_ACK, PKE_STATUS_OK};
	int err;

	err = pke_proof_verify(PKE_LABEL_CAR, nonce_c, nonce_f, mac);
	if (!err) {
		err = pke_session_key(nonce_c, nonce_f, session_key);
	}
	if (err) {
		LOG_WRN("car proof rejected (err %d), dropping link", err);
		ack[1] = PKE_STATUS_FAIL;
		notify(ack, sizeof(ack));
		drop_link();
		return;
	}

	state = FOB_AUTHED;
	LOG_INF("auth OK, session key id=%08x", pke_key_id(session_key));
	notify(ack, sizeof(ack));
}

static void on_range_req(const uint8_t *nonce_r)
{
	uint8_t ack[PKE_RANGE_ACK_LEN] = {PKE_MSG_RANGE_ACK, PKE_STATUS_OK};
	uint8_t sts_key[PKE_KEY_LEN];

	if (pke_sts_key(session_key, nonce_r, sts_key)) {
		LOG_ERR("STS key derivation failed");
		ack[1] = PKE_STATUS_FAIL;
		notify(ack, sizeof(ack));
		return;
	}
	LOG_INF("RANGE_REQ received, sts key id=%08x", pke_key_id(sts_key));
	notify(ack, sizeof(ack));

	/*
	 * Step 7, fob side. Later: power up the DW3000, answer 3-5 DS-TWR rounds
	 * with the STS seeded from sts_key, then power it down.
	 */
	LOG_INF("UWB stub: would power DW3000, run 3-5 DS-TWR rounds, power down");
	memset(sts_key, 0, sizeof(sts_key));
}

static void on_write(const uint8_t *data, uint8_t len)
{
	if (state == FOB_IDLE) {
		return;
	}
	if (state == FOB_WAIT_HELLO && data[0] == PKE_MSG_HELLO && len == PKE_HELLO_LEN) {
		on_hello(data + 1);
	} else if (state == FOB_WAIT_CAR_PROOF && data[0] == PKE_MSG_CAR_PROOF &&
		   len == PKE_CAR_PROOF_LEN) {
		on_car_proof(data + 1);
	} else if (state == FOB_AUTHED && data[0] == PKE_MSG_RANGE_REQ &&
		   len == PKE_RANGE_REQ_LEN) {
		on_range_req(data + 1);
	} else {
		LOG_WRN("unexpected message 0x%02x (len %u) in state %d, dropping link", data[0],
			len, state);
		drop_link();
	}
}

int main(void)
{
	bt_addr_le_t id_addr = {.type = BT_ADDR_LE_RANDOM, .a.val = PKE_FOB_ID_ADDR_INIT};
	uint8_t irk[16] = PKE_FOB_IRK_INIT;
	char addr_str[BT_ADDR_LE_STR_LEN];
	struct fob_msg msg;
	int err;

	err = pke_crypto_init();
	if (err) {
		LOG_ERR("crypto init failed (err %d)", err);
		return 0;
	}

	/* Step 1: the identity created before bt_enable() becomes the default one, with our IRK. */
	err = bt_id_create(&id_addr, irk);
	if (err < 0) {
		LOG_ERR("identity create failed (err %d)", err);
		return 0;
	}
	err = bt_enable(NULL);
	if (err) {
		LOG_ERR("bluetooth init failed (err %d)", err);
		return 0;
	}
	bt_addr_le_to_str(&id_addr, addr_str, sizeof(addr_str));
	LOG_INF("fob ready, identity %s", addr_str);
	advertise();

	for (;;) {
		k_timeout_t timeout = K_FOREVER;

		if (state == FOB_WAIT_HELLO || state == FOB_WAIT_CAR_PROOF) {
			timeout = K_MSEC(MAX(auth_deadline - k_uptime_get(), 0));
		}
		if (k_msgq_get(&fob_q, &msg, timeout)) {
			LOG_WRN("car did not authenticate within %d ms, dropping link",
				AUTH_TIMEOUT_MS);
			drop_link();
			continue;
		}

		switch (msg.event) {
		case FOB_EV_CONNECTED:
			bt_addr_le_to_str(bt_conn_get_dst(cur_conn), addr_str, sizeof(addr_str));
			LOG_INF("car connected (%s)", addr_str);
			state = FOB_WAIT_HELLO;
			auth_deadline = k_uptime_get() + AUTH_TIMEOUT_MS;
			break;
		case FOB_EV_DISCONNECTED:
			bt_conn_unref(cur_conn);
			cur_conn = NULL;
			state = FOB_IDLE;
			memset(session_key, 0, sizeof(session_key));
			break;
		case FOB_EV_RECYCLED:
			advertise();
			break;
		case FOB_EV_WRITE:
			on_write(msg.data, msg.len);
			break;
		}
	}
}
