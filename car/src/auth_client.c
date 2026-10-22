#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/gatt.h>

#include "auth_client.h"
#include "car.h"
#include "pke_crypto.h"

LOG_MODULE_REGISTER(auth_client, LOG_LEVEL_INF);

/* Budget for each exchange, measured once the link is encrypted. */
#define EXCHANGE_TIMEOUT_MS 3000

static const struct bt_uuid_128 auth_uuid = BT_UUID_INIT_128(PKE_AUTH_UUID_VAL);

/* The Bluetooth stack keeps pointers to these until their callback runs. */
static struct bt_gatt_exchange_params mtu_params;
static struct bt_gatt_discover_params disc_params;
static struct bt_gatt_discover_params ccc_disc_params;
static struct bt_gatt_subscribe_params sub_params;
static struct bt_gatt_write_params write_params;

static uint16_t auth_handle;
static uint8_t att_err;
static uint8_t rx_buf[PKE_MSG_MAX_LEN];
static uint16_t rx_len;
static int64_t deadline;

static void mtu_cb(struct bt_conn *conn, uint8_t err, struct bt_gatt_exchange_params *params)
{
	att_err = err;
	k_event_post(&car_events, EV_MTU_DONE);
}

static uint8_t discover_cb(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			   struct bt_gatt_discover_params *params)
{
	if (attr) {
		const struct bt_gatt_chrc *chrc = attr->user_data;

		auth_handle = chrc->value_handle;
	}
	k_event_post(&car_events, EV_DISCOVERED);
	return BT_GATT_ITER_STOP;
}

static void subscribe_cb(struct bt_conn *conn, uint8_t err, struct bt_gatt_subscribe_params *params)
{
	att_err = err;
	k_event_post(&car_events, EV_SUBSCRIBED);
}

static uint8_t notify_cb(struct bt_conn *conn, struct bt_gatt_subscribe_params *params,
			 const void *data, uint16_t len)
{
	if (!data) {
		/* Unsubscribed, e.g. on disconnect. */
		params->value_handle = 0;
		return BT_GATT_ITER_STOP;
	}
	rx_len = MIN(len, sizeof(rx_buf));
	memcpy(rx_buf, data, rx_len);
	k_event_post(&car_events, EV_RX);
	return BT_GATT_ITER_CONTINUE;
}

static void write_cb(struct bt_conn *conn, uint8_t err, struct bt_gatt_write_params *params)
{
	att_err = err;
	k_event_post(&car_events, EV_WRITE_DONE);
}

static k_timeout_t remaining(void)
{
	return K_MSEC(MAX(deadline - k_uptime_get(), 0));
}

/* Clears `bit` and the last ATT error before starting the operation that posts it. */
static void begin(uint32_t bit)
{
	k_event_clear(&car_events, bit);
	att_err = 0;
}

static int wait_att(uint32_t bit, const char *what)
{
	if (!(car_wait(bit, remaining()) & bit)) {
		LOG_WRN("%s: timeout or link lost", what);
		return -ETIMEDOUT;
	}
	if (att_err) {
		LOG_WRN("%s: ATT error 0x%02x", what, att_err);
		return -EIO;
	}
	return 0;
}

/* Writes msg to the Auth characteristic and waits for the fob's notification of rsp_type. */
static int exchange(struct bt_conn *conn, const uint8_t *msg, uint16_t len, uint8_t rsp_type,
		    uint16_t rsp_len)
{
	int err;

	begin(EV_WRITE_DONE | EV_RX);
	write_params = (struct bt_gatt_write_params){
		.func = write_cb,
		.handle = auth_handle,
		.data = msg,
		.length = len,
	};
	err = bt_gatt_write(conn, &write_params);
	if (err) {
		LOG_WRN("write 0x%02x failed (err %d)", msg[0], err);
		return err;
	}
	err = wait_att(EV_WRITE_DONE, "write");
	if (err) {
		return err;
	}
	if (!(car_wait(EV_RX, remaining()) & EV_RX)) {
		LOG_WRN("no reply to 0x%02x", msg[0]);
		return -ETIMEDOUT;
	}
	if (rx_len != rsp_len || rx_buf[0] != rsp_type) {
		LOG_WRN("unexpected reply 0x%02x (len %u)", rx_buf[0], rx_len);
		return -EPROTO;
	}
	return 0;
}

/* MTU exchange, Auth characteristic discovery and notification subscription. */
static int setup_link(struct bt_conn *conn)
{
	int err;

	begin(EV_MTU_DONE);
	mtu_params.func = mtu_cb;
	err = bt_gatt_exchange_mtu(conn, &mtu_params);
	if (!err) {
		err = wait_att(EV_MTU_DONE, "MTU exchange");
	}
	if (err && err != -EALREADY) {
		return err;
	}
	if (bt_gatt_get_mtu(conn) < PKE_MSG_MAX_LEN + 3) {
		LOG_WRN("ATT MTU %u too small", bt_gatt_get_mtu(conn));
		return -EMSGSIZE;
	}

	begin(EV_DISCOVERED);
	auth_handle = 0;
	disc_params = (struct bt_gatt_discover_params){
		.uuid = &auth_uuid.uuid,
		.func = discover_cb,
		.start_handle = BT_ATT_FIRST_ATTRIBUTE_HANDLE,
		.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE,
		.type = BT_GATT_DISCOVER_CHARACTERISTIC,
	};
	err = bt_gatt_discover(conn, &disc_params);
	if (!err) {
		err = wait_att(EV_DISCOVERED, "discovery");
	}
	if (err) {
		return err;
	}
	if (!auth_handle) {
		LOG_WRN("PKE Auth characteristic not found");
		return -ENOENT;
	}

	begin(EV_SUBSCRIBED);
	sub_params = (struct bt_gatt_subscribe_params){
		.notify = notify_cb,
		.subscribe = subscribe_cb,
		.value_handle = auth_handle,
		.ccc_handle = BT_GATT_AUTO_DISCOVER_CCC_HANDLE,
		.end_handle = BT_ATT_LAST_ATTRIBUTE_HANDLE,
		.disc_params = &ccc_disc_params,
		.value = BT_GATT_CCC_NOTIFY,
	};
	err = bt_gatt_subscribe(conn, &sub_params);
	if (!err) {
		err = wait_att(EV_SUBSCRIBED, "subscribe");
	}
	return err;
}

int auth_client_run(struct bt_conn *conn, uint8_t session_key[PKE_KEY_LEN])
{
	uint8_t hello[PKE_HELLO_LEN] = {PKE_MSG_HELLO};
	uint8_t proof[PKE_CAR_PROOF_LEN] = {PKE_MSG_CAR_PROOF};
	uint8_t nonce_c[PKE_NONCE_LEN];
	uint8_t nonce_f[PKE_NONCE_LEN];
	int err;

	deadline = k_uptime_get() + EXCHANGE_TIMEOUT_MS;
	err = setup_link(conn);
	if (err) {
		return err;
	}

	err = pke_random(nonce_c, sizeof(nonce_c));
	if (err) {
		return err;
	}
	memcpy(hello + 1, nonce_c, PKE_NONCE_LEN);
	LOG_INF("sending HELLO");
	err = exchange(conn, hello, sizeof(hello), PKE_MSG_FOB_PROOF, PKE_FOB_PROOF_LEN);
	if (err) {
		return err;
	}

	memcpy(nonce_f, rx_buf + 1, PKE_NONCE_LEN);
	if (pke_proof_verify(PKE_LABEL_FOB, nonce_c, nonce_f, rx_buf + 1 + PKE_NONCE_LEN)) {
		LOG_WRN("fob proof invalid");
		return -EACCES;
	}
	LOG_INF("fob proof valid, sending CAR_PROOF");

	err = pke_proof(PKE_LABEL_CAR, nonce_c, nonce_f, proof + 1);
	if (!err) {
		err = exchange(conn, proof, sizeof(proof), PKE_MSG_ACK, PKE_ACK_LEN);
	}
	if (err) {
		return err;
	}
	if (rx_buf[1] != PKE_STATUS_OK) {
		LOG_WRN("fob rejected the car proof");
		return -EACCES;
	}

	err = pke_session_key(nonce_c, nonce_f, session_key);
	if (err) {
		return err;
	}
	LOG_INF("mutual auth OK, session key id=%08x", pke_key_id(session_key));
	return 0;
}

int auth_client_range(struct bt_conn *conn, const uint8_t session_key[PKE_KEY_LEN],
		      uint8_t sts_key[PKE_KEY_LEN])
{
	uint8_t req[PKE_RANGE_REQ_LEN] = {PKE_MSG_RANGE_REQ};
	int err;

	deadline = k_uptime_get() + EXCHANGE_TIMEOUT_MS;
	err = pke_random(req + 1, PKE_NONCE_LEN);
	if (err) {
		return err;
	}
	LOG_INF("sending RANGE_REQ");
	err = exchange(conn, req, sizeof(req), PKE_MSG_RANGE_ACK, PKE_RANGE_ACK_LEN);
	if (err) {
		return err;
	}
	if (rx_buf[1] != PKE_STATUS_OK) {
		LOG_WRN("fob refused the ranging session");
		return -EACCES;
	}

	err = pke_sts_key(session_key, req + 1, sts_key);
	if (err) {
		return err;
	}
	LOG_INF("sts key id=%08x", pke_key_id(sts_key));
	return 0;
}
