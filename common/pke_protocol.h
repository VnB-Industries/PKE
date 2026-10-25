/* GATT protocol shared by the fob and the car (README steps 3 and 7). */
#ifndef PKE_PROTOCOL_H
#define PKE_PROTOCOL_H

#include <zephyr/bluetooth/uuid.h>

#define PKE_SVC_UUID_VAL  BT_UUID_128_ENCODE(0x6b1e0001, 0x5f3a, 0x4c1d, 0x9a2e, 0x3c7d1f0b8a01)
#define PKE_AUTH_UUID_VAL BT_UUID_128_ENCODE(0x6b1e0002, 0x5f3a, 0x4c1d, 0x9a2e, 0x3c7d1f0b8a01)
#define PKE_SVC_UUID      BT_UUID_DECLARE_128(PKE_SVC_UUID_VAL)
#define PKE_AUTH_UUID     BT_UUID_DECLARE_128(PKE_AUTH_UUID_VAL)

#define PKE_NONCE_LEN 16
#define PKE_MAC_LEN   32
#define PKE_KEY_LEN   32

/*
 * Every message on the Auth characteristic starts with its type byte.
 * The car writes, the fob answers with a notification.
 */
enum pke_msg_type {
	PKE_MSG_HELLO = 0x01,     /* car: nonce_C */
	PKE_MSG_FOB_PROOF = 0x02, /* fob: nonce_F | HMAC(K, "fob" | nonce_C | nonce_F) */
	PKE_MSG_CAR_PROOF = 0x03, /* car: HMAC(K, "car" | nonce_C | nonce_F) */
	PKE_MSG_ACK = 0x04,       /* fob: status */
	PKE_MSG_RANGE_REQ = 0x05, /* car: nonce_R, starts a UWB ranging session */
	PKE_MSG_RANGE_ACK = 0x06, /* fob: status */
	PKE_MSG_DRIVE = 0x07,     /* car: 1 while the car is driven, 0 once parked; fob answers ACK */
};

#define PKE_HELLO_LEN     (1 + PKE_NONCE_LEN)
#define PKE_FOB_PROOF_LEN (1 + PKE_NONCE_LEN + PKE_MAC_LEN)
#define PKE_CAR_PROOF_LEN (1 + PKE_MAC_LEN)
#define PKE_ACK_LEN       2
#define PKE_RANGE_REQ_LEN (1 + PKE_NONCE_LEN)
#define PKE_RANGE_ACK_LEN 2
#define PKE_DRIVE_LEN     2
#define PKE_MSG_MAX_LEN   PKE_FOB_PROOF_LEN

#define PKE_STATUS_OK   0x00
#define PKE_STATUS_FAIL 0x01

/* Role labels mixed into the proofs, so a fob proof can't be replayed as a car proof. */
#define PKE_LABEL_FOB "fob"
#define PKE_LABEL_CAR "car"
#define PKE_LABEL_LEN 3

/* HKDF info strings. */
#define PKE_INFO_SESSION "pke-session-v1"
#define PKE_INFO_STS     "pke-sts-v1"

#endif /* PKE_PROTOCOL_H */
