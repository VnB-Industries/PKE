/* Car side of the PKE GATT protocol. Both calls block the caller and return 0 or a negative errno. */
#ifndef AUTH_CLIENT_H
#define AUTH_CLIENT_H

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/bluetooth/conn.h>

#include "pke_protocol.h"

/*
 * Step 3, on an encrypted link: finds the Auth characteristic, runs the mutual
 * challenge-response and derives the session key.
 */
int auth_client_run(struct bt_conn *conn, uint8_t session_key[PKE_KEY_LEN]);

/* Step 7: sends RANGE_REQ with a fresh nonce and derives the STS key from the session key. */
int auth_client_range(struct bt_conn *conn, const uint8_t session_key[PKE_KEY_LEN],
		      uint8_t sts_key[PKE_KEY_LEN]);

/* Tells the fob whether the car is being driven, so it keeps the link instead of sleeping at rest. */
int auth_client_drive(struct bt_conn *conn, bool driving);

#endif /* AUTH_CLIENT_H */
