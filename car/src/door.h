/* Door lock/unlock over CAN, with the relay wake for a body ECU in deep sleep. */
#ifndef DOOR_H
#define DOOR_H

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/kernel.h>

#ifdef __cplusplus
extern "C" {
#endif

enum door_cmd {
	DOOR_UNLOCK,
	DOOR_LOCK,
};

/* Sets up the CAN controller, then puts it to sleep, and releases the relay. Returns 0 or a negative errno. */
int door_init(void);

/*
 * Sends the door command, retrying a few times. Thread-safe and blocking:
 * an unlock into a sleeping bus can take ~25 s of relay pulses. True when the frame was ACKed.
 */
bool door_send(enum door_cmd cmd);

/*
 * Sends one request frame and waits for a reply from an id in [rsp_id, rsp_id + rsp_span). Wakes the
 * MCP2515 but never the bus: with no traffic for BUS_AWAKE_TIMEOUT_MS it returns -ENETDOWN without
 * transmitting. Returns 0 with the reply in rsp, or a negative errno.
 */
int door_query(uint16_t req_id, const uint8_t req[8], uint16_t rsp_id, uint8_t rsp_span,
	       uint8_t rsp[8], k_timeout_t timeout);

#ifdef __cplusplus
}
#endif

#endif /* DOOR_H */
