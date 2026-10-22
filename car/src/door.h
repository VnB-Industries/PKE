/* Door lock/unlock over CAN, with the relay wake for a body ECU in deep sleep. */
#ifndef DOOR_H
#define DOOR_H

#include <stdbool.h>

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
 * Sends the door command, retrying a few times. Wakes the MCP2515 if it sleeps. Thread-safe and blocking:
 * an unlock into a sleeping bus can take ~25 s of relay pulses. True when the frame was ACKed.
 */
bool door_send(enum door_cmd cmd);

/*
 * Puts the MCP2515 in Sleep mode until the next door_send(), which wakes it. While asleep it neither
 * receives nor ACKs, like an empty OBD port.
 */
void door_sleep(void);

#ifdef __cplusplus
}
#endif

#endif /* DOOR_H */
