/* Door lock/unlock over CAN, with the relay wake for a body ECU in deep sleep. */
#ifndef DOOR_H
#define DOOR_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Bus wake (CAN activity wakes the unit from System OFF) ---- */
/*
 * The reverse of the relay wake: here the car wakes the unit, not the unit the car. Opening the
 * car (key, original remote, a door) brings the bus up; the gateway's NM frame then ticks ~1 Hz on
 * the OBD segment (README "Deep sleep is real"). Just before System OFF, door_arm_bus_wake() sets
 * WAKIE in the sleeping MCP2515, so the first bus activity pulls its INT low, and the INT pin gets
 * SENSE like the wake input. The SN65HVD230 receiver stays on in standby, so no hardware change.
 * Validate on the car before relying on it alone (tests/reveil-unite-bus-can.md). Set to 0 to leave
 * System OFF to the wake input only.
 */
#define BUS_WAKE_ENABLED 1

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

/*
 * Lets CAN bus activity wake the unit from System OFF: puts the MCP2515 to sleep with WAKIE set and
 * gives its INT pin SENSE. Call right before sys_poweroff(). Returns 0 or a negative errno, in which
 * case only the wake input can wake the unit.
 */
int door_arm_bus_wake(void);

#ifdef __cplusplus
}
#endif

#endif /* DOOR_H */
