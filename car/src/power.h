/*
 * Long-parking protection: the less often a fob session starts, the less the
 * car unit scans, down to System OFF (README "Power").
 */
#ifndef POWER_H
#define POWER_H

#include <stdbool.h>

#include <zephyr/kernel.h>

/* Without a fob session for this long, scanning drops to short bursts. */
#define POWER_DEEP_IDLE_AFTER_S (2 * 24 * 3600)
/* Without one for this long, the unit enters System OFF until the wake input. */
#define POWER_OFF_AFTER_S (4 * 24 * 3600)

enum power_stage {
	POWER_NORMAL,
	POWER_DEEP_IDLE,
	POWER_OFF,
};

/*
 * Sets up the wake input and starts the idle clock. can_ok says the MCP2515
 * came up, so CAN bus activity can also wake the unit (BUS_WAKE_ENABLED).
 * Returns 0, or a negative errno if the input is missing. The unit then relies
 * on the CAN bus alone, or stops at POWER_DEEP_IDLE if that is unavailable too,
 * since nothing could bring it back from System OFF.
 */
int power_init(bool can_ok);

/* Restarts the idle clock: a fob session ran, or the fob was heard nearby. */
void power_note_activity(void);

/* The stage the idle clock is in now. A press of the wake input counts as activity. */
enum power_stage power_stage(void);

/* Time left in POWER_NORMAL. */
k_timeout_t power_until_deep_idle(void);

/* Waits up to timeout for the wake input. True, with the idle clock restarted, if it came. */
bool power_wait_wake(k_timeout_t timeout);

/*
 * Enters System OFF; the wake input going active, or CAN bus activity, wakes the unit, which then
 * reboots. Returns only if nothing could wake it: it then stays at POWER_DEEP_IDLE.
 */
void power_off(void);

#endif /* POWER_H */
