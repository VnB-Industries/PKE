/*
 * Long-parking protection: the less often a fob session starts, the less the
 * car unit scans, down to System OFF (README "Power").
 */
#ifndef POWER_H
#define POWER_H

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
 * Sets up the wake input and starts the idle clock. Returns 0, or a negative
 * errno if the input is missing: the unit then stops at POWER_DEEP_IDLE, since
 * nothing could bring it back from System OFF.
 */
int power_init(void);

/* Restarts the idle clock: a fob session ran, or the fob was heard nearby. */
void power_note_activity(void);

/* The stage the idle clock is in now. A press of the wake input counts as activity. */
enum power_stage power_stage(void);

/* Time left in POWER_NORMAL. */
k_timeout_t power_until_deep_idle(void);

/* Waits up to timeout for the wake input. True, with the idle clock restarted, if it came. */
bool power_wait_wake(k_timeout_t timeout);

/* Enters System OFF; the wake input going active wakes the unit, which then reboots. Doesn't return. */
void power_off(void);

#endif /* POWER_H */
