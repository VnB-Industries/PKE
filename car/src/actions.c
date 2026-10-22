#include <zephyr/logging/log.h>

#include "actions.h"
#include "door.h"

LOG_MODULE_REGISTER(actions, LOG_LEVEL_INF);

bool unlock_decide(uint32_t session_key_id, int32_t distance_cm)
{
	/*
	 * The UWB dummy gives no distance (distance_cm < 0) and always passes. Later: compare
	 * distance_cm with the unlock radius before unlocking.
	 */
	if (distance_cm < 0) {
		LOG_INF("UNLOCK decision: fob authenticated, key id %08x, distance n/a (UWB dummy) "
			"-> unlock", session_key_id);
	} else {
		LOG_INF("UNLOCK decision: fob authenticated, key id %08x, distance %d cm -> unlock",
			session_key_id, distance_cm);
	}

	/*
	 * True even when the frame was not delivered: the session stays up, so exit detection
	 * still runs and a double tap retries the unlock. Dropping the link instead would
	 * reconnect every few seconds and pulse the relay each time.
	 */
	if (!door_send(DOOR_UNLOCK)) {
		LOG_WRN("unlock not delivered; double tap to retry");
	}
	return true;
}

bool lock_decide(int rounds_above_radius)
{
	if (rounds_above_radius < LOCK_K_ROUNDS) {
		LOG_INF("LOCK decision: %d/%d rounds above radius -> keep waiting",
			rounds_above_radius, LOCK_K_ROUNDS);
		return false;
	}
	LOG_INF("LOCK decision: %d/%d rounds above radius -> lock", rounds_above_radius,
		LOCK_K_ROUNDS);

	/* A failed lock returns false, so the next exit trigger retries it while still linked. */
	return door_send(DOOR_LOCK);
}
