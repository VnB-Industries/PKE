#include "actions.h"
#include "door.h"
#include "log.h"

LOG_MODULE("actions");

bool unlock_decide(int rssi_dbm)
{
	/*
	 * Simplified authentication: an advert whose RPA resolves with the fob IRK is enough. There
	 * is no challenge-response and no UWB distance check (see README "Safety").
	 */
	LOG_INF("UNLOCK decision: fob RPA resolved, rssi ema=%d dBm -> unlock", rssi_dbm);

	/*
	 * True even when the frame was not delivered: the session stays up, so exit detection
	 * still runs and a double tap retries the unlock. Ending it instead would detect the fob
	 * again within seconds and pulse the relay each time.
	 */
	if (!door_send(DOOR_UNLOCK)) {
		LOG_WRN("unlock not delivered; double tap to retry");
	}
	return true;
}

bool lock_decide(void)
{
	LOG_INF("LOCK decision: fob walked away -> lock");

	/* A failed lock returns false, so the next exit trigger retries it while the fob is still seen. */
	return door_send(DOOR_LOCK);
}
