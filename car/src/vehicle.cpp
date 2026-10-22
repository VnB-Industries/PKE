#include "log.h"
#include "vehicle.h"

LOG_MODULE("vehicle");

bool vehicle_is_parked(void)
{
	/*
	 * Stub. Later: a door-closed frame, gear in P, or no ECU answer because
	 * the bus is asleep, held for T1 ~ 3-5 s. An RSSI drop seen before the
	 * precondition holds should be latched for a few seconds, so the exit
	 * arms as soon as the precondition is met.
	 */
	LOG_INF("vehicle parked (stub: door/gear state not read)");
	return true;
}
