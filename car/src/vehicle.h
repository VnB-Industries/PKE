/* Vehicle state read over OBD-II (README step 6 and "Driving mode"). */
#ifndef VEHICLE_H
#define VEHICLE_H

#include <stdbool.h>

enum vehicle_state {
	VEHICLE_OFF,     /* no ECU answers: ignition off or bus asleep */
	VEHICLE_PARKED,  /* ignition on, standing still (gear in P once the gear check is enabled) */
	VEHICLE_DRIVING, /* moving, or in gear */
};

/*
 * One OBD-II round (speed, then gear when enabled). Blocks up to ~VEHICLE_REPLY_MS per request,
 * plus ~1.2 s the first time if the MCP2515 sleeps. Never transmits on a quiet bus.
 */
enum vehicle_state vehicle_poll(void);

/* Step 6 precondition: true unless the last vehicle_poll() saw the car being driven. */
bool vehicle_is_parked(void);

#endif /* VEHICLE_H */
