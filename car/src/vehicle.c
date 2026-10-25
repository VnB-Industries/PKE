#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "door.h"
#include "vehicle.h"
#include "vehicle_config.h"

LOG_MODULE_REGISTER(vehicle, LOG_LEVEL_INF);

static const char *const state_names[] = {"off", "parked", "driving"};

static enum vehicle_state state = VEHICLE_OFF;
static int64_t last_moving_ms;
static int misses;

/* Speed in km/h, or a negative errno when no ECU answered. */
static int read_speed(void)
{
	uint8_t rsp[8];
	int err = door_query(OBD_FUNC_REQ_ID, SPEED_REQ, OBD_RSP_ID, OBD_RSP_SPAN, rsp,
			     K_MSEC(VEHICLE_REPLY_MS));

	if (err) {
		return err;
	}
	/* Single frame: length, 0x41 (positive response to service 01), the PID, then the value. */
	if (rsp[0] < 3 || rsp[1] != 0x41 || rsp[2] != 0x0D) {
		return -EPROTO;
	}
	return rsp[3];
}

#if VEHICLE_GEAR_ENABLED
/* True in P, false in any other position, or a negative errno when unread. */
static int read_gear_is_park(void)
{
	uint8_t rsp[8];
	int err = door_query(GEAR_REQ_ID, GEAR_REQ, GEAR_RSP_ID, 1, rsp, K_MSEC(VEHICLE_REPLY_MS));

	if (err) {
		return err;
	}
	/* Single frame only: length, 0x61 (positive response to service 0x21), the PID, the data. */
	if ((rsp[0] & 0xF0) != 0 || rsp[1] != 0x61 || rsp[2] != GEAR_PID ||
	    GEAR_BYTE > (rsp[0] & 0x0F)) {
		return -EPROTO;
	}
	return rsp[GEAR_BYTE] == GEAR_P_VALUE;
}
#endif

enum vehicle_state vehicle_poll(void)
{
	int64_t now = k_uptime_get();
	enum vehicle_state next;
	int speed = read_speed();

	if (speed < 0) {
		/* A driven car that misses a reply or two is still driven. */
		next = state == VEHICLE_DRIVING && ++misses < VEHICLE_MISSES ? VEHICLE_DRIVING
									  : VEHICLE_OFF;
	} else {
		bool moving = speed > 0;

		misses = 0;
#if VEHICLE_GEAR_ENABLED
		/* An unread gear leaves the decision to the speed. */
		if (read_gear_is_park() == 0) {
			moving = true;
		}
#endif
		if (moving) {
			last_moving_ms = now;
		}
		next = moving || (state == VEHICLE_DRIVING &&
				  now - last_moving_ms < VEHICLE_STOP_HOLD_S * MSEC_PER_SEC)
			       ? VEHICLE_DRIVING
			       : VEHICLE_PARKED;
	}

	if (next != state) {
		if (speed >= 0) {
			LOG_INF("vehicle %s (%d km/h)", state_names[next], speed);
		} else {
			LOG_INF("vehicle %s (no OBD-II reply, err %d)", state_names[next], speed);
		}
		state = next;
	}
	return state;
}

bool vehicle_is_parked(void)
{
	return state != VEHICLE_DRIVING;
}
