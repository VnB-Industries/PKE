/*
 * OBD-II requests behind vehicle_poll(). See README "Driving mode" for how to
 * verify them on the car with obd-tool before relying on them.
 */
#ifndef VEHICLE_CONFIG_H
#define VEHICLE_CONFIG_H

#include <stdint.h>

/* ---- Speed: standard OBD-II service 01, PID 0x0D, km/h in one byte ---- */
#define OBD_FUNC_REQ_ID 0x7DF /* functional request, every emission ECU listens */
#define OBD_RSP_ID      0x7E8 /* first physical response id */
#define OBD_RSP_SPAN    8     /* 0x7E8..0x7EF */
static const uint8_t SPEED_REQ[8] = {0x02, 0x01, 0x0D, 0x00, 0x00, 0x00, 0x00, 0x00};

/*
 * ---- Gear: Toyota hybrid ECU, service 0x21 ----
 * The PID, byte and P value are not verified for the CT200h yet. Query the hybrid ECU with obd-tool
 * in P, R, N, D and B, fill these in, then set VEHICLE_GEAR_ENABLED. Only a single-frame reply
 * (first byte 0x0N) is decoded. Until then, "parked" means speed 0 for VEHICLE_STOP_HOLD_S.
 */
#define VEHICLE_GEAR_ENABLED 0
#define GEAR_REQ_ID  0x7E2
#define GEAR_RSP_ID  0x7EA
#define GEAR_PID     0x00 /* to verify */
#define GEAR_BYTE    3    /* reply byte holding the shift position, to verify */
#define GEAR_P_VALUE 0x00 /* its value in P, to verify */
static const uint8_t GEAR_REQ[8] = {0x02, 0x21, GEAR_PID, 0x00, 0x00, 0x00, 0x00, 0x00};

/* Per-request wait for the ECU reply. */
#define VEHICLE_REPLY_MS 100

/* Unanswered polls in a row before a driven car counts as off, so one lost reply doesn't flip it. */
#define VEHICLE_MISSES 3

/*
 * Without the gear, a stop at a light is speed 0 like a parked car: stay "driving" for this long
 * after the last motion. With the gear, P decides at once.
 */
#define VEHICLE_STOP_HOLD_S (VEHICLE_GEAR_ENABLED ? 0 : 10)

#endif /* VEHICLE_CONFIG_H */
