/*
 * Door command constants, as in car/src/door_config.h on nrf52840/main.
 * Frames verified on a 2018 Lexus CT200h; see README "Waking a sleeping bus".
 */
#ifndef DOOR_CONFIG_H
#define DOOR_CONFIG_H

#include <stdint.h>

#include <mcp_can.h>

/* ---- MCP2515 ---- */
/* Check the crystal printed on the module (8 or 16 MHz): a wrong value still starts, but frames won't ACK. */
#define MCP_OSC_FREQ  MCP_8MHZ
#define MCP_CAN_SPEED CAN_500KBPS

/* ---- Door lock/unlock frame ---- */
#define DOOR_CMD_ID 0x750
static const uint8_t LOCK_CMD_DATA[8] = {0x40, 0x05, 0x30, 0x11, 0x00, 0x80, 0x00, 0x00};
static const uint8_t UNLOCK_CMD_DATA[8] = {0x40, 0x05, 0x30, 0x11, 0x00, 0x40, 0x00, 0x00};

/*
 * Diagnostic responses come back on the request ID + 8. A frame here proves the body ECU processed
 * the command, which a bare ACK does not.
 */
#define DOOR_RESPONSE_ID 0x758

#define DOOR_MAX_RETRIES   3   /* door frame attempts before giving up */
#define DOOR_RETRY_GAP_MS  150 /* pause between attempts */
#define DOOR_REPLY_WAIT_MS 200 /* how long to wait for the body ECU's 758 reply */

/*
 * How long after the last received frame the bus is still assumed awake. Used to skip a send that
 * would only fail, so the controller isn't driven error-passive for nothing.
 */
#define BUS_AWAKE_TIMEOUT_MS 2000

/* ---- Relay wake (drives the driver's-door jamb-switch line to wake a sleeping body ECU) ---- */
/*
 * In deep sleep (~10+ min idle) the body ECU no longer listens on CAN, so no frame can reach it
 * (measured: 16 wake frames, 0 ACKed). It wakes on its hardwired door-courtesy switches instead.
 * A relay wired in PARALLEL with the driver's jamb switch (Toyota 84231-60070) reproduces a door
 * opening: the ECU holds the line high via its internal pull-up (door closed), the switch grounds
 * it (door open), and closing the relay grounds it the same way. We pulse a train of open/close
 * cycles, as Toyota's service procedure for a sleeping main body ECU does, then send the door frame.
 *
 * Validate on the car with the taped-plunger test before relying on this (see tests/reveil-ecu-relais.md).
 * With no relay wired, the relay pin just toggles an unconnected GPIO, which is harmless.
 * The pin and its polarity are RELAY_PIN and RELAY_ACTIVE_HIGH in board.h.
 */
#define RELAY_WAKE_ENABLED    1
#define RELAY_PULSE_COUNT     5    /* open/close cycles per wake attempt */
#define RELAY_PULSE_ON_MS     1500 /* "door open" dwell (relay energised) */
#define RELAY_PULSE_OFF_MS    1500 /* "door closed" dwell (relay released) */
#define RELAY_WAKE_TIMEOUT_MS 8000 /* after the train, wait this long for the bus to come alive */

#endif /* DOOR_CONFIG_H */
