/*
 * Door command constants, ported from the ESP32 config.h (branch ESP32/main).
 * Frames verified on a 2018 Lexus CT200h; see README "Waking a sleeping bus".
 */
#ifndef DOOR_CONFIG_H
#define DOOR_CONFIG_H

#include <stdint.h>

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
 * An unACKed frame is retransmitted by the MCP2515 forever, so a send that is not ACKed within this
 * long is aborted (see send_frame()).
 */
#define DOOR_TX_TIMEOUT_MS 100

/*
 * How long after the last received frame the bus is still assumed awake. Used to skip a send that
 * would only fail, so the controller isn't driven error-passive for nothing.
 */
#define BUS_AWAKE_TIMEOUT_MS 2000

/* After a query frame that couldn't be delivered, door_query() refuses to send for this long. */
#define QUERY_HOLDOFF_MS 30000

/* ---- CAN module power (door_sleep()) ---- */
/*
 * Outside a fob session the P-MOS on D7 cuts the module's supply (can-power-gpios). Register values
 * are from the MCP2515 datasheet; the Zephyr driver keeps its own copies private.
 */
#define MCP_OPCODE_RESET      0xC0
#define MCP_OPCODE_READ       0x03
#define MCP_OPCODE_BIT_MODIFY 0x05
#define MCP_ADDR_CANSTAT      0x0E
#define MCP_ADDR_CANCTRL      0x0F
#define MCP_ADDR_CNF3         0x28
#define MCP_ADDR_CNF2         0x29
#define MCP_CNF2_SAM          0x40 /* sample each bit three times */
#define MCP_ADDR_CNF1         0x2A
#define MCP_ADDR_TXB0CTRL     0x30 /* TXB1CTRL/TXB2CTRL follow every 0x10 */
#define MCP_ADDR_CANINTE      0x2B
#define MCP_ADDR_CANINTF      0x2C
#define MCP_MODE_MASK         0xE0 /* CANCTRL.REQOP / CANSTAT.OPMOD */
#define MCP_ADDR_EFLG         0x2D
#define MCP_ADDR_TEC          0x1C
#define MCP_OSC_START_MS      5    /* crystal start-up after a reset, before talking to the chip */
/*
 * Supply rise (~1 ms with the board's 100 nF gate capacitor), the module's decoupling and the
 * MCP2515's power-on reset, before the first SPI transfer.
 */
#define MCP_POWER_UP_MS       10
/* Powers the module at boot after the GPIO drivers (40) and before the MCP2515 driver (80). */
#define CAN_POWER_INIT_PRIORITY 60
/*
 * After power-up the chip has missed everything, so listen this long before judging the bus. The
 * gateway's NM frame ticks ~1 Hz while the bus is responsive.
 */
#define CAN_WAKE_LISTEN_MS    1200
/*
 * Sample point used when the MCP2515 has to be reconfigured after losing its setup. With the 8 MHz
 * crystal a bit is 8 TQ at 500 kbit/s and PS2 is at least 2, so 75 % is the highest reachable.
 */
#define CAN_SAMPLE_POINT_PERMILLE 750

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
 * The pin and its polarity are relay-gpios in app.overlay.
 */
#define RELAY_WAKE_ENABLED    1
#define RELAY_PULSE_COUNT     5    /* open/close cycles per wake attempt */
#define RELAY_PULSE_ON_MS     1500 /* "door open" dwell (relay energised) */
#define RELAY_PULSE_OFF_MS    1500 /* "door closed" dwell (relay released) */
#define RELAY_WAKE_TIMEOUT_MS 8000 /* after the train, wait this long for the bus to come alive */

#endif /* DOOR_CONFIG_H */
