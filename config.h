#pragma once

#include <mcp_can.h>

// ---- Debug ----
#define DEBUG_ENABLED 1 // set to 0 to silence Serial debug output

#if DEBUG_ENABLED
  #define DEBUG_PRINT(...)   Serial.print(__VA_ARGS__)
  #define DEBUG_PRINTF(...)  Serial.printf(__VA_ARGS__)
  #define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
  #define DEBUG_PRINT(...)
  #define DEBUG_PRINTF(...)
  #define DEBUG_PRINTLN(...)
#endif

// ---- MCP2515 CAN controller (SPI, ESP32 VSPI: SCK=18, MISO=19, MOSI=23) ----
static const uint8_t MCP_CS_PIN  = 5;
static const uint8_t MCP_INT_PIN = 4; // reserved; not needed for TX-only use

// Check the crystal printed on your module (8MHz or 16MHz). A wrong value still returns CAN_OK
// from begin(), but frames won't ACK on the bus.
#define MCP_OSC_FREQ  MCP_8MHZ
#define MCP_CAN_SPEED CAN_500KBPS

static const uint32_t MCP_RETRY_INTERVAL_MS = 1000;

// ---- Door lock/unlock frame (verified on 2018 Lexus CT200h, see obd-tool) ----
static const uint32_t DOOR_CMD_ID = 0x750;
static const uint8_t LOCK_CMD_DATA[8]   = {0x40, 0x05, 0x30, 0x11, 0x00, 0x80, 0x00, 0x00};
static const uint8_t UNLOCK_CMD_DATA[8] = {0x40, 0x05, 0x30, 0x11, 0x00, 0x40, 0x00, 0x00};

// ---- Bus wake-up ----
// A sleeping bus has no node awake to ACK, so the door frame fails. There is no secret "wake
// frame": per ISO 11898-2 a transceiver leaves standby on any dominant-recessive-dominant bus
// activity, so transmitting *anything* is the wake signal. We use a read-only OBD-II request
// (mode 01 PID 00, "supported PIDs") rather than the door frame, so the frames sent into a
// sleeping bus have no side effect if they land oddly.
// Capturing a real wake (door opened from inside) showed the gateway's startup sequence on 0x45A,
// with byte 1 stepping and byte 2 carrying a flag:
//
//   5A 00 80 42 01 0F 0F 0F   <- first frame of the wake
//   5A 01 80 42 01 00 00 00
//   5A 02 00 / 5A 01 00       <- alternating handshake, ~4 cycles at 100/260 ms
//   5A 04 00 42 01 00 00 00   <- steady state, 1 Hz
//   5A 14 00 42 01 00 00 00   <- last frame before sleep
//
// So 5A 00 80 ... 0F 0F 0F is the *initial* wake frame, not the failure artifact it was first taken
// for - it looked like one only because, in listen-only mode, it was the frame being retried.
static const uint32_t WAKE_CMD_ID = 0x45A;
static const uint8_t WAKE_CMD_DATA[8] = {0x5A, 0x00, 0x80, 0x42, 0x01, 0x0F, 0x0F, 0x0F};

// Waking by imitating the gateway's own network-management frame. An earlier attempt used an OBD-II
// diagnostic request (0x7DF) and never worked - an NM-governed network has no reason to answer that.
// Sniffing showed the only traffic on an idle bus is 0x45A at ~1 Hz from the central gateway
// (CGW1N02 in opendbc), so that is what gets imitated instead. See ../obd-tool/README.md.
// Settled by measurement: CAN injection cannot wake this car. Replaying the gateway's exact startup
// sequence on 0x45A, frame for frame and gap for gap, gave "16 sent, 0 acked" with the bus still
// silent - while the same sequence appears verbatim when a door is opened. Zero ACKs is the tell:
// during deep sleep no module is listening at all, so no frame of any content can reach one. These
// frames are what the gateway emits *after* something else woke it, not the cause of the wake.
//
// Leave this off. A failed attempt costs ~2 s and drives the controller error-passive for nothing.
#define WAKE_BUS_ENABLED 0

// Periodic, because NM is periodic: a single frame proves nothing. The burst stops the moment any
// traffic appears, which also keeps us off the air once the real gateway starts sending this same ID
// - two transmitters on one ID produce error frames.
static const uint8_t  WAKE_MAX_BURSTS  = 20;  // wake frames sent before giving up
static const uint32_t WAKE_LISTEN_MS   = 100; // listen for bus traffic after each wake frame
static const uint8_t  DOOR_MAX_RETRIES   = 3;  // door frame attempts before giving up
static const uint32_t DOOR_RETRY_GAP_MS  = 150; // pause between attempts, spent draining RX
static const uint32_t DOOR_REPLY_WAIT_MS = 200; // how long to wait for the body ECU's 758 reply

// Diagnostic responses come back on the request ID + 8. A frame here proves the body ECU processed
// the command, which a bare ACK does not.
static const uint32_t DOOR_RESPONSE_ID = 0x758;

// How long after the last received frame the bus is still assumed awake. Used to skip a send that
// would only fail, so the controller isn't driven error-passive for nothing.
static const uint32_t BUS_AWAKE_TIMEOUT_MS = 2000;

// An ACK error normally adds 8 to the transmit error counter, but an error-passive transmitter that
// sees no dominant bit during its passive error flag does not increment it — so TEC stops at exactly
// 128 and never reaches the 256 bus-off limit. Resetting the chip clears it back to 0.
static const uint8_t TEC_REINIT_LIMIT = 128;

// ---- GPIO ----
static const uint8_t LED_PIN = 2;

// ---- INMP441 I2S microphone ----
#define I2S_WS_PIN   25
#define I2S_SCK_PIN  33
#define I2S_SD_PIN   32
#define I2S_PORT     I2S_NUM_0

#define SAMPLE_RATE_HZ 16000
#define BLOCK_SIZE     128   // power of 2 (also the FFT size)
#define SAMPLE_SHIFT   6     // adjust if the signal saturates or is too weak
