/*
 * Car unit wiring on the ESP32 DevKit (README "Hardware"). Takes the place of
 * app.overlay on nrf52840/main.
 */
#ifndef BOARD_H
#define BOARD_H

#include <stdint.h>

#include <driver/i2s.h>

/* ---- MCP2515 CAN controller (SPI on VSPI: SCK=18, MISO=19, MOSI=23) ---- */
static const uint8_t MCP_CS_PIN = 5;
/* Active low, falls when a frame is received (door.cpp). */
static const uint8_t MCP_INT_PIN = 4;

/* ---- Relay wake ---- */
/* GPIO driving the relay coil, through a transistor and a flyback diode. */
static const uint8_t RELAY_PIN = 13;
/* Energised = relay closed = jamb-switch line grounded = "driver's door open". */
static const bool RELAY_ACTIVE_HIGH = true;

/* ---- INMP441 I2S microphone (L/R tied to GND: left channel) ---- */
static const int I2S_WS_PIN = 25;
static const int I2S_SCK_PIN = 33;
static const int I2S_SD_PIN = 32;
#define I2S_PORT I2S_NUM_0

#endif /* BOARD_H */
