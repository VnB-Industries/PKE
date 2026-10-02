# PKE

Full Passive Entry System

## Tap-to-lock/unlock firmware (PKE.ino)

ESP32 firmware that unlocks the car on a **double tap** and locks it on a **triple tap**, detected by an INMP441 microphone on the window and sent over CAN through an MCP2515.

Combines [../tapDetector](../tapDetector) (tap detection) and [../obd-tool](../obd-tool) (CAN door frames).

## Hardware

- ESP32 DevKit
- INMP441 I2S microphone: VDD→3.3V, GND→GND, L/R→GND, WS→GPIO25, SCK→GPIO33, SD→GPIO32
- MCP2515 + SN65HVD230 CAN module: see the wiring table in [../obd-tool/README.md](../obd-tool/README.md) (CS→GPIO5, INT→GPIO4, SCK/MISO/MOSI→18/19/23, VCC→3.3V)

No pin conflicts between the two.

## Build

Arduino IDE with the ESP32 core, plus libraries `arduinoFFT` (v2.x) and `MCP_CAN_lib` (coryjfowler). Set `MCP_OSC_FREQ` in `config.h` to match your module's crystal. Serial monitor at 115200.

## Behaviour

| Gesture | Action |
| --- | --- |
| 2 taps, 150–600 ms apart, then 1 s of silence | unlock |
| 3 taps, 150–700 ms between each | lock |

The 1 s silence after a double tap is what lets a third tap turn it into a lock instead. The cost is a 1 s delay before unlock.

LED: fast blink = CAN not ready, slow blink = on-bus, solid 1 s = command acknowledged, 2 s stutter = command not delivered.

## Waking a sleeping bus

A few minutes after the car is parked, its ECUs sleep and CAN goes silent. A CAN frame needs at least one other node to ACK it, so the door frame fails with `code=7` (send timeout) and the transmit error counter climbs — at 255 the MCP2515 goes bus-off and stops transmitting until it's reset.

### CAN wake-up was tried and does not work on this car

There is no secret wake frame. Per ISO 11898-2 a transceiver leaves standby on any dominant–recessive–dominant bus activity, so *transmitting at all* is the wake signal. That was implemented (send a read-only OBD-II request on `0x7DF`, listen for any ECU to answer, then retry the door frame) and it **failed on the 2018 CT200h**:

```text
[can] door frame failed: code=7 EFLG=0x15 TEC=128 REC=0
[can] bus appears asleep, sending wake frames...
[can] no traffic seen; bus may still be asleep
```

`REC=0` means not one bit ever came back, and no amount of bus activity changed that. The reason is that **a Toyota main body ECU wakes on its hardwired inputs — the door courtesy switches — not on CAN activity.** Toyota's own service procedure for a sleeping main body ECU is physical: with the engine switch off, open the driver door, then open and close a door several times at ~1.5 s intervals. Bus wake-up is a feature of partial-networking transceivers on newer platforms; this one doesn't have it.

So `WAKE_BUS_ENABLED` defaults to `0`. The code is kept for platforms where it does work.

### What the error flags mean

`EFLG=0x15` is EWARN + TXWAR + TXEP: error-passive, **not** bus-off (that would be bit 5, `0x20`). `TEC` pins at exactly 128 and never climbs, because an error-passive transmitter that sees no dominant bit during its passive error flag doesn't increment TEC. The MCP2515 therefore can't reach the 256 bus-off limit this way — the earlier worry about bus-off was unfounded, and `TEC_REINIT_LIMIT` is 128 so the reset still fires and clears the state for the next gesture.

### Current behaviour

The firmware polls RX continuously, so it knows whether the bus is alive without transmitting. On a gesture it tries the door frame, and if nothing answers it stops rather than retrying into a dead bus. The log distinguishes a silent bus from a frame refused on an active bus (which would point at bitrate or wiring instead). The LED goes solid 1 s when a command was acknowledged and stutters for 2 s when it wasn't, so a failure is visible from outside the car.

An ACK only proves *some* node is awake, not the body ECU specifically, so a successful send still doesn't guarantee the doors moved.

### What actually fixed it: stay in normal mode and retry

The earlier conclusion here — that a sleeping car can't be driven from the OBD port — was wrong. Two measurements with the sniffer (see [../obd-tool/README.md](../obd-tool/README.md)) changed it:

1. **Acknowledging matters.** The gateway is the only other node on the diagnostic segment. If this node doesn't ACK its ~1 Hz NM frame, the gateway retransmits at ~1400 frames/s and the wake aborts. So the firmware must run the MCP2515 in normal mode and keep draining RX — which it does.
2. **A quiet bus is not necessarily a sleeping one.** A door frame sent after **42 s of silence**, with no fob press, was ACKed *and* positively answered by the body ECU. As long as this node stays present and ACKs, those quiet stretches look like gaps in the NM cycle rather than deep sleep.

So `sendDoorFrame()` now always attempts the send, retries `DOOR_MAX_RETRIES` times with `DOOR_RETRY_GAP_MS` between attempts, and never gives up on the first failure. The previous version bailed out immediately whenever `WAKE_BUS_ENABLED` was off, which meant it only ever tried once.

It also waits briefly for the reply on `0x758` and logs `body ECU confirmed` when byte 2 is `0x70` — the positive response to service `0x30`. That is real confirmation the doors were commanded, unlike an ACK, which only says some node heard the frame.

### Deep sleep is real, and CAN cannot defeat it

Two regimes, and the difference decides the design:

| | Bus | Door commands |
| --- | --- | --- |
| **Responsive** (minutes after a physical event) | `0x45A` ticks 1 Hz, gaps of 36–48 s | **work** — ACKed, `0x758` replies |
| **Deep sleep** (roughly 10+ min idle) | nothing at all | fail, nothing acknowledges |

Measured facts:

- A door command succeeded **42 s** after the bus went silent, and again **~2.5 min** after a door-open wake. No fob needed.
- Staying present and ACKing does **not** prevent sleep: the gateway ticked for 68 s with this node ACKing throughout, then slept anyway.
- After ~13 min idle, nothing works.

Four wake attempts failed: an OBD-II request on `0x7DF`, a burst of the steady-state NM payload, and a full replay of the gateway's startup sequence with its captured timing — `16 sent, 0 acked`. The zero is the point. If any module were watching the bus, it would have ACKed. In deep sleep none is listening, so no frame of any content can reach one. The `0x45A` sequence is what the gateway emits *after* something else wakes it; it is a symptom, not a cause. ISO 11898-6 selective wake is evidently not in play on this segment.

### Where that leaves it

- **Lock works today.** You lock seconds after walking away, well inside the responsive window.
- **Unlock on return does not**, once the car has sat. That is the whole remaining problem.

The honest fix is to wake the body ECU the way the car does: **drive its hardwired wake input** (a door courtesy switch line). Zero standby drain, works indefinitely, but it means tapping the car's wiring rather than living on the OBD port.

A keepalive remains theoretically possible — injecting NM frames *during* the responsive window to extend it — but it cannot help after deep sleep, `0x45A` collides with the real gateway while it is awake, and holding the network up drains a 45 Ah hybrid aux battery. Not recommended.

## Calibration

Thresholds are copied from tapDetector and depend on your mic gain and the car window. Set `ENVELOPE_DEBUG = true` in `PKE.ino` and use the Serial Plotter to tune `RMS_THRESHOLD`, `PEAK_THRESHOLD` and the release thresholds.

## Safety

Anyone who can tap your window can unlock the car. Use only while parked, and verify the CAN frames before using this on a vehicle other than the 2018 Lexus CT200h.
