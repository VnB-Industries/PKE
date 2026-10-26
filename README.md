# PKE

Passive Entry System for a 2018 Lexus CT200h. The car unit runs on an **ESP32 DevKit** (Arduino core, PlatformIO). The fob is a Seeed XIAO nRF52840 (Plus) running the fob firmware from the `nrf52840/main` branch.

- **Fob**: advertises with a rotating Resolvable Private Address (RPA). Its firmware is unchanged from `nrf52840/main`.
- **Car unit**: scans for the fob and resolves its RPA with the enrolled IRK. When the fob is close, the car unlocks the doors over CAN (MCP2515 on the OBD port), and it locks them when the fob walks away. It wakes a body ECU in deep sleep by pulsing a relay on the driver's door jamb switch.
- **Window taps** (INMP441 mic on the glass, car unit) give manual control: double tap = unlock, triple tap = lock. **The mic only runs while the fob is detected**, so a tap on its own can never open the car.

This is a port of the `nrf52840/main` car unit with **simplified authentication**. The car never connects to the fob: a resolved RPA with a strong enough signal is enough to unlock. There is no challenge-response, no session key and no UWB step. Read [Safety](#safety) before relying on it.

| Directory | Contents |
| --- | --- |
| [car/](car/) | car unit: BLE scanner, RPA resolution, CAN door frames, relay wake, tap listener |
| [common/](common/) | `keys.h`, generated and gitignored, shared with the fob build |
| [scripts/](scripts/) | key generation and build/flash |
| [tests/](tests/) | on-car test protocols |

## Hardware (car unit)

| ESP32 GPIO | Connects to |
| --- | --- |
| 13 | relay coil driver (transistor + flyback diode, or opto-isolated module) |
| 5 | MCP2515 CS |
| 4 | MCP2515 INT (required: received frames wake the CAN task) |
| 18 / 19 / 23 | MCP2515 SCK / MISO / MOSI (VSPI) |
| 33 | INMP441 SCK |
| 25 | INMP441 WS |
| 32 | INMP441 SD |
| 3V3 / GND | MCP2515 + SN65HVD230 module, INMP441 (L/R → GND) |

The pins are set in [car/src/board.h](car/src/board.h). Set `MCP_OSC_FREQ` in [car/src/door_config.h](car/src/door_config.h) to match the crystal on your MCP2515 module (8 MHz by default). A wrong value still starts, but frames won't ACK. The CAN bitrate is 500 kbit/s.

No LEDs are used. Everything is reported on the USB serial log.

## Build & flash

1. Use the same `common/keys.h` as the fob. `keys.h` is gitignored, so a copy generated on `nrf52840/main` stays in place when you switch branches. To start from scratch, run `python3 scripts/gen_keys.py`, then flash the fob from `nrf52840/main` with the new file. The car only uses the fob IRK from it.
2. Build, flash and open the log:
   ```sh
   scripts/flash.sh              # add -p for a pristine build, or a port: scripts/flash.sh /dev/ttyUSB0
   ```
   This runs `pio run -d car -t upload -t monitor`. It uses `pio` from the PATH, or the copy installed by the PlatformIO IDE extension. PlatformIO fetches NimBLE-Arduino and mcp_can (coryjfowler) on the first build.
3. The log is on the DevKit's USB serial port, at 115200 baud.

## How it works

1. **Enrollment**: `scripts/gen_keys.py` creates the fob IRK and its static identity address (plus the shared secret, which only the nRF car firmware uses). The fob is built with them; the car only needs the IRK.
2. **Detection and unlock**: the fob's RPA rotates every 60 s (`CONFIG_BT_RPA_TIMEOUT` in the fob build, short for testing). The car scans passively and continuously, and resolves each RPA advert on the host with AES-128 (ESP32 hardware AES through mbedTLS). Once the filtered advert RSSI holds -70 dBm for 3 adverts, it starts the tap listener and sends the unlock frame ([door.cpp](car/src/door.cpp)), with the relay wake if the bus is asleep. The session stays up even if delivery failed; a double tap retries it.
3. **Exit detection**: the car keeps following the fob's adverts. An exit starts when the filtered RSSI stays below -80 dBm for 5 adverts, or when no advert arrives for 3 s. The vehicle precondition ([vehicle.cpp](car/src/vehicle.cpp)) is a stub that always returns true.
4. **Lock**: on an exit the car sends the lock frame. If the lock failed and the fob is still heard, it waits for the next exit trigger; if the fob is gone, the session ends anyway. The tap listener stops, and the car searches again after 2 s. The 10 dB gap between the detect and exit thresholds stops it from flapping.

BLE thresholds are `#define`s at the top of [car/src/main.cpp](car/src/main.cpp). Door frames, retries and relay timings are in [car/src/door_config.h](car/src/door_config.h).

### Tap gestures (fob detected only)

| Gesture | Action |
| --- | --- |
| 2 taps, 150–600 ms apart | unlock |
| 3 taps, 150–700 ms between each, gaps within ±30% of each other | lock |

A gesture is only matched after 1.2 s of silence following its last tap. That is what lets a third tap turn an unlock into a lock, and it makes a fourth tap cancel the gesture. The cost is a 1.2 s delay before either command.

The detector in [car/src/tap/](car/src/tap/) is a copy of [../tapDetector/src](../tapDetector/src): an onset detector on every sample, temporal and FFT validation of each event, then timing pattern matching. [tap_listener.cpp](car/src/tap_listener.cpp) runs it in two tasks: capture on core 0, processing on core 1. Between sessions the I2S peripheral is stopped and both tasks block, so the mic is unclocked and draws nothing.

### Power

The ESP32 scans continuously, so the car unit draws on the order of 100 mA all the time. That is far more than the nRF52840 version, and it matters on a parked car's 12 V auxiliary battery. Budget for it, or power the unit from a switched supply.

What is kept from the nRF version: there are no LEDs, the I2S clock runs only while the fob is detected, and the relay dwell blocks on a semaphore instead of polling.

The CAN module isn't power-managed on this branch (decided on 26/10): no MCP2515 Sleep and no transceiver standby. The controller goes on the bus at boot and stays there in normal mode, ACKing and receiving every frame, so the bus-awake check always reflects the bus. Leave the SN65HVD230's Rs tied to GND, as the module ships. Its few mA are small next to the ESP32's scan.

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

So the CAN wake is not in this firmware. The earlier tap-only sketch with `WAKE_BUS_ENABLED` (`PKE.ino`) is in this branch's history, for platforms where it does work.

### What the error flags mean

`EFLG=0x15` is EWARN + TXWAR + TXEP: error-passive, **not** bus-off (that would be bit 5, `0x20`). `TEC` pins at exactly 128 and never climbs, because an error-passive transmitter that sees no dominant bit during its passive error flag doesn't increment TEC. The MCP2515 therefore can't reach the 256 bus-off limit this way — the earlier worry about bus-off was unfounded. A door frame that mcp_can can't get ACKed within its 2.5 ms send timeout is dropped by resetting the MCP2515, so it can't stay queued and fire later when the ECU wakes.

### Current behaviour

The firmware receives every frame (a task woken by the MCP2515 INT pin, no polling), so it knows whether the bus is alive without transmitting. On a gesture it tries the door frame, and if nothing answers it stops rather than retrying into a dead bus. The log distinguishes a silent bus from a frame refused on an active bus (which would point at bitrate or wiring instead).

An ACK only proves *some* node is awake, not the body ECU specifically, so a successful send still doesn't guarantee the doors moved.

### What actually fixed it: stay in normal mode and retry

The earlier conclusion here — that a sleeping car can't be driven from the OBD port — was wrong. Two measurements with the sniffer (see [../obd-tool/README.md](../obd-tool/README.md)) changed it:

1. **Acknowledging matters.** The gateway is the only other node on the diagnostic segment. If this node doesn't ACK its ~1 Hz NM frame, the gateway retransmits at ~1400 frames/s and the wake aborts. So the firmware must run the MCP2515 in normal mode and keep draining RX — which it does.
2. **A quiet bus is not necessarily a sleeping one.** A door frame sent after **42 s of silence**, with no fob press, was ACKed *and* positively answered by the body ECU. As long as this node stays present and ACKs, those quiet stretches look like gaps in the NM cycle rather than deep sleep.

So `door_send()` now always attempts the send, retries `DOOR_MAX_RETRIES` times with `DOOR_RETRY_GAP_MS` between attempts, and never gives up on the first failure.

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

The fix is to wake the body ECU the way the car does: **drive its hardwired wake input** — the driver's door jamb switch, Toyota **84231-60070**. Zero standby drain, works indefinitely, but it means tapping the car's wiring rather than living on the OBD port.

### The jamb switch line

A single wire at battery voltage: the body ECU holds one state through an internal pull-up, and the switch's own metal body grounds it to the jamb in the other. A relay wired across that switch and pulsed makes the ECU see a door opening. Wiring, the topology measurement and the board checks are in [../Pin-diagram.md](../Pin-diagram.md); the pulse is implemented as the `wake` command in [../obd-tool](../obd-tool/README.md).

Pulse a train, not once — Toyota's service procedure for a sleeping main body ECU is *open the driver door, then open and close a door several times at ~1.5 s intervals*.

### Relay wake is integrated into the firmware

The relay pulse is built into PKE itself (`RELAY_WAKE_ENABLED` in [car/src/door_config.h](car/src/door_config.h), on by default). On an **unlock** (fob detected or double tap), if the bus reads as asleep, `door_send()` calls `relay_wake()`: it pulses the relay output (GPIO13) as a train of `RELAY_PULSE_COUNT` open/close cycles (`RELAY_PULSE_ON_MS` / `RELAY_PULSE_OFF_MS`, ~1.5 s each), watching CAN RX throughout, and sends the door frame the moment the bus comes alive. Lock is unaffected — it fires in the responsive window, where the bus is already awake and this path isn't taken.

Wire the relay **in parallel** with the driver's jamb switch, one contact on the switch line and one on a good chassis ground, with a common ground to the ESP32. Drive the coil through a transistor and a flyback diode (or use an opto-isolated relay module), contacts dry — switch the line, don't inject voltage into it. With no relay wired, GPIO13 just toggles harmlessly.

**Validate on the car before relying on it** with the taped-plunger test, then the relay test — see [tests/reveil-ecu-relais.md](tests/reveil-ecu-relais.md).

What is established, what isn't, and the test that closes the gap:

- **A door opening does wake it**, with the car fob-locked, and **no alarm sounded** — so the theft deterrent, which reads this same switch, does not veto the approach. That was the risk that would have killed it.
- **Which input woke it is not yet isolated.** The observation was a door opened *from inside*, which closes the jamb switch **and** pulls the latch and lock linkage — and on a Toyota the first pull of the inside handle unlocks the door. That is also a plausible alternative explanation for the silent alarm, since an inside-handle unlock is a legitimate disarm.
- **The test that settles it needs no electrical access:** tape the jamb switch plunger down (door physically open, car believes it closed), fob-lock, wait for deep sleep, then release the plunger. The latch never moves, no handle is touched, and releasing the plunger is electrically identical to what the relay will do. If the ECU sleeps with the plunger held, the courtesy switch is also confirmed as the input that governs sleep. Run this before splicing anything into the A-pillar.

A keepalive remains theoretically possible — injecting NM frames *during* the responsive window to extend it — but it cannot help after deep sleep, `0x45A` collides with the real gateway while it is awake, and holding the network up drains a 45 Ah hybrid aux battery. Not recommended.


## Calibration

Thresholds live in [car/src/tap/tap_config.h](car/src/tap/tap_config.h) and the spectral template in [car/src/tap/tap_template.h](car/src/tap/tap_template.h). They were tuned on the ESP32 with this sample scaling (32-bit I2S words, `kRawToFloat` in [tap_listener.cpp](car/src/tap_listener.cpp)), and they depend on the mic gain and the car window. Every detected event is logged as an `EVT` line with its features and the reason it was rejected, if any. Tune from recordings following [../tapDetector/TAP_TRAINING_PROCESS.md](../tapDetector/TAP_TRAINING_PROCESS.md), then copy the updated sources here.

## Safety

**This firmware trades security for simplicity.** The car unlocks on any advert whose address resolves with the fob IRK. Nothing proves that the fob is the one sending it, or that it is actually near the car:

- **Replay**: anyone who records one of the fob's adverts can replay it to unlock the car, for as long as that RPA is still current (60 s with the test setting of `CONFIG_BT_RPA_TIMEOUT`, 15 min with Zephyr's default).
- **Relay**: a pair of radios that forwards the fob's adverts from your pocket to the car unlocks it at any distance, with no time limit.

The `nrf52840/main` car firmware stops replay with an HMAC challenge-response over an encrypted link. Stopping a relay attack needs the planned UWB distance check on either firmware.

A tap only acts while the fob is detected. The CAN frames are verified only on the 2018 Lexus CT200h; check them before using this on another vehicle.
