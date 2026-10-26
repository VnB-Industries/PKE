# PKE

Full Passive Entry System for a 2018 Lexus CT200h, on two Seeed XIAO nRF52840 (Plus) boards running nRF Connect SDK (Zephyr), board target `xiao_ble/nrf52840`.

- **Fob**: advertises with a rotating Resolvable Private Address and answers a challenge-response over BLE.
- **Car unit**: resolves the fob, authenticates it, then unlocks and locks the doors over CAN (MCP2515 on the OBD port). It wakes a body ECU in deep sleep by pulsing a relay on the driver's door jamb switch.
- **Window taps** (INMP441 mic on the glass, car unit) give manual control: double tap = unlock, triple tap = lock. **The mic only runs while an authenticated fob is connected**, so a tap on its own can never open the car.

The car unit is also ported to an ESP32 DevKit on the `ESP32/main` branch, with proximity-only authentication and the same fob. The earlier ESP32 Arduino firmware (tap-only, no BLE) is on the `ESP32/old_tapdetect` branch. The BLE/UWB groundwork came from the BLEUWB project.

| Directory | Contents |
| --- | --- |
| [car/](car/) | car unit: BLE central, CAN door frames, relay wake, tap listener |
| [fob/](fob/) | fob: BLE peripheral with RPA, challenge-response, motion sleep |
| [common/](common/) | GATT protocol ([pke_protocol.h](common/pke_protocol.h)) and PSA crypto on CryptoCell-310 ([pke_crypto.c](common/pke_crypto.c)) |
| [scripts/](scripts/) | key generation and build/flash |
| [tests/](tests/) | on-car test protocols |

## Hardware (car unit)

| XIAO pin | Connects to |
| --- | --- |
| D0 | relay coil driver (transistor + flyback diode, or opto-isolated module) |
| D1 | MCP2515 CS |
| D2 | MCP2515 INT (required by the Zephyr driver) |
| D3 | INMP441 SCK |
| D4 | INMP441 WS |
| D5 | INMP441 SD |
| D6 | wake input, optional: push button to GND, and/or the driver's jamb-switch line through a diode (cathode to the line) |
| D7 | free |
| D8 / D9 / D10 | MCP2515 SCK / MISO / MOSI |
| 3V3 / GND | INMP441 (L/R → GND). GND also goes to the MCP2515 module. The module needs its own 3.3 V supply: see below. |

The pins are set in [car/app.overlay](car/app.overlay). Set `osc-freq` there to match the crystal on your MCP2515 module (8 MHz by default). A wrong value still starts, but frames won't ACK. The CAN bitrate is 500 kbit/s. The UWB module is not fitted yet.

**Important notes before assembly:**

- **Don't power the CAN module from the XIAO's 3V3 pin**: the XIAO's regulator holds 3.3 V at rest, but it can't deliver the SN65HVD230's current bursts when it transmits. Measured on the car: from the XIAO's 3V3, every transmitted frame failed with bit errors (`TXB0CTRL 18`) and the MCP2515 went bus-off, while receiving still worked. The same module powered from another 3.3 V supply transmitted fine with the same XIAO and firmware. The weak supply also left the MCP2515 unconfigured at boot (`CNF 00 00 00`, `CANINTE 00`), which the firmware now detects and repairs (`mcp_restore_config()` in [car/src/door.c](car/src/door.c)). Feed the module from its own 3.3 V regulator (e.g. an MCP1700 or AMS1117-3.3), with its GND tied to the XIAO's GND, and put 47–100 µF plus 100 nF across VCC/GND at the module.
- **The MOSI line back-powers the module**: with the module's VCC disconnected, its PWR LED still lights, fed from MOSI through the MCP2515's input protection diode. Don't run it that way: check that VCC is really connected.
- **Leave the module's Rs as it is**: the CAN module isn't power-managed on this branch, so Rs stays tied to GND (high-speed mode), as most SN65HVD230 modules ship.
- **No UWB on this wiring**: D0–D10 are all taken. UWB will come with a dedicated carrier PCB for the XIAO and the UWB3000F00 module, which has its own pinout, so the overlay will change then. Until that board is made, don't wire a DW3000 to this unit.

## Hardware (fob)

| Part | Role |
| --- | --- |
| Seeed XIAO nRF52840 (Plus) | BLE, crypto, and the onboard LiPo charger |
| AFTERTECH 402030 LiPo (~190 mAh usable) | power, on the XIAO's BAT+ / BAT− pads, charged over USB-C |
| ADXL362 accelerometer | motion switch: keeps the fob awake while it is carried, wakes it from System OFF |
| UWB3000F00 (DW3000) | distance check; **not fitted yet** |

| XIAO pin | Connects to |
| --- | --- |
| D1 | ADXL362 CS |
| D2 | ADXL362 INT1 (wakes the nRF52840 from System OFF) |
| D8 / D9 / D10 | ADXL362 SCLK / MISO / MOSI |
| 3V3 / GND | ADXL362 VS and VIO |

The XIAO also wires its charger to internal pins, which the firmware reads (from Seeed's schematic, not the Zephyr board files):

| nRF52840 pin | Onboard connection |
| --- | --- |
| P0.17 | BQ25101 CHG, low while charging |
| P0.14 | low switches on the 1M/510k divider from BAT+ |
| P0.31 (AIN7) | battery voltage through that divider |

The pins are set in [fob/app.overlay](fob/app.overlay). The ADXL362 must stay powered from 3V3 when the fob is in System OFF: it is the only thing that can wake it. Leave INT1 on a pin with no pull-up or LED; the line idles low.

The fob has two power states:

- **Carried**: the fob advertises every ~250 ms and answers the car. The DW3000 will be powered only during a car session, for the 3–5 ranging rounds, and kept off otherwise.
- **At rest**: after 1 minute without movement, the fob stops advertising and enters System OFF. Only the ADXL362 (~270 nA) and the nRF52840's wake logic stay powered. The next movement wakes it, and it boots and advertises again. With a car connected, the fob waits twice as long at rest, then closes the link itself before powering off.

Details are under [Power](#power).

When the UWB3000F00 is fitted, it shares the SPI bus (D8/D9/D10) and needs its own CS, IRQ and RST pins. Feed it from the regulated 3V3 pin, never from BAT+: a charged cell (~4.2 V) is above its 3.6 V maximum.

The onboard RGB LED is used only at boot: a ~0.7 s red-green-blue chase then a white flash ([common/boot_led.c](common/boot_led.c)), on both boards. Each wake from System OFF is a reboot, so it also marks every wake. The LED stays off otherwise, to keep consumption down. Everything else is reported on the USB serial log.

## Build & flash

1. Generate the enrollment keys once: `python3 scripts/gen_keys.py`. This writes `common/keys.h`, which is gitignored. Both boards must be built from the same file.
2. Build and flash each board, from any shell:
   ```sh
   scripts/flash.sh fob      # add -p for a pristine build
   scripts/flash.sh car
   ```
   When asked, double-tap reset on that board only, to enter the UF2 bootloader. The script loads the nRF Connect toolchain itself when `west` is not on the PATH (override with `NCS` and `NCS_TOOLCHAIN`), builds into `build/<app>`, then runs `west flash`.
3. Read the logs on each board's USB serial port, at 115200 baud.

`west flash` uses the `uf2` runner set in each app's `CMakeLists.txt`. The Flash button of the nRF Connect VS Code extension does not work with it: the extension always passes a device ID, which the `uf2` runner rejects.

## How it works

1. **Enrollment**: `scripts/gen_keys.py` creates the shared secret, the fob IRK and the fob static identity address. They are compiled into both firmwares.
2. **Discovery**: the fob's RPA rotates every 60 s (`CONFIG_BT_RPA_TIMEOUT`, short for testing). The car resolves it on the host with AES-ECB. It connects once the filtered advert RSSI holds -70 dBm for 3 samples.
3. **Authenticated exchange**: the link is encrypted with LE Secure Connections Just Works, without bonding. On top of it, both sides prove they hold the secret with HMAC-SHA256 over two fresh nonces, then derive a session key with HKDF-SHA256. Both log the same `key id`, a hash prefix of the key, never the key itself.
4. **Mic on**: the car starts the tap listener. It stops again when the session ends, whether by a lock or a lost link. The MCP2515 stays on the bus.
5. **UWB ranging**: dummy for now, and it always passes ([ranging.c](car/src/ranging.c)).
6. **Unlock**: off for now (`AUTO_UNLOCK_ENABLED` 0 in [car/src/main.c](car/src/main.c)): the session skips this step, and the doors open with a double tap or the original key. When enabled, the car sends the unlock frame ([door.c](car/src/door.c)), with the relay wake if the bus is asleep. The session stays up even if delivery failed; a double tap retries it. While the car is being driven there is no automatic unlock, so a fob reconnecting mid-drive never opens the doors (see [Driving mode](#driving-mode)).
7. **Exit detection**: the car polls the link RSSI every 250 ms. An exit starts when the filtered RSSI stays below -80 dBm for 5 samples. It only counts while the car is not being driven: the car polls the vehicle state over OBD-II once a second ([vehicle.c](car/src/vehicle.c)), and while driving it ignores the RSSI and slows the link (see [Driving mode](#driving-mode)).
8. **UWB ranging session**: `RANGE_REQ` with a fresh nonce; both sides derive the STS key. The DS-TWR rounds are the dummy and pass.
9. **Lock**: once `LOCK_K_ROUNDS` rounds are beyond the lock radius, the car sends the lock frame. A lost link (supervision timeout) counts as beyond the radius. A link the fob closes itself (`0x13`, remote user terminated, sent when it goes to sleep at rest) does not lock: the doors stay as they are. The car then disconnects and scans again after 2 s; the 10 dB gap between the connect and exit thresholds stops it from flapping.

BLE thresholds are `#define`s at the top of [car/src/main.c](car/src/main.c). Door frames, retries and relay timings are in [car/src/door_config.h](car/src/door_config.h).

### Tap gestures (authenticated fob only)

| Gesture | Action |
| --- | --- |
| 2 taps, 150–600 ms apart | unlock |
| 3 taps, 150–700 ms between each, gaps within ±30% of each other | lock |

A gesture is only matched after 1.2 s of silence following its last tap. That is what lets a third tap turn an unlock into a lock, and it makes a fourth tap cancel the gesture. The cost is a 1.2 s delay before either command.

The detector in [car/src/tap/](car/src/tap/) is a copy of [../tapDetector/src](../tapDetector/src): an onset detector on every sample, temporal and FFT validation of each event, then timing pattern matching. [tap_listener.cpp](car/src/tap_listener.cpp) runs it in two threads. Between sessions the I2S peripheral is stopped and both threads block, so the mic is unclocked and draws nothing.

### Power

The car unit is built to idle at the lowest draw the radio allows. The onboard LED lights only for the ~0.7 s boot animation. The I2S clock runs only during an authenticated session. CAN reception is interrupt-driven, and the relay dwell sleeps on a semaphore instead of polling.

**The CAN module isn't power-managed on this branch** (decided on 26/10): no MCP2515 Sleep, no SN65HVD230 standby, no supply switching. The controller goes on the bus at boot and stays there, in normal mode, ACKing and receiving every frame, so `bus_awake()` always reflects the bus. The module draws its full current all the time (~10 mA for the transceiver, datasheet value), which the System OFF stage below does not remove. Power management of the module is done on `nrf52840/UWB` only, where the carte_uwb board switches the module's supply.

Long parking is handled in three stages, counted from the end of the last fob session ([car/src/power.h](car/src/power.h)):

| Time without a session | Car unit | Fob found |
| --- | --- | --- |
| under 2 days | normal scan, 50% radio duty | within ~1–2 s |
| 2 to 4 days | deep idle: 1 s of scanning every 60 s (~1.7% duty) | within ~60 s |
| over 4 days | System OFF: radio and CPU stopped | only after the wake input |

In deep idle, hearing the fob at any RSSI returns the unit to normal scanning, and so does a press of the wake input. In System OFF, the wake input reboots the unit, which then scans normally. The idle clock restarts at every boot, so after a reflash or a replug the stages start from zero.

The wake input is D6, active low with the internal pull-up. Two sources can share it:
- A push button to GND, hidden somewhere reachable from inside the car.
- The driver's jamb-switch line, through a diode with its cathode on the line (a Schottky such as a BAT54). Opening the door grounds the line, which pulls D6 low. When the door is closed, the diode blocks the line's battery voltage. After System OFF, the driver opens the car once with the original key, and PKE works again from the next approach.

If D6 is left unconnected, the pull-up holds it inactive. The unit never wakes on its own, so after 4 days it stays off until it is unplugged and plugged back in. If the wake input fails to set up at boot, the unit logs an error and stops at deep idle instead.

System OFF stops only the nRF52840. The CAN module stays powered and on the bus, so it and the 12 V regulator's quiescent current set the unit's draw from then on.

The fob sleeps when nobody carries it. The ADXL362 runs as a motion switch in wake-up mode (~270 nA), with INT1 high while the fob moves. After 60 s without an INT1 edge (`MOTION_IDLE_TIMEOUT_S` in [fob/src/motion.h](fob/src/motion.h)), the fob stops advertising and enters System OFF. It never sleeps while INT1 is high. With a car connected it waits twice as long at rest, then closes the link itself (`0x13`, which the car does not treat as an exit) before sleeping. While the car reports it is being driven, it does not sleep at all. The next movement wakes it through INT1, and it reboots and starts advertising again. If the ADXL362 doesn't answer at boot, the fob logs an error and never sleeps, since nothing could wake it.

The chip decides on its own when the fob is at rest ([fob/src/motion.c](fob/src/motion.c)). Movement is more than 250 mg away from its reference on any axis (`ACT_THRESHOLD_MG`). Rest is all axes within 150 mg of the reference (`INACT_THRESHOLD_MG`) for 30 consecutive samples (`INACT_SAMPLES`), ~5 s at the ~6 Hz wake-up rate. A single sample beyond 150 mg restarts that count. Each INT1 edge is logged:

```
<inf> motion: ADXL362 INT1: moving
<inf> motion: ADXL362 INT1: at rest
```

INT1 normally drops 5–6 s after the fob is put down.

**Known issue, under investigation:** INT1 sometimes stays high for minutes while the fob lies still: 1 Hz reads vary by less than 150 mg, yet no "at rest" edge comes. The fob then never sleeps, since `motion_active()` keeps deferring the idle timer. The suspected cause is noise between the 1 Hz reads, with single samples beyond 150 mg that keep restarting the inactivity count. The fob firmware carries a temporary debug dump for this: while INT1 is high, it reads XYZ every 50 ms, which catches every chip sample, and logs each axis's min..max and span once a second:

```
<inf> motion: x 1052..1191 (139) y -25..-2 (23) z -104..-45 (59) mg
```

A span regularly above 150 mg at rest points to noise: raise `INACT_THRESHOLD_MG` (~250–300 mg) or enable the ADXL362 low-noise mode. Spans well under 150 mg point to the chip: add a software fallback that resets the ADXL362 when the idle timer expires with INT1 still high. Remove the dump (`dump_work` in motion.c) once this is settled: it keeps SPI busy every 50 ms while the fob moves.

At every boot, including each wake from System OFF, the fob logs the battery once ([fob/src/battery.c](fob/src/battery.c)):

```
<inf> battery: battery 3987 mV, charging
```

"not charging" means either full or no USB power; the voltage tells them apart (~4.2 V on USB means full). The divider is switched on only for the reading. With no cell connected, the voltage is meaningless.

The BQ25101 charges the cell from USB-C on its own, so charging continues while the nRF52840 is in System OFF. The firmware leaves P0.13 (charge current: low 100 mA, high 50 mA) unconfigured, so the board's default applies.

Fob battery budget: chip sleep is a few µA, BLE bursts are a few mA, and UWB ranging (once fitted) averages ~21 mA with 72 mA peaks. Battery life therefore depends on keeping UWB asleep except for brief ranging bursts. A CR2032 needs a 100–470 µF buffer cap near the UWB module. A LiPo must feed the UWB module from the regulated 3.3 V pin: a charged cell (~4.2 V) is above the UWB3000F00's 3.6 V maximum.

## Driving mode

During a session the car reads the vehicle state over OBD-II once a second ([car/src/vehicle.c](car/src/vehicle.c), requests in [car/src/vehicle_config.h](car/src/vehicle_config.h)):

| State | Condition |
| --- | --- |
| off | no ECU answers, or the bus is quiet (nothing is transmitted then) |
| driving | speed > 0, or in gear once the gear check is enabled; without it, speed 0 counts as driving for 10 s after the last motion, so a red light does not flip it |
| parked | otherwise |

A query only goes out when the bus has carried traffic in the last 2 s, so it never wakes a sleeping car. Three missed replies in a row are needed before a driven car counts as off.

While driving:

1. The car sends `DRIVE 1` to the fob while the link is still fast. The fob then stops its rest timer.
2. The car switches the link to a 1 s interval with peripheral latency 4 and a 16 s supervision timeout. The fob's radio wakes every 5 s.
3. The car ignores the link RSSI, so no exit can start.

When the car stops being driven, the car restores the default link (30–50 ms) and then sends `DRIVE 0`. Exit detection resumes once both are done.

Estimated fob current for the chip alone (board leakage comes on top): ~200 µA on the default link, ~5 µA on the driving link.

### Signals

- **Speed**: standard OBD-II, request `7DF 02 01 0D`, reply on `7E8`–`7EF` as `03 41 0D <km/h>`.
- **Gear**: Toyota hybrid ECU, service `0x21` on `7E2`, reply on `7EA`. The PID, byte and P value for the CT200h are not verified, so `VEHICLE_GEAR_ENABLED` is 0.

Verify both with [obd-tool](../obd-tool) `tx`, with the car in READY:

1. Send `tx 7DF 02 01 0D 00 00 00 00 00` while stationary and while rolling. Expect `03 41 0D <km/h>` from `7E8`.
2. Send `tx 7E2 02 21 <pid> 00 00 00 00 00` in P, R, N, D and B. Find the byte that changes with the shift position and its value in P.
3. Fill in `GEAR_PID`, `GEAR_BYTE` and `GEAR_P_VALUE`, then set `VEHICLE_GEAR_ENABLED` to 1. Only a single-frame reply (first byte `0x0N`) is decoded. A multi-frame reply (first byte `0x1N`) needs ISO-TP flow control, which is not implemented.

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

So the CAN wake was not ported to the nRF52840 firmware. The ESP32 version, with `WAKE_BUS_ENABLED`, is still on the `ESP32/old_tapdetect` branch for platforms where it does work.

### What the error flags mean

`EFLG=0x15` is EWARN + TXWAR + TXEP: error-passive, **not** bus-off (that would be bit 5, `0x20`). `TEC` pins at exactly 128 and never climbs, because an error-passive transmitter that sees no dominant bit during its passive error flag doesn't increment TEC. The MCP2515 therefore can't reach the 256 bus-off limit this way — the earlier worry about bus-off was unfounded. On the nRF52840 a door frame that isn't ACKed within `DOOR_TX_TIMEOUT_MS` is aborted with `can_stop()`/`can_start()`, so it can't stay queued and fire later when the ECU wakes.

### Current behaviour

The firmware receives every frame (interrupt-driven, no polling), so it knows whether the bus is alive without transmitting. On a gesture it tries the door frame, and if nothing answers it stops rather than retrying into a dead bus. The log distinguishes a silent bus from a frame refused on an active bus (which would point at bitrate or wiring instead).

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

A single wire at battery voltage: the body ECU holds one state through an internal pull-up, and the switch's own metal body grounds it to the jamb in the other. A relay wired across that switch and pulsed makes the ECU see a door opening. Wiring is under [Relay wake is integrated into the firmware](#relay-wake-is-integrated-into-the-firmware), and the on-car checks are in [tests/reveil-ecu-relais.md](tests/reveil-ecu-relais.md); the pulse is implemented as the `wake` command in [../obd-tool](../obd-tool/README.md).

Pulse a train, not once — Toyota's service procedure for a sleeping main body ECU is *open the driver door, then open and close a door several times at ~1.5 s intervals*.

### Relay wake is integrated into the firmware

The relay pulse is built into PKE itself (`RELAY_WAKE_ENABLED` in [car/src/door_config.h](car/src/door_config.h), on by default). On an **unlock** (fob or double tap), if the bus reads as asleep, `door_send()` calls `relay_wake()`: it pulses the relay output (D0) as a train of `RELAY_PULSE_COUNT` open/close cycles (`RELAY_PULSE_ON_MS` / `RELAY_PULSE_OFF_MS`, ~1.5 s each), watching CAN RX throughout, and sends the door frame the moment the bus comes alive. Lock is unaffected — it fires in the responsive window, where the bus is already awake and this path isn't taken.

Wire the relay **in parallel** with the driver's jamb switch, one contact on the switch line and one on a good chassis ground, with a common ground to the XIAO. Drive the coil through a transistor and a flyback diode (or use an opto-isolated relay module), contacts dry — switch the line, don't inject voltage into it. With no relay wired, D0 just toggles harmlessly.

**Validate on the car before relying on it** with the taped-plunger test, then the relay test — see [tests/reveil-ecu-relais.md](tests/reveil-ecu-relais.md).

What is established, what isn't, and the test that closes the gap:

- **A door opening does wake it**, with the car fob-locked, and **no alarm sounded** — so the theft deterrent, which reads this same switch, does not veto the approach. That was the risk that would have killed it.
- **Which input woke it is not yet isolated.** The observation was a door opened *from inside*, which closes the jamb switch **and** pulls the latch and lock linkage — and on a Toyota the first pull of the inside handle unlocks the door. That is also a plausible alternative explanation for the silent alarm, since an inside-handle unlock is a legitimate disarm.
- **The test that settles it needs no electrical access:** tape the jamb switch plunger down (door physically open, car believes it closed), fob-lock, wait for deep sleep, then release the plunger. The latch never moves, no handle is touched, and releasing the plunger is electrically identical to what the relay will do. If the ECU sleeps with the plunger held, the courtesy switch is also confirmed as the input that governs sleep. Run this before splicing anything into the A-pillar.

A keepalive remains theoretically possible — injecting NM frames *during* the responsive window to extend it — but it cannot help after deep sleep, `0x45A` collides with the real gateway while it is awake, and holding the network up drains a 45 Ah hybrid aux battery. Not recommended.

## Calibration

Thresholds live in [car/src/tap/tap_config.h](car/src/tap/tap_config.h) and the spectral template in [car/src/tap/tap_template.h](car/src/tap/tap_template.h). Both were tuned on the ESP32, and they depend on the mic gain, the sample scaling and the car window. Every detected event is logged as an `EVT` line with its features and the reason it was rejected, if any. Tune from recordings following [../tapDetector/TAP_TRAINING_PROCESS.md](../tapDetector/TAP_TRAINING_PROCESS.md), then copy the updated sources here.

**Check the mic on the nRF52840 first.** Its I2S peripheral stops at 24-bit samples, so it clocks 48 SCK cycles per frame where the INMP441 datasheet expects 64. Samples are scaled as 24-bit sign-extended values (`kRawToFloat` in [tap_listener.cpp](car/src/tap_listener.cpp)). With a fob connected, check that `floor` and `peak` in the `EVT` lines are in the same range as on the ESP32 before trusting the thresholds.

## Safety

A tap only acts while an authenticated fob is connected. The CAN frames are verified only on the 2018 Lexus CT200h; check them before using this on another vehicle.
