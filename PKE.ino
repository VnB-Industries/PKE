/*
  PKE - ESP32 + INMP441 microphone + MCP2515 CAN controller

  Double tap on the window -> unlock the car
  Triple tap on the window -> lock the car

  Tap detection comes from tapDetector (src/, copied from ../tapDetector/src):
  per-sample onset detector in an audio task, temporal + FFT validation of each
  event, then a timing pattern matcher over the whole tap sequence.
  CAN door frames come from obd-tool (raw 11-bit frame on ID 0x750).

  Wiring: see config.h and ../obd-tool/README.md (MCP2515) and
  ../tapDetector/README.md (INMP441).

  Libraries: MCP_CAN_lib (coryjfowler).
*/

#include <SPI.h>
#include <mcp_can.h>
#include <driver/i2s.h>

#include "config.h"
#include "src/detector.h"
#include "src/tap_features.h"
#include "src/pattern.h"

constexpr size_t kDmaBlock = 256;       // samples per I2S read (16 ms)
constexpr float kRawToFloat = 1.0f / 2147483648.0f;  // 24-bit data, left-justified in 32 bits
constexpr uint8_t kEventQueueLength = 3;

// ---------------------------------------------------------------------------
// PATTERNS -> ACTIONS
// ---------------------------------------------------------------------------
// A sequence is only matched after this much silence, so a third tap can still turn a double tap
// into a triple tap. Both gestures therefore fire this long after their last tap.
constexpr uint32_t kSequenceEndMs = 1200;

const tap::TapPattern kPatterns[] = {
  // name           taps  relative gaps  tolerance  gap min-max (ms)
  {"double tap",    2,    {1},           0.30f,     150, 600},
  {"triple tap",    3,    {1, 1},        0.30f,     150, 700},
};
const uint8_t* const kPatternActions[] = { UNLOCK_CMD_DATA, LOCK_CMD_DATA };  // same order as kPatterns

// ---------------------------------------------------------------------------
// TAP DETECTION STATE
// ---------------------------------------------------------------------------
volatile uint32_t audioNowMs = 0;
volatile uint32_t droppedEvents = 0;

QueueHandle_t eventQueue;
tap::Detector detector;
tap::ValidatorConfig validatorConfig;
tap::PatternMatcher matcher(kPatterns, sizeof(kPatterns) / sizeof(kPatterns[0]), kSequenceEndMs);

// ---------------------------------------------------------------------------
// CAN
// ---------------------------------------------------------------------------
MCP_CAN CAN0(MCP_CS_PIN);
bool canReady = false;
uint32_t lastInitAttempt = 0;

bool initCan() {
  DEBUG_PRINTLN(F("[can] initializing MCP2515..."));
  if (CAN0.begin(MCP_ANY, MCP_CAN_SPEED, MCP_OSC_FREQ) != CAN_OK) {
    DEBUG_PRINTLN(F("[can] MCP2515 init failed"));
    return false;
  }
  CAN0.setMode(MCP_NORMAL);
  DEBUG_PRINTLN(F("[can] MCP2515 init OK, on-bus"));
  return true;
}

void logCanError(const char* what, byte result) {
  // result codes: 6 = CAN_GETTXBFTIMEOUT, 7 = CAN_SENDMSGTIMEOUT (no ACK within the timeout)
  // EFLG: bit0 EWARN, bit2 TXWAR, bit4 TXEP (error-passive), bit5 TXBO (bus-off)
  DEBUG_PRINTF("[can] %s failed: code=%u EFLG=0x%02X TEC=%u REC=%u\n",
               what, result, CAN0.getError(), CAN0.errorCountTX(), CAN0.errorCountRX());
}

// Last frame received from any other node. Polled in the main loop, so the bus state is known
// without having to transmit and fail.
uint32_t lastRxMs = 0;
bool lastSendOk = false;

void pollCanRx() {
  while (CAN0.checkReceive() == CAN_MSGAVAIL) {
    uint32_t id;
    uint8_t len, buf[8];
    if (CAN0.readMsgBuf(&id, &len, buf) != CAN_OK) break;
    lastRxMs = millis();
  }
}

bool busLooksAwake() {
  return lastRxMs != 0 && (millis() - lastRxMs) < BUS_AWAKE_TIMEOUT_MS;
}

// Bus-off, or an error counter stuck at the error-passive ceiling, means the last attempts all went
// unacknowledged. begin() resets the chip, which clears both error counters.
void recoverCanIfNeeded() {
  uint8_t eflg = CAN0.getError();
  uint8_t tec = CAN0.errorCountTX();
  if ((eflg & MCP_EFLG_TXBO) || tec >= TEC_REINIT_LIMIT) {
    DEBUG_PRINTF("[can] EFLG=0x%02X TEC=%u -> resetting controller\n", eflg, tec);
    canReady = initCan();
  }
}

// Any frame from another node proves the bus is awake and can ACK ours.
bool busTrafficWithin(uint32_t windowMs) {
  uint32_t start = millis();
  while (millis() - start < windowMs) {
    if (CAN0.checkReceive() == CAN_MSGAVAIL) {
      uint32_t id;
      uint8_t len, buf[8];
      CAN0.readMsgBuf(&id, &len, buf);
      lastRxMs = millis();
      DEBUG_PRINTF("[can] bus awake (traffic on 0x%03X)\n", (unsigned)id);
      return true;
    }
  }
  return false;
}

// Sends harmless frames to generate the bus activity that brings the transceivers and ECUs out of
// standby, listening after each one for the bus to come alive. One-shot TX is enabled for these so
// a failed attempt isn't retried in hardware, which keeps the error counter from running to bus-off.
bool wakeBus() {
#if !WAKE_BUS_ENABLED
  DEBUG_PRINTLN(F("[can] bus asleep; CAN wake disabled"));
  return false;
#else
  DEBUG_PRINTLN(F("[can] bus appears asleep, sending wake frames..."));
  CAN0.enOneShotTX();
  bool awake = false;

  for (uint8_t i = 0; i < WAKE_MAX_BURSTS && !awake; i++) {
    CAN0.sendMsgBuf(WAKE_CMD_ID, 0, 8, const_cast<uint8_t*>(WAKE_CMD_DATA));
    awake = busTrafficWithin(WAKE_LISTEN_MS);
    if (!awake && (CAN0.getError() & MCP_EFLG_TXBO)) {
      DEBUG_PRINTLN(F("[can] went bus-off while waking, resetting"));
      if (!initCan()) return false;
      CAN0.enOneShotTX();
    }
  }

  CAN0.disOneShotTX();
  if (!awake) DEBUG_PRINTLN(F("[can] no traffic seen; bus may still be asleep"));
  return awake;
#endif
}

// Waits briefly for the body ECU's diagnostic reply. A send that was ACKed only proves some node
// heard the frame; a positive response (service 0x30 + 0x40 = 0x70, echoing LID 0x11) proves the
// body ECU actually processed it.
bool awaitDoorReply(uint32_t windowMs) {
  uint32_t start = millis();
  while (millis() - start < windowMs) {
    if (CAN0.checkReceive() != CAN_MSGAVAIL) continue;
    uint32_t id;
    uint8_t len, buf[8];
    if (CAN0.readMsgBuf(&id, &len, buf) != CAN_OK) break;
    lastRxMs = millis();
    if (id == DOOR_RESPONSE_ID && len >= 4 && buf[2] == 0x70) {
      DEBUG_PRINTF("[door] body ECU confirmed (758: %02X %02X %02X %02X)\n",
                   buf[0], buf[1], buf[2], buf[3]);
      return true;
    }
  }
  return false;
}

// Waits up to ms while draining RX, returning early the moment the bus comes alive.
bool dwellAndPoll(uint32_t ms) {
  uint32_t until = millis() + ms;
  while ((int32_t)(until - millis()) > 0) {
    pollCanRx();
    if (busLooksAwake()) return true;
  }
  return false;
}

#if RELAY_WAKE_ENABLED
// Drives the relay level. With RELAY_ACTIVE_HIGH, energising the coil closes the relay and grounds
// the jamb-switch line, which the body ECU reads as "driver's door open".
void relaySet(bool energised) {
  digitalWrite(RELAY_PIN, (energised == RELAY_ACTIVE_HIGH) ? HIGH : LOW);
}

// Wakes a body ECU in deep sleep by pulsing the relay across the driver's jamb switch as a train of
// open/close cycles - the way Toyota's service procedure wakes a sleeping main body ECU (open the
// door, then open and close several times at ~1.5 s intervals). A single pulse isn't enough. Keeps
// draining RX throughout and returns as soon as the bus comes alive, so a wake short-circuits the
// rest of the train.
bool pulseRelayWake() {
  DEBUG_PRINTLN(F("[wake] pulsing relay on the jamb-switch line..."));
  for (uint8_t i = 0; i < RELAY_PULSE_COUNT; i++) {
    relaySet(true);                              // "door open" - ground the line
    bool awake = dwellAndPoll(RELAY_PULSE_ON_MS);
    relaySet(false);                             // "door closed" - release to the pull-up
    if (awake || dwellAndPoll(RELAY_PULSE_OFF_MS)) {
      DEBUG_PRINTF("[wake] bus came alive after %u pulse(s)\n", i + 1);
      return true;
    }
  }
  // Train done: give the gateway/ECU a last moment to bring the bus up before giving up.
  bool awake = dwellAndPoll(RELAY_WAKE_TIMEOUT_MS);
  DEBUG_PRINTLN(awake ? F("[wake] bus came alive after the pulse train")
                      : F("[wake] no traffic after relay pulses; ECU may still be asleep"));
  return awake;
}
#endif

// Sends the door command, retrying a few times.
//
// Retrying matters even when the bus reads silent: measured on the CT200h, a door frame sent after
// 42 s of silence was both ACKed and positively answered. A quiet bus is not necessarily a sleeping
// one - as long as this node stays in normal mode and ACKs the gateway's NM frames, those quiet
// stretches are gaps in the NM cycle, not deep sleep. So always try, and never give up on the first
// failure. Lock and unlock are idempotent, so a repeat is harmless.
void sendDoorFrame(const uint8_t* data) {
  lastSendOk = false;
  if (!canReady) {
    DEBUG_PRINTLN(F("[door] CAN not ready, command dropped"));
    return;
  }

  recoverCanIfNeeded();
  if (!canReady) return;

  // Don't launch a door frame into a bus in deep sleep. With auto-retransmit on (needed so a send
  // survives NM-cycle gaps on an awake bus), an unacknowledged frame stays in the MCP2515 TX buffer
  // and fires the instant the ECU wakes - a stale lock/unlock on wake. abortTX() can't clear it
  // cleanly here: this lib leaves the CANCTRL ABAT bit set with no way to clear it, which would
  // abort every later send too. So gate on bus state. The NM frame ticks ~1 Hz in the responsive
  // window, so busLooksAwake() stays true there and the quiet-stretch sends above still happen.
  if (!busLooksAwake()) {
#if RELAY_WAKE_ENABLED
    // Only unlock-on-return has to defeat deep sleep; lock happens in the responsive window just
    // after walking away, where the bus is still awake and this branch isn't taken.
    if (data == UNLOCK_CMD_DATA && pulseRelayWake()) {
      DEBUG_PRINTLN(F("[door] bus woke after relay pulse, sending command"));
    } else {
      DEBUG_PRINTLN(F("[door] bus in deep sleep, command dropped (would fire on wake)"));
      return;
    }
#else
    DEBUG_PRINTLN(F("[door] bus in deep sleep, command dropped (would fire on wake)"));
    return;
#endif
  }

  for (uint8_t attempt = 1; attempt <= DOOR_MAX_RETRIES; attempt++) {
    byte result = CAN0.sendMsgBuf(DOOR_CMD_ID, 0, 8, const_cast<uint8_t*>(data));
    if (result == CAN_OK) {
      DEBUG_PRINTF("[door] sent OK (attempt %u)\n", attempt);
      lastSendOk = true;
      awaitDoorReply(DOOR_REPLY_WAIT_MS);
      return;
    }
    logCanError("door frame", result);

    if (attempt < DOOR_MAX_RETRIES) {
      wakeBus(); // no-op unless WAKE_BUS_ENABLED; the retry itself is what usually gets through
      recoverCanIfNeeded();
      if (!canReady) return;
      uint32_t until = millis() + DOOR_RETRY_GAP_MS;
      while ((int32_t)(until - millis()) > 0) pollCanRx();
    }
  }

  if (busLooksAwake()) {
    // traffic is flowing but our frame still wasn't ACKed: not a sleep problem
    DEBUG_PRINTLN(F("[door] NOT delivered although the bus is active - check bitrate/wiring"));
  } else {
    DEBUG_PRINTLN(F("[door] NOT delivered after retries: bus unresponsive"));
  }
  recoverCanIfNeeded(); // clear the error-passive state so the next gesture starts fresh
}

void handleNotReady() {
  uint32_t now = millis();
  if (now - lastInitAttempt >= MCP_RETRY_INTERVAL_MS) {
    lastInitAttempt = now;
    canReady = initCan();
  }
}

// ---------------------------------------------------------------------------
// LED: fast blink while CAN isn't ready, slow heartbeat once on-bus, solid 1s when a command was
// acknowledged, stutter 2s when it wasn't. Non-blocking so I2S reads aren't starved.
// ---------------------------------------------------------------------------
uint32_t lastLedToggle = 0;
uint32_t ledSolidUntil = 0;
uint32_t ledErrorUntil = 0;
bool ledOn = false;

void updateLed() {
  uint32_t now = millis();
  if ((int32_t)(ledErrorUntil - now) > 0) {
    if (now - lastLedToggle >= 80) {
      lastLedToggle = now;
      ledOn = !ledOn;
      digitalWrite(LED_PIN, ledOn ? HIGH : LOW);
    }
    return;
  }
  if ((int32_t)(ledSolidUntil - now) > 0) {
    digitalWrite(LED_PIN, HIGH);
    return;
  }
  uint32_t interval = canReady ? 1000 : 150;
  if (now - lastLedToggle >= interval) {
    lastLedToggle = now;
    ledOn = !ledOn;
    digitalWrite(LED_PIN, ledOn ? HIGH : LOW);
  }
}


// ---------------------------------------------------------------------------
// AUDIO TASK: reads I2S without interruption, runs only the cheap per-sample
// detector, and hands event snippets to loop(). Running on its own core, it
// keeps reading while loop() blocks on a CAN send.
// ---------------------------------------------------------------------------
void setupI2S() {
  i2s_config_t cfg = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = tap::kSampleRate,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = kDmaBlock,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };
  i2s_pin_config_t pins = {
    .bck_io_num = I2S_SCK_PIN,
    .ws_io_num = I2S_WS_PIN,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD_PIN
  };
  i2s_driver_install(I2S_PORT, &cfg, 0, NULL);
  i2s_set_pin(I2S_PORT, &pins);
  i2s_zero_dma_buffer(I2S_PORT);
}

void audioTask(void*) {
  static int32_t raw[kDmaBlock];
  static tap::TapEvent event;  // too big for the task stack

  for (;;) {
    size_t bytesRead = 0;
    i2s_read(I2S_PORT, raw, sizeof(raw), &bytesRead, portMAX_DELAY);
    size_t n = bytesRead / sizeof(int32_t);

    for (size_t i = 0; i < n; i++) {
      if (detector.process(raw[i] * kRawToFloat, event) && xQueueSend(eventQueue, &event, 0) != pdTRUE) {
        droppedEvents = droppedEvents + 1;
      }
    }
    audioNowMs = detector.nowMs();
  }
}

// ---------------------------------------------------------------------------
// EVENT HANDLING (loop task)
// ---------------------------------------------------------------------------
void handleEvent(const tap::TapEvent& ev) {
  tap::TapFeatures f;
  tap::computeFeatures(ev.samples, f);
  const char* reject = tap::checkFeatures(f, validatorConfig);
  static bool hasLastTap = false;
  static uint32_t lastTapMs = 0;
  uint32_t sinceTap = hasLastTap ? ev.onsetMs - lastTapMs : UINT32_MAX;
  if (!reject) reject = tap::checkContext(ev.preActivityDb, sinceTap, validatorConfig);

  // Every event is logged, accepted or not, to refine thresholds later.
  DEBUG_PRINTF("EVT t=%lu floor=%.1f peak=%.1f rise=%.2f decay=%.0f crest=%.1f pre=%.1f "
               "band=%.2f excl=%.2f cent=%.0f flat=%.2f sim=%.2f act=%.1f -> %s\n",
               (unsigned long)ev.onsetMs, ev.floorDb, f.peakDbfs, f.riseMs, f.decayMs, f.crestDb,
               f.preContrastDb, f.bandRatio, f.exclusionRatio, f.centroidHz, f.flatness,
               f.similarity, ev.preActivityDb, reject ? reject : "TAP");
  if (!reject) {
    hasLastTap = true;
    lastTapMs = ev.onsetMs;
    matcher.addTap(ev.onsetMs);
  }
}

void handleSequence(const tap::SequenceResult& seq) {
  if (!seq.match) {
    DEBUG_PRINTF("SEQ %u taps, no match, gaps(ms):", seq.tapCount);
    for (uint8_t i = 0; i + 1 < seq.tapCount && i < tap::kMaxTaps - 1; i++) {
      DEBUG_PRINTF(" %lu", (unsigned long)seq.gapsMs[i]);
    }
    DEBUG_PRINTLN();
    return;
  }

  const uint8_t* action = kPatternActions[seq.match - kPatterns];
  DEBUG_PRINTF("=== %s -> %s ===\n", seq.match->name, action == UNLOCK_CMD_DATA ? "UNLOCK" : "LOCK");
  sendDoorFrame(action);
  if (lastSendOk) ledSolidUntil = millis() + 1000;
  else            ledErrorUntil = millis() + 2000;
}

// ---------------------------------------------------------------------------
// SETUP / LOOP
// ---------------------------------------------------------------------------
void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(MCP_INT_PIN, INPUT);
#if RELAY_WAKE_ENABLED
  pinMode(RELAY_PIN, OUTPUT);
  relaySet(false); // start with the relay released ("door closed")
#endif
  Serial.begin(115200);
  delay(300);
  DEBUG_PRINTLN(F("=== PKE: double tap = unlock, triple tap = lock ==="));

  canReady = initCan();

  eventQueue = xQueueCreate(kEventQueueLength, sizeof(tap::TapEvent));
  setupI2S();
  // Core 0, high priority: Arduino loop() and its prints run on core 1.
  xTaskCreatePinnedToCore(audioTask, "audio", 4096, nullptr, configMAX_PRIORITIES - 2, nullptr, 0);
}

void loop() {
  static tap::TapEvent ev;  // too big for the loop stack
  static uint32_t reportedDrops = 0;

  if (!canReady) handleNotReady();
  else           pollCanRx(); // track bus activity so we know if it's awake without transmitting
  updateLed();

  if (xQueueReceive(eventQueue, &ev, pdMS_TO_TICKS(10)) == pdTRUE) {
    handleEvent(ev);
  }

  tap::SequenceResult seq;
  if (matcher.poll(audioNowMs, seq)) handleSequence(seq);

  if (droppedEvents != reportedDrops) {
    reportedDrops = droppedEvents;
    DEBUG_PRINTF("WARN %lu events dropped (queue full)\n", (unsigned long)reportedDrops);
  }
}
