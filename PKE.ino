/*
  PKE - ESP32 + INMP441 microphone + MCP2515 CAN controller

  Double tap on the window -> unlock the car
  Triple tap on the window -> lock the car

  Tap detection comes from tapDetector (RMS/peak trigger, FFT signature check,
  impact duration check, temporal pattern matching).
  CAN door frames come from obd-tool (raw 11-bit frame on ID 0x750).

  Wiring: see config.h and ../obd-tool/README.md (MCP2515) and
  ../tapDetector/tapDetector.ino (INMP441).

  Libraries: arduinoFFT v2.x (Enrique Condes), MCP_CAN_lib (coryjfowler).
*/

#include <SPI.h>
#include <mcp_can.h>
#include <driver/i2s.h>
#include <arduinoFFT.h>

#include "config.h"

// ---------------------------------------------------------------------------
// TAP DETECTION THRESHOLDS (calibrate with ENVELOPE_DEBUG, see tapDetector)
// ---------------------------------------------------------------------------
float RMS_THRESHOLD  = 800000.0f;
float PEAK_THRESHOLD = 1400000.0f;

#define BAND_LOW_HZ   200.0f
#define BAND_HIGH_HZ  1600.0f
float SPECTRAL_MIN_RATIO = 0.60f;

#define EXCLUSION_LOW_HZ   2000.0f
#define EXCLUSION_HIGH_HZ  6000.0f
float EXCLUSION_MAX_RATIO = 0.12f;

unsigned long MAX_IMPACT_DURATION_MS = 100;

float RMS_RELEASE_THRESHOLD  = RMS_THRESHOLD  * 0.4f;
float PEAK_RELEASE_THRESHOLD = PEAK_THRESHOLD * 0.6f;

unsigned long MIN_REFRACTORY_MS = 30;

bool ENVELOPE_DEBUG = false;
bool debugEnabled = true;

// ---------------------------------------------------------------------------
// PATTERNS -> ACTIONS
// ---------------------------------------------------------------------------
#define SILENT_END 1000 // a double tap is only confirmed after this much silence,
                        // so that a third tap can still turn it into a triple tap

enum DoorAction { ACTION_UNLOCK, ACTION_LOCK };

struct GapWindow { unsigned long minMs; unsigned long maxMs; };

const GapWindow PATTERN_DOUBLE[] = { { 150, 600 } };
const GapWindow PATTERN_TRIPLE[] = { { 150, 700 }, { 150, 700 } };

struct TocPattern {
  const char* name;
  const GapWindow* gaps;
  uint8_t gapCount;
  unsigned long trailingSilenceMs;
  DoorAction action;
};

#define MAKE_PATTERN(nameStr, arr, silenceMs, act) \
  { nameStr, arr, sizeof(arr) / sizeof(GapWindow), silenceMs, act }

TocPattern PATTERNS[] = {
  MAKE_PATTERN("double tap", PATTERN_DOUBLE, SILENT_END, ACTION_UNLOCK),
  MAKE_PATTERN("triple tap", PATTERN_TRIPLE, 0,          ACTION_LOCK),
};
const uint8_t PATTERN_COUNT = sizeof(PATTERNS) / sizeof(TocPattern);

struct PatternState {
  uint8_t step;
  unsigned long lastMs;
  bool pending;
};
PatternState patternStates[PATTERN_COUNT];

// ---------------------------------------------------------------------------
// BUFFERS (no dynamic allocation)
// ---------------------------------------------------------------------------
int32_t samples[BLOCK_SIZE];
double  fftReal[BLOCK_SIZE];
double  fftImag[BLOCK_SIZE];
ArduinoFFT<double> FFT = ArduinoFFT<double>(fftReal, fftImag, (uint_fast16_t)BLOCK_SIZE, (double)SAMPLE_RATE_HZ);

unsigned long lastTocMs = 0;
bool inImpact = false;
unsigned long impactStartMs = 0;
bool tocPendingValidation = false;

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
// I2S
// ---------------------------------------------------------------------------
void setupI2S() {
  i2s_config_t cfg = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE_HZ,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 256,
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

size_t readBlock() {
  size_t bytesRead = 0;
  i2s_read(I2S_PORT, (void*)samples, BLOCK_SIZE * sizeof(int32_t), &bytesRead, portMAX_DELAY);
  return bytesRead / sizeof(int32_t);
}

// ---------------------------------------------------------------------------
// SIGNAL ANALYSIS
// ---------------------------------------------------------------------------
void computeRmsPeak(size_t count, float &rms, float &peak) {
  double sumSq = 0.0;
  int32_t pk = 0;
  for (size_t i = 0; i < count; i++) {
    int32_t v = samples[i] >> SAMPLE_SHIFT;
    sumSq += (double)v * (double)v;
    int32_t av = v < 0 ? -v : v;
    if (av > pk) pk = av;
  }
  rms = (count > 0) ? sqrt(sumSq / count) : 0.0f;
  peak = (float)pk;
}

// FFT only runs once a peak has been detected (expensive)
void computeSpectralRatios(size_t count, float &bandRatio, float &exclusionRatio) {
  bandRatio = 0.0f;
  exclusionRatio = 0.0f;
  if (count < BLOCK_SIZE) return;

  for (uint16_t i = 0; i < BLOCK_SIZE; i++) {
    fftReal[i] = (double)(samples[i] >> SAMPLE_SHIFT);
    fftImag[i] = 0.0;
  }

  FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT.compute(FFTDirection::Forward);
  FFT.complexToMagnitude();

  double bandEnergy = 0.0, exclusionEnergy = 0.0, totalEnergy = 0.0;
  double freqPerBin = (double)SAMPLE_RATE_HZ / BLOCK_SIZE;

  for (uint16_t bin = 1; bin < BLOCK_SIZE / 2; bin++) {
    double freq = bin * freqPerBin;
    double energy = fftReal[bin] * fftReal[bin];
    totalEnergy += energy;
    if (freq >= BAND_LOW_HZ && freq < BAND_HIGH_HZ) bandEnergy += energy;
    if (freq >= EXCLUSION_LOW_HZ && freq < EXCLUSION_HIGH_HZ) exclusionEnergy += energy;
  }

  if (totalEnergy > 0.0) {
    bandRatio = (float)(bandEnergy / totalEnergy);
    exclusionRatio = (float)(exclusionEnergy / totalEnergy);
  }
}

// ---------------------------------------------------------------------------
// PATTERN MATCHING
// ---------------------------------------------------------------------------
// after an action fires, taps are ignored for this long so that trailing/extra taps
// of the same gesture can't start a new one (e.g. a 4th tap right after a triple)
#define POST_ACTION_LOCKOUT_MS 700
unsigned long lockoutUntil = 0;

void resetAllPatterns() {
  for (uint8_t p = 0; p < PATTERN_COUNT; p++) {
    patternStates[p].step = 0;
    patternStates[p].pending = false;
  }
}

void onPatternRecognized(const TocPattern &pat) {
  resetAllPatterns();
  DEBUG_PRINTF("=== %s -> %s ===\n", pat.name, pat.action == ACTION_UNLOCK ? "UNLOCK" : "LOCK");
  sendDoorFrame(pat.action == ACTION_UNLOCK ? UNLOCK_CMD_DATA : LOCK_CMD_DATA);

  // waking the bus can block for a second or so, during which the I2S DMA ring keeps filling and
  // then overflows. Drop that backlog so stale audio can't be read back as a fresh tap, and start
  // the lockout from here rather than from before the send.
  i2s_zero_dma_buffer(I2S_PORT);
  lockoutUntil = millis() + POST_ACTION_LOCKOUT_MS;
  if (lastSendOk) ledSolidUntil = millis() + 1000;
  else            ledErrorUntil = millis() + 2000;
}

void checkPatternConfirmations(unsigned long now) {
  for (uint8_t p = 0; p < PATTERN_COUNT; p++) {
    PatternState &st = patternStates[p];
    if (st.pending && (now - st.lastMs) >= PATTERNS[p].trailingSilenceMs) {
      onPatternRecognized(PATTERNS[p]);
      return;
    }
  }
}

void feedAllPatterns(unsigned long now) {
  if ((long)(lockoutUntil - now) > 0) {
    if (debugEnabled) DEBUG_PRINTLN(F("  tap ignored (post-action lockout)"));
    return;
  }
  for (uint8_t p = 0; p < PATTERN_COUNT; p++) {
    TocPattern &pat = PATTERNS[p];
    PatternState &st = patternStates[p];

    // a tap arrived while waiting for the trailing silence: this wasn't a clean
    // double tap (e.g. it's the 3rd tap of a triple), so cancel and restart on this tap
    if (st.pending) {
      if (debugEnabled) DEBUG_PRINTF("  [%s] cancelled (tap during confirmation silence)\n", pat.name);
      st.pending = false;
      st.step = 1;
      st.lastMs = now;
      continue;
    }

    if (st.step == 0) {
      st.step = 1;
      st.lastMs = now;
      continue;
    }

    unsigned long gap = now - st.lastMs;
    GapWindow expected = pat.gaps[st.step - 1];

    if (gap >= expected.minMs && gap <= expected.maxMs) {
      st.step++;
      st.lastMs = now;
      if (debugEnabled) {
        DEBUG_PRINTF("  [%s] tap %d/%d (gap=%lums, expected %lu-%lums)\n",
                     pat.name, st.step, pat.gapCount + 1, gap, expected.minMs, expected.maxMs);
      }
      if (st.step == pat.gapCount + 1) {
        if (pat.trailingSilenceMs > 0) {
          st.pending = true;
          if (debugEnabled) DEBUG_PRINTF("  [%s] complete, waiting %lums of silence...\n", pat.name, pat.trailingSilenceMs);
        } else {
          onPatternRecognized(pat);
          return; // state was reset; don't feed the remaining patterns with this tap
        }
      }
    } else {
      if (debugEnabled) {
        const char* reason = (gap < expected.minMs) ? "too fast" : "too late";
        DEBUG_PRINTF("  [%s] failed (%s, gap=%lums, expected %lu-%lums) -> restarting on this tap\n",
                     pat.name, reason, gap, expected.minMs, expected.maxMs);
      }
      st.step = 1;
      st.lastMs = now;
    }
  }
}

// ---------------------------------------------------------------------------
// SETUP / LOOP
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  pinMode(MCP_INT_PIN, INPUT);
  delay(300);
  DEBUG_PRINTLN(F("=== PKE: double tap = unlock, triple tap = lock ==="));

  canReady = initCan();
  setupI2S();
}

void loop() {
  size_t n = readBlock();
  if (n == 0) return;

  unsigned long now = millis();

  if (!canReady) handleNotReady();
  else           pollCanRx(); // track bus activity so we know if it's awake without transmitting
  updateLed();
  checkPatternConfirmations(now);

  float rms, peak;
  computeRmsPeak(n, rms, peak);

  if (ENVELOPE_DEBUG) {
    Serial.printf("%.0f,%.0f\n", rms, peak);
  }

  if (inImpact) {
    // still decaying: wait for the level to drop below the release thresholds
    if (rms < RMS_RELEASE_THRESHOLD && peak < PEAK_RELEASE_THRESHOLD) {
      unsigned long duration = now - impactStartMs;
      inImpact = false;

      if (tocPendingValidation) {
        tocPendingValidation = false;
        if (duration <= MAX_IMPACT_DURATION_MS) {
          DEBUG_PRINTF(">>> TAP valid (duration=%lums) <<<\n", duration);
          feedAllPatterns(impactStartMs); // keep the tap's original timestamp
        } else {
          DEBUG_PRINTF("  rejected: duration %lums > %lums\n", duration, MAX_IMPACT_DURATION_MS);
        }
      }
    }
    return;
  }

  bool candidate = (rms >= RMS_THRESHOLD) || (peak >= PEAK_THRESHOLD);
  if (!candidate) return;

  if ((now - lastTocMs) < MIN_REFRACTORY_MS) return;

  float ratio, exclusionRatio;
  computeSpectralRatios(n, ratio, exclusionRatio);

  if (ratio >= SPECTRAL_MIN_RATIO && exclusionRatio <= EXCLUSION_MAX_RATIO) {
    inImpact = true;
    impactStartMs = now;
    tocPendingValidation = true;
    lastTocMs = now;
    if (debugEnabled) {
      DEBUG_PRINTF("  tap candidate (ratio=%.2f, exclusion=%.2f), waiting for duration check...\n",
                   ratio, exclusionRatio);
    }
  }
}
