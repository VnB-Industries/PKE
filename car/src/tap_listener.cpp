/*
 * Window tap gestures, ported from car/src/tap_listener.cpp on nrf52840/main:
 * double tap = unlock, triple tap = lock, heard by an INMP441 on the window.
 *
 * The mic is clocked only while the fob is detected, so a tap alone can't
 * open the car. The capture task reads I2S and runs the cheap per-sample
 * detector (tap/detector.h); the processing task validates each event
 * (tap/tap_features.h) and matches the tap sequence (tap/pattern.h).
 * Between sessions the I2S peripheral is stopped and both tasks block on
 * listen_event, so the mic draws nothing.
 */
#include <errno.h>

#include <atomic>
#include <new>

#include <Arduino.h>
#include <driver/i2s.h>
#include <freertos/event_groups.h>

#include "board.h"
#include "detector.h"
#include "door.h"
#include "log.h"
#include "pattern.h"
#include "tap_features.h"
#include "tap_listener.h"

LOG_MODULE("tap");

namespace {

constexpr size_t kBlockSamples = 256;  // samples per I2S read (16 ms)
constexpr int kDmaBufCount = 8;
constexpr TickType_t kReadTimeout = pdMS_TO_TICKS(100);
// 24-bit data, left-justified in 32 bits.
constexpr float kRawToFloat = 1.0f / 2147483648.0f;
constexpr uint8_t kEventQueueLength = 3;
// How often the processing task checks for a finished sequence while listening.
constexpr TickType_t kPoll = pdMS_TO_TICKS(50);

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
constexpr size_t kPatternCount = sizeof(kPatterns) / sizeof(kPatterns[0]);
const door_cmd kPatternActions[] = {DOOR_UNLOCK, DOOR_LOCK};  // same order as kPatterns

constexpr EventBits_t kListening = BIT0;

bool i2s_ready;
EventGroupHandle_t listen_event;
QueueHandle_t event_q;

// Static: the detector holds a 2048-sample ring, far too big for a task stack. It is rebuilt in
// place on each start, so every session begins with a fresh noise floor and warm-up.
alignas(tap::Detector) uint8_t detector_mem[sizeof(tap::Detector)];
tap::Detector* detector;
tap::ValidatorConfig validator_config;
tap::PatternMatcher matcher(kPatterns, kPatternCount, kSequenceEndMs);

std::atomic<uint32_t> audio_now_ms;
std::atomic<uint32_t> dropped_events;

bool listening() { return (xEventGroupGetBits(listen_event) & kListening) != 0; }

void wait_listening() { xEventGroupWaitBits(listen_event, kListening, pdFALSE, pdTRUE, portMAX_DELAY); }

// The driver starts the clocks on install; they are stopped again until the first session.
esp_err_t install_i2s() {
  i2s_config_t cfg = {};
  cfg.mode = static_cast<i2s_mode_t>(I2S_MODE_MASTER | I2S_MODE_RX);
  cfg.sample_rate = tap::kSampleRate;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;
  // INMP441 with L/R tied to GND transmits in the slot the ESP32 legacy I2S
  // driver reads as RIGHT; ONLY_LEFT reads the empty slot (raw_max stuck at 1).
  cfg.channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT;
  cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  cfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  cfg.dma_buf_count = kDmaBufCount;
  cfg.dma_buf_len = kBlockSamples;

  i2s_pin_config_t pins = {};
  pins.mck_io_num = I2S_PIN_NO_CHANGE;
  pins.bck_io_num = I2S_SCK_PIN;
  pins.ws_io_num = I2S_WS_PIN;
  pins.data_out_num = I2S_PIN_NO_CHANGE;
  pins.data_in_num = I2S_SD_PIN;

  esp_err_t err = i2s_driver_install(I2S_PORT, &cfg, 0, nullptr);
  if (err == ESP_OK) err = i2s_set_pin(I2S_PORT, &pins);
  if (err == ESP_OK) err = i2s_stop(I2S_PORT);
  return err;
}

// ---------------------------------------------------------------------------
// CAPTURE TASK: reads I2S without interruption and runs only the cheap per-sample detector, so
// it keeps reading while the processing task blocks on a CAN send.
// ---------------------------------------------------------------------------
void capture_task(void*) {
  static int32_t raw[kBlockSamples];
  static tap::TapEvent event;  // too big for the task stack

  for (;;) {
    wait_listening();
    detector = new (detector_mem) tap::Detector();
    i2s_zero_dma_buffer(I2S_PORT);
    esp_err_t err = i2s_start(I2S_PORT);
    if (err != ESP_OK) {
      LOG_ERR("I2S start failed (err %d)", err);
      xEventGroupClearBits(listen_event, kListening);
      continue;
    }
    LOG_INF("mic on");

    while (listening()) {
      size_t bytes = 0;
      err = i2s_read(I2S_PORT, raw, sizeof(raw), &bytes, kReadTimeout);
      if (err != ESP_OK) {
        LOG_WRN("I2S read failed (err %d)", err);
        break;
      }
      for (size_t i = 0; i < bytes / sizeof(int32_t); i++) {
        if (detector->process(raw[i] * kRawToFloat, event) &&
            xQueueSend(event_q, &event, 0) != pdTRUE) {
          dropped_events++;
        }
      }
      audio_now_ms = detector->nowMs();
    }

    // Without SCK the INMP441 sleeps.
    i2s_stop(I2S_PORT);
    LOG_INF("mic off");
  }
}

// ---------------------------------------------------------------------------
// PROCESSING TASK
// ---------------------------------------------------------------------------
bool has_last_tap;
uint32_t last_tap_ms;

void handle_event(const tap::TapEvent& ev) {
  tap::TapFeatures f;
  tap::computeFeatures(ev.samples, f);
  const char* reject = tap::checkFeatures(f, validator_config);
  uint32_t since_tap = has_last_tap ? ev.onsetMs - last_tap_ms : UINT32_MAX;
  if (!reject) reject = tap::checkContext(ev.preActivityDb, since_tap, validator_config);

  // Only recognized taps are logged.
  if (!reject) {
    LOG_INF("TAP t=%lu peak=%.1f", (unsigned long)ev.onsetMs, f.peakDbfs);
    has_last_tap = true;
    last_tap_ms = ev.onsetMs;
    matcher.addTap(ev.onsetMs);
  }
}

void handle_sequence(const tap::SequenceResult& seq) {
  if (!seq.match) {
    return;
  }

  door_cmd action = kPatternActions[seq.match - kPatterns];
  LOG_INF("=== %s -> %s ===", seq.match->name, action == DOOR_UNLOCK ? "UNLOCK" : "LOCK");
  door_send(action);
}

void process_task(void*) {
  static tap::TapEvent ev;  // too big for the task stack
  uint32_t reported_drops = 0;

  for (;;) {
    wait_listening();
    matcher = tap::PatternMatcher(kPatterns, kPatternCount, kSequenceEndMs);
    has_last_tap = false;

    while (listening()) {
      if (xQueueReceive(event_q, &ev, kPoll) == pdTRUE) handle_event(ev);

      tap::SequenceResult seq;
      if (matcher.poll(audio_now_ms, seq)) handle_sequence(seq);

      uint32_t drops = dropped_events;
      if (drops != reported_drops) {
        reported_drops = drops;
        LOG_WRN("%lu events dropped (queue full)", (unsigned long)drops);
      }
    }
  }
}

}  // namespace

int tap_listener_init(void) {
  listen_event = xEventGroupCreate();
  event_q = xQueueCreate(kEventQueueLength, sizeof(tap::TapEvent));

  esp_err_t err = install_i2s();
  if (err != ESP_OK) {
    LOG_ERR("I2S init failed (err %d), tap gestures disabled", err);
    return -ENODEV;
  }
  i2s_ready = true;

  // Capture on core 0 at high priority so I2S blocks are never starved; processing on core 1
  // next to the loop task, below the CAN receive task.
  xTaskCreatePinnedToCore(capture_task, "tap_capture", 4096, nullptr, configMAX_PRIORITIES - 2,
                          nullptr, 0);
  xTaskCreatePinnedToCore(process_task, "tap_process", 8192, nullptr, 2, nullptr, 1);
  return 0;
}

void tap_listener_start(void) {
  if (!i2s_ready) return;
  xQueueReset(event_q);
  audio_now_ms = 0;
  xEventGroupSetBits(listen_event, kListening);
  LOG_INF("tap listener started: double tap = unlock, triple tap = lock");
}

void tap_listener_stop(void) {
  if (!i2s_ready || !listening()) return;
  xEventGroupClearBits(listen_event, kListening);
  LOG_INF("tap listener stopped");
}
