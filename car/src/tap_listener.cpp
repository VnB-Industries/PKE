/*
 * Window tap gestures, ported from the ESP32 PKE.ino (branch ESP32/main):
 * double tap = unlock, triple tap = lock, heard by an INMP441 on the window.
 *
 * The mic is clocked only while an authenticated fob is connected, so a tap
 * alone can no longer open the car. The capture thread reads I2S and runs the
 * cheap per-sample detector (tap/detector.h); the processing thread validates
 * each event (tap/tap_features.h) and matches the tap sequence (tap/pattern.h).
 * Between sessions the I2S peripheral is stopped and both threads block on
 * listen_event, so the mic draws nothing and the CPU idles.
 */
#include <new>

#include <zephyr/device.h>
#include <zephyr/drivers/i2s.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

#include "detector.h"
#include "door.h"
#include "pattern.h"
#include "tap_features.h"
#include "tap_listener.h"

LOG_MODULE_REGISTER(tap, LOG_LEVEL_INF);

namespace {

constexpr size_t kBlockSamples = 256;  // samples per I2S block (16 ms)
constexpr size_t kBlockBytes = kBlockSamples * sizeof(int32_t);
constexpr uint32_t kBlockCount = 4;
constexpr int32_t kReadTimeoutMs = 100;
// The nRF52840 I2S stops at 24-bit samples, stored sign-extended in 32-bit words.
constexpr float kRawToFloat = 1.0f / 8388608.0f;
constexpr uint8_t kEventQueueLength = 3;
// How often the processing thread checks for a finished sequence while listening.
constexpr int32_t kPollMs = 50;

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
const door_cmd kPatternActions[] = {DOOR_UNLOCK, DOOR_LOCK};  // same order as kPatterns

constexpr uint32_t kListening = BIT(0);

const struct device* const i2s_dev = DEVICE_DT_GET(DT_NODELABEL(i2s0));
bool i2s_ready;

K_EVENT_DEFINE(listen_event);
K_MEM_SLAB_DEFINE_STATIC(rx_slab, kBlockBytes, kBlockCount, 4);
K_MSGQ_DEFINE(event_q, sizeof(tap::TapEvent), kEventQueueLength, 4);

// Static: the detector holds a 2048-sample ring, far too big for a thread stack. It is rebuilt in
// place on each start, so every session begins with a fresh noise floor and warm-up.
alignas(tap::Detector) uint8_t detector_mem[sizeof(tap::Detector)];
tap::Detector* detector;
tap::ValidatorConfig validator_config;
tap::PatternMatcher matcher(kPatterns, ARRAY_SIZE(kPatterns), kSequenceEndMs);

atomic_t audio_now_ms;
atomic_t dropped_events;

bool listening() { return k_event_test(&listen_event, kListening) != 0; }

void wait_listening() { k_event_wait(&listen_event, kListening, false, K_FOREVER); }

int start_i2s() {
  i2s_config cfg = {};
  cfg.word_size = 24;
  cfg.channels = 1;  // INMP441 L/R tied to GND: left channel
  cfg.format = I2S_FMT_DATA_FORMAT_I2S;
  cfg.options = I2S_OPT_BIT_CLK_CONTROLLER | I2S_OPT_FRAME_CLK_CONTROLLER;
  cfg.frame_clk_freq = tap::kSampleRate;
  cfg.mem_slab = &rx_slab;
  cfg.block_size = kBlockBytes;
  cfg.timeout = kReadTimeoutMs;

  int err = i2s_configure(i2s_dev, I2S_DIR_RX, &cfg);
  if (!err) err = i2s_trigger(i2s_dev, I2S_DIR_RX, I2S_TRIGGER_START);
  if (err) LOG_ERR("I2S start failed (err %d)", err);
  return err;
}

// ---------------------------------------------------------------------------
// CAPTURE THREAD: reads I2S without interruption and runs only the cheap per-sample detector, so
// it keeps reading while the processing thread blocks on a CAN send.
// ---------------------------------------------------------------------------
void capture_thread(void*, void*, void*) {
  static tap::TapEvent event;  // too big for the thread stack

  for (;;) {
    wait_listening();
    detector = new (detector_mem) tap::Detector();
    if (start_i2s()) {
      k_event_clear(&listen_event, kListening);
      continue;
    }
    LOG_INF("mic on");

    while (listening()) {
      void* block;
      size_t size;
      int err = i2s_read(i2s_dev, &block, &size);
      if (err) {
        LOG_WRN("I2S read failed (err %d)", err);
        break;
      }
      const int32_t* raw = static_cast<const int32_t*>(block);
      for (size_t i = 0; i < size / sizeof(int32_t); i++) {
        if (detector->process(raw[i] * kRawToFloat, event) &&
            k_msgq_put(&event_q, &event, K_NO_WAIT) != 0) {
          atomic_inc(&dropped_events);
        }
      }
      k_mem_slab_free(&rx_slab, block);
      atomic_set(&audio_now_ms, detector->nowMs());
    }

    // DROP stops the clocks at once and frees the queued blocks; without SCK the INMP441 sleeps.
    i2s_trigger(i2s_dev, I2S_DIR_RX, I2S_TRIGGER_DROP);
    LOG_INF("mic off");
  }
}

// ---------------------------------------------------------------------------
// PROCESSING THREAD
// ---------------------------------------------------------------------------
bool has_last_tap;
uint32_t last_tap_ms;

void handle_event(const tap::TapEvent& ev) {
  tap::TapFeatures f;
  tap::computeFeatures(ev.samples, f);
  const char* reject = tap::checkFeatures(f, validator_config);
  uint32_t since_tap = has_last_tap ? ev.onsetMs - last_tap_ms : UINT32_MAX;
  if (!reject) reject = tap::checkContext(ev.preActivityDb, since_tap, validator_config);

  // Every event is logged, accepted or not, to refine thresholds later.
  LOG_INF("EVT t=%u floor=%.1f peak=%.1f rise=%.2f decay=%.0f crest=%.1f pre=%.1f "
          "band=%.2f excl=%.2f cent=%.0f flat=%.2f sim=%.2f act=%.1f -> %s",
          ev.onsetMs, (double)ev.floorDb, (double)f.peakDbfs, (double)f.riseMs,
          (double)f.decayMs, (double)f.crestDb, (double)f.preContrastDb, (double)f.bandRatio,
          (double)f.exclusionRatio, (double)f.centroidHz, (double)f.flatness,
          (double)f.similarity, (double)ev.preActivityDb, reject ? reject : "TAP");
  if (!reject) {
    has_last_tap = true;
    last_tap_ms = ev.onsetMs;
    matcher.addTap(ev.onsetMs);
  }
}

void handle_sequence(const tap::SequenceResult& seq) {
  if (!seq.match) {
    LOG_INF("SEQ %u taps, no match", seq.tapCount);
    return;
  }

  door_cmd action = kPatternActions[seq.match - kPatterns];
  LOG_INF("=== %s -> %s ===", seq.match->name, action == DOOR_UNLOCK ? "UNLOCK" : "LOCK");
  door_send(action);
}

void process_thread(void*, void*, void*) {
  static tap::TapEvent ev;  // too big for the thread stack
  atomic_val_t reported_drops = 0;

  for (;;) {
    wait_listening();
    matcher = tap::PatternMatcher(kPatterns, ARRAY_SIZE(kPatterns), kSequenceEndMs);
    has_last_tap = false;

    while (listening()) {
      if (k_msgq_get(&event_q, &ev, K_MSEC(kPollMs)) == 0) handle_event(ev);

      tap::SequenceResult seq;
      if (matcher.poll(atomic_get(&audio_now_ms), seq)) handle_sequence(seq);

      atomic_val_t drops = atomic_get(&dropped_events);
      if (drops != reported_drops) {
        reported_drops = drops;
        LOG_WRN("%ld events dropped (queue full)", (long)drops);
      }
    }
  }
}

// The capture thread outranks the processing thread so I2S blocks are never starved.
K_THREAD_DEFINE(tap_capture, 1024, capture_thread, nullptr, nullptr, nullptr, 2, 0, 0);
K_THREAD_DEFINE(tap_process, 3072, process_thread, nullptr, nullptr, nullptr, 8, 0, 0);

}  // namespace

extern "C" int tap_listener_init(void) {
  i2s_ready = device_is_ready(i2s_dev);
  if (!i2s_ready) {
    LOG_ERR("I2S not ready, tap gestures disabled");
    return -ENODEV;
  }
  return 0;
}

extern "C" void tap_listener_start(void) {
  if (!i2s_ready) return;
  k_msgq_purge(&event_q);
  atomic_set(&audio_now_ms, 0);
  k_event_post(&listen_event, kListening);
  LOG_INF("tap listener started: double tap = unlock, triple tap = lock");
}

extern "C" void tap_listener_stop(void) {
  if (!i2s_ready || !listening()) return;
  k_event_clear(&listen_event, kListening);
  LOG_INF("tap listener stopped");
}
