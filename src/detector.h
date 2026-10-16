#pragma once
// Per-sample onset detector: DC blocker, band-pass, 1 ms energy envelope in
// dB, adaptive noise floor, trigger with refractory period and hysteresis.
// Each onset yields a TapEvent holding the DC-blocked snippet around it, so
// expensive features are computed only on events (see tap_features.h).

#include <cstdint>

#include "dsp.h"
#include "tap_config.h"

namespace tap {

struct TapEvent {
  uint32_t onsetMs;
  float floorDb;                    // noise floor at trigger time
  float preActivityDb;              // in-band level over the 300 ms before onset, above floorDb
  float samples[kSnippetSamples];   // onset at index kPreSamples
};

class Detector {
 public:
  explicit Detector(const DetectorConfig& cfg = DetectorConfig());

  // Feeds one sample (full scale = 1.0). Returns true when `out` holds a
  // completed event, kPostSamples after its onset.
  bool process(float x, TapEvent& out);

  float lastSample() const { return last_; }  // DC-blocked
  uint32_t nowMs() const { return static_cast<uint32_t>(count_ / kSamplesPerMs); }
  float noiseFloorDb() const { return floorDb_; }

 private:
  static constexpr uint32_t kRingSize = 2048;
  static constexpr uint32_t kRingMask = kRingSize - 1;
  static_assert(kRingSize >= kSnippetSamples + kFrameSamples, "ring too small");

  void onFrame(float levelDb);

  DetectorConfig cfg_;
  DcBlocker dc_;
  Biquad hp_, lp_;
  uint64_t warmupSamples_, refractorySamples_, maxEventSamples_;
  float floorRiseAlpha_, floorFallAlpha_, ringAlpha_;

  float ring_[kRingSize] = {};
  uint64_t count_ = 0;
  float last_ = 0.0f;

  float frameEnergy_ = 0.0f;
  uint32_t frameFill_ = 0;
  float activity_[kActivityHistoryFrames] = {};  // linear in-band frame energies
  uint32_t activityPos_ = 0;
  float history_[kRiseFrames] = {};
  uint32_t historyPos_ = 0;

  float floorDb_ = -120.0f;
  bool active_ = false;
  uint64_t onset_ = 0;
  bool capturing_ = false;
  float ringDb_ = 0.0f;  // smoothed level while the event rings, for retriggers
  float eventFloorDb_ = 0.0f;
  float eventPreActivityDb_ = 0.0f;
};

}  // namespace tap
