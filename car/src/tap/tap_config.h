#pragma once
// Shared constants and tunable thresholds. Used by the firmware and by the
// host tool (tools/tapscan), so keep this file free of Arduino headers.
// Tune every threshold from real recordings (see TAP_TRAINING_PROCESS.md).

#include <cstdint>

namespace tap {

constexpr uint32_t kSampleRate = 16000;
static_assert(kSampleRate % 1000 == 0, "sample counts per ms must be whole");
constexpr uint32_t kSamplesPerMs = kSampleRate / 1000;

constexpr uint32_t kFrameSamples = kSamplesPerMs;  // 1 ms envelope frames
constexpr uint32_t kRiseFrames = 2;                // onset = level jump over 2 ms

// Snippet kept for each event: 10 ms before onset, 64 ms after.
constexpr uint32_t kPreSamples = 10 * kSamplesPerMs;
constexpr uint32_t kPostSamples = 1024;
constexpr uint32_t kSnippetSamples = kPreSamples + kPostSamples;

// Spectral analysis: 512-point Hann window (32 ms) starting 3 ms before onset,
// grouped into log-spaced bands.
constexpr uint32_t kFftSize = 512;
constexpr uint32_t kFftLeadSamples = 3 * kSamplesPerMs;
constexpr uint32_t kBandCount = 16;
constexpr float kBandLowHz = 150.0f;
constexpr float kBandHighHz = 7000.0f;

constexpr uint8_t kMaxTaps = 8;

// Context before an onset: mean in-band level over 300 ms, stopping 15 ms
// before the onset, from a history of 1 ms frame energies.
constexpr uint32_t kActivityHistoryFrames = 512;
constexpr uint32_t kPreActivityFrames = 300;
constexpr uint32_t kPreActivityGapFrames = 15;
static_assert(kPreActivityFrames < kActivityHistoryFrames, "activity history too short");

struct DetectorConfig {
  float dcCutoffHz = 15.0f;
  // Band-pass applied before the envelope. Compare averaged tap and noise
  // spectra (tools/build_template.py) to pick it.
  float bandLowHz = 200.0f;
  float bandHighHz = 4000.0f;
  float triggerDb = 15.0f;     // level above the noise floor needed to trigger
  float minRiseDb = 6.0f;      // level jump over kRiseFrames needed to trigger
  float releaseDb = 6.0f;      // hysteresis: the event ends below floor + releaseDb
  // A new tap can start while the glass still rings from the previous one (double
  // tap ~200 ms apart): retrigger during an active event when the level exceeds
  // the running ringing level by this much. The tail itself fluctuates by up to
  // ~20 dB over 2 ms, so a plain rise test would fire on the ringing.
  float retriggerDb = 15.0f;
  float ringTauMs = 5.0f;      // smoothing of the ringing level
  float minLevelDbfs = -70.0f; // ignore anything quieter, even in a silent room
  uint32_t refractoryMs = 80;  // one detection per tap despite the glass ringing
  uint32_t maxEventMs = 400;   // force release so steady noise cannot freeze the floor
  uint32_t warmupMs = 300;     // discard the I2S start-up transient
  float floorRiseTauMs = 2000.0f;
  float floorFallTauMs = 200.0f;
};

struct ValidatorConfig {
  // Tuned on recordings/train (sessions 1 and 2), checked on recordings/test:
  // taps peak -26..-10 dBFS, contrast 29..55 dB, centroid 120..570 Hz, flatness ~0.
  // Level: too faint or too violent to be a finger tap. Soft taps peak around
  // -29..-31 dBFS; below -32 ambient glass ringing starts to pass.
  float minPeakDbfs = -32.0f;
  float maxPeakDbfs = -3.0f;
  // Temporal shape. Rise and decay overlap between taps and non-taps (taps rise
  // in 0.3..30 ms), so they only reject slow swells.
  float maxRiseMs = 30.0f;
  float maxDecayMs = 62.0f;      // time to fall 20 dB below the peak; snippet caps it near 61
  float minCrestDb = 6.0f;
  float minPreContrastDb = 20.0f;  // peak vs energy just before the onset: the strongest feature
  // Spectral shape: rejects impulsive sounds that did not come from the glass.
  float minBandRatio = 0.15f;      // energy share in 200-1600 Hz
  float maxExclusionRatio = 0.05f; // energy share in 2000-6000 Hz
  float minCentroidHz = 100.0f;    // bumps and handling noise sit around 35-50 Hz
  float maxCentroidHz = 1000.0f;
  float maxFlatness = 0.05f;
  // Context: speech is already active before its plosives, a tap starts from
  // quiet. Taps right after an accepted tap (double tap) are exempt, since the
  // previous tap's ringing fills the window.
  float maxPreActivityDb = 20.0f;  // level over the 300 ms before onset, above the floor
  uint32_t followUpMs = 700;
  float minSimilarity = 0.92f;     // cosine vs tap template, if one is built (taps p5 ~0.93-0.97)
};

}  // namespace tap
