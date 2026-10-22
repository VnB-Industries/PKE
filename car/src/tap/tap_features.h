#pragma once
// Per-event validation features, computed on a TapEvent snippet only.

#include "tap_config.h"

namespace tap {

struct TapFeatures {
  // Temporal
  float peakDbfs;
  float riseMs;         // 10% -> 90% of peak amplitude
  float decayMs;        // peak frame -> 20 dB below it
  float crestDb;        // peak vs RMS over the FFT window
  float preContrastDb;  // peak frame vs energy just before the onset
  // Spectral
  float bandRatio;
  float exclusionRatio;
  float centroidHz;
  float flatness;
  float similarity;           // NaN when no template is built
  float bands[kBandCount];    // band dB, zero mean, unit norm
};

// `snippet` holds kSnippetSamples with the onset at kPreSamples.
void computeFeatures(const float* snippet, TapFeatures& f);

// Returns nullptr if the event looks like a tap, otherwise the name of the
// first failed check (temporal checks first).
const char* checkFeatures(const TapFeatures& f, const ValidatorConfig& cfg);

// Checks the event's context once its features passed: `msSinceLastTap` is
// the time since the previous accepted tap (UINT32_MAX if none).
const char* checkContext(float preActivityDb, uint32_t msSinceLastTap, const ValidatorConfig& cfg);

}  // namespace tap
