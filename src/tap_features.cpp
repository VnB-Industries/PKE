#include "tap_features.h"

#include <cmath>

#include "dsp.h"
#include "tap_template.h"

namespace tap {

namespace {

constexpr uint32_t kBins = kFftSize / 2;
constexpr float kHzPerBin = static_cast<float>(kSampleRate) / kFftSize;
constexpr float kEps = 1e-20f;

// Legacy band checks, kept from the first version of the detector.
constexpr float kBandRatioLowHz = 200.0f, kBandRatioHighHz = 1600.0f;
constexpr float kExclusionLowHz = 2000.0f, kExclusionHighHz = 6000.0f;

float cosTable[kBins], sinTable[kBins], hann[kFftSize];
int8_t bandOfBin[kBins];  // -1 outside [kBandLowHz, kBandHighHz)
float re[kFftSize], im[kFftSize];
bool tablesReady = false;

void initTables() {
  for (uint32_t k = 0; k < kBins; k++) {
    cosTable[k] = cosf(2.0f * kPi * k / kFftSize);
    sinTable[k] = sinf(2.0f * kPi * k / kFftSize);
  }
  for (uint32_t i = 0; i < kFftSize; i++) hann[i] = 0.5f - 0.5f * cosf(2.0f * kPi * i / kFftSize);
  float ratio = kBandHighHz / kBandLowHz;
  for (uint32_t k = 0; k < kBins; k++) {
    float hz = k * kHzPerBin;
    bandOfBin[k] = -1;
    if (hz < kBandLowHz || hz >= kBandHighHz) continue;
    int band = static_cast<int>(kBandCount * logf(hz / kBandLowHz) / logf(ratio));
    bandOfBin[k] = static_cast<int8_t>(band < static_cast<int>(kBandCount) ? band : kBandCount - 1);
  }
  tablesReady = true;
}

// In-place iterative radix-2 FFT.
void fft() {
  for (uint32_t i = 1, j = 0; i < kFftSize; i++) {
    uint32_t bit = kFftSize >> 1;
    for (; j & bit; bit >>= 1) j ^= bit;
    j |= bit;
    if (i < j) {
      float t = re[i]; re[i] = re[j]; re[j] = t;
      t = im[i]; im[i] = im[j]; im[j] = t;
    }
  }
  for (uint32_t len = 2; len <= kFftSize; len <<= 1) {
    uint32_t half = len / 2, step = kFftSize / len;
    for (uint32_t i = 0; i < kFftSize; i += len) {
      for (uint32_t k = 0; k < half; k++) {
        float wr = cosTable[k * step], wi = -sinTable[k * step];
        uint32_t a = i + k, b = a + half;
        float tr = re[b] * wr - im[b] * wi;
        float ti = re[b] * wi + im[b] * wr;
        re[b] = re[a] - tr;
        im[b] = im[a] - ti;
        re[a] += tr;
        im[a] += ti;
      }
    }
  }
}

float toDb(float energy) { return 10.0f * log10f(energy + kEps); }

void computeTemporal(const float* s, TapFeatures& f) {
  const uint32_t w0 = kPreSamples - kFftLeadSamples;

  float peak = 0.0f, sumSq = 0.0f;
  uint32_t peakIdx = kPreSamples;
  for (uint32_t i = w0; i < w0 + kFftSize; i++) {
    float a = fabsf(s[i]);
    sumSq += s[i] * s[i];
    if (a > peak) { peak = a; peakIdx = i; }
  }
  f.peakDbfs = 20.0f * log10f(peak + kEps);
  f.crestDb = 20.0f * log10f((peak + kEps) / (sqrtf(sumSq / kFftSize) + kEps));

  uint32_t t10 = w0;
  while (t10 < peakIdx && fabsf(s[t10]) < 0.1f * peak) t10++;
  uint32_t t90 = t10;
  while (t90 < peakIdx && fabsf(s[t90]) < 0.9f * peak) t90++;
  f.riseMs = static_cast<float>(t90 - t10) / kSamplesPerMs;

  constexpr uint32_t kFrames = kSnippetSamples / kFrameSamples;
  constexpr uint32_t kOnsetFrame = kPreSamples / kFrameSamples;
  // Decay is judged on a 5 ms moving average so the gaps between voice pitch
  // pulses do not look like a fast decay.
  constexpr uint32_t kDecaySmoothFrames = 5;
  float frameEnergy[kFrames];
  for (uint32_t fr = 0; fr < kFrames; fr++) {
    float e = 0.0f;
    for (uint32_t i = 0; i < kFrameSamples; i++) {
      float v = s[fr * kFrameSamples + i];
      e += v * v;
    }
    frameEnergy[fr] = e / kFrameSamples;
  }

  float preEnergy = 0.0f;
  for (uint32_t fr = 0; fr + 1 < kOnsetFrame; fr++) preEnergy += frameEnergy[fr];
  uint32_t peakFrame = peakIdx / kFrameSamples;
  f.preContrastDb = toDb(frameEnergy[peakFrame]) - toDb(preEnergy / (kOnsetFrame - 1));

  float decayed = frameEnergy[peakFrame] * 0.01f;  // -20 dB
  uint32_t end = peakFrame + 1;
  for (; end + kDecaySmoothFrames <= kFrames; end++) {
    float smooth = 0.0f;
    for (uint32_t k = 0; k < kDecaySmoothFrames; k++) smooth += frameEnergy[end + k];
    if (smooth / kDecaySmoothFrames < decayed) break;
  }
  f.decayMs = static_cast<float>(end - peakFrame);  // capped near the snippet end if it never decays
}

void computeSpectral(const float* s, TapFeatures& f) {
  const float* w = s + kPreSamples - kFftLeadSamples;
  for (uint32_t i = 0; i < kFftSize; i++) {
    re[i] = w[i] * hann[i];
    im[i] = 0.0f;
  }
  fft();

  float total = 0.0f, band = 0.0f, exclusion = 0.0f, weighted = 0.0f;
  float inRange = 0.0f, logSum = 0.0f;
  uint32_t inRangeBins = 0;
  float bandEnergy[kBandCount] = {};
  for (uint32_t k = 1; k < kBins; k++) {
    float p = re[k] * re[k] + im[k] * im[k];
    float hz = k * kHzPerBin;
    total += p;
    weighted += p * hz;
    if (hz >= kBandRatioLowHz && hz < kBandRatioHighHz) band += p;
    if (hz >= kExclusionLowHz && hz < kExclusionHighHz) exclusion += p;
    if (bandOfBin[k] >= 0) {
      bandEnergy[bandOfBin[k]] += p;
      inRange += p;
      logSum += logf(p + kEps);
      inRangeBins++;
    }
  }

  f.bandRatio = total > 0.0f ? band / total : 0.0f;
  f.exclusionRatio = total > 0.0f ? exclusion / total : 0.0f;
  f.centroidHz = total > 0.0f ? weighted / total : 0.0f;
  f.flatness = inRange > 0.0f ? expf(logSum / inRangeBins) / (inRange / inRangeBins) : 1.0f;

  // Band shape: dB, zero mean, unit norm, so the dot product is a cosine.
  float mean = 0.0f;
  for (uint32_t b = 0; b < kBandCount; b++) {
    f.bands[b] = toDb(bandEnergy[b]);
    mean += f.bands[b];
  }
  mean /= kBandCount;
  float norm = 0.0f;
  for (uint32_t b = 0; b < kBandCount; b++) {
    f.bands[b] -= mean;
    norm += f.bands[b] * f.bands[b];
  }
  norm = sqrtf(norm);
  for (uint32_t b = 0; b < kBandCount; b++) f.bands[b] = norm > 0.0f ? f.bands[b] / norm : 0.0f;

  f.similarity = NAN;
  if (kHasTapTemplate) {
    float dot = 0.0f;
    for (uint32_t b = 0; b < kBandCount; b++) dot += f.bands[b] * kTapTemplate[b];
    f.similarity = dot;
  }
}

}  // namespace

void computeFeatures(const float* snippet, TapFeatures& f) {
  if (!tablesReady) initTables();
  computeTemporal(snippet, f);
  computeSpectral(snippet, f);
}

const char* checkFeatures(const TapFeatures& f, const ValidatorConfig& cfg) {
  if (f.peakDbfs < cfg.minPeakDbfs) return "too_faint";
  if (f.peakDbfs > cfg.maxPeakDbfs) return "too_loud";
  if (f.riseMs > cfg.maxRiseMs) return "slow_rise";
  if (f.decayMs > cfg.maxDecayMs) return "slow_decay";
  if (f.crestDb < cfg.minCrestDb) return "low_crest";
  if (f.preContrastDb < cfg.minPreContrastDb) return "noisy_before";
  if (f.bandRatio < cfg.minBandRatio) return "band_ratio";
  if (f.exclusionRatio > cfg.maxExclusionRatio) return "exclusion";
  if (f.centroidHz < cfg.minCentroidHz || f.centroidHz > cfg.maxCentroidHz) return "centroid";
  if (f.flatness > cfg.maxFlatness) return "flatness";
  if (kHasTapTemplate && f.similarity < cfg.minSimilarity) return "template";
  return nullptr;
}

const char* checkContext(float preActivityDb, uint32_t msSinceLastTap, const ValidatorConfig& cfg) {
  if (msSinceLastTap > cfg.followUpMs && preActivityDb > cfg.maxPreActivityDb) return "busy_before";
  return nullptr;
}

}  // namespace tap
