#include "detector.h"

#include <cmath>

namespace tap {

namespace {
uint64_t msToSamples(uint32_t ms) { return static_cast<uint64_t>(ms) * kSamplesPerMs; }
float frameAlpha(float tauMs) { return 1.0f - expf(-1.0f / tauMs); }  // one frame = 1 ms
}  // namespace

Detector::Detector(const DetectorConfig& cfg)
    : cfg_(cfg),
      warmupSamples_(msToSamples(cfg.warmupMs)),
      refractorySamples_(msToSamples(cfg.refractoryMs)),
      maxEventSamples_(msToSamples(cfg.maxEventMs)),
      floorRiseAlpha_(frameAlpha(cfg.floorRiseTauMs)),
      floorFallAlpha_(frameAlpha(cfg.floorFallTauMs)),
      ringAlpha_(frameAlpha(cfg.ringTauMs)) {
  dc_.setup(cfg.dcCutoffHz, kSampleRate);
  hp_.highPass(cfg.bandLowHz, kSampleRate);
  lp_.lowPass(cfg.bandHighHz, kSampleRate);
}

bool Detector::process(float x, TapEvent& out) {
  last_ = dc_.process(x);
  ring_[count_ & kRingMask] = last_;
  ++count_;

  float y = lp_.process(hp_.process(last_));
  frameEnergy_ += y * y;
  if (++frameFill_ == kFrameSamples) {
    activity_[activityPos_] = frameEnergy_ / kFrameSamples;
    activityPos_ = (activityPos_ + 1) % kActivityHistoryFrames;
    onFrame(10.0f * log10f(frameEnergy_ / kFrameSamples + 1e-20f));
    frameEnergy_ = 0.0f;
    frameFill_ = 0;
  }

  if (!capturing_ || count_ < onset_ + kPostSamples) return false;
  capturing_ = false;
  uint64_t start = onset_ - kPreSamples;
  for (uint32_t i = 0; i < kSnippetSamples; i++) out.samples[i] = ring_[(start + i) & kRingMask];
  out.onsetMs = static_cast<uint32_t>(onset_ / kSamplesPerMs);
  out.floorDb = eventFloorDb_;
  out.preActivityDb = eventPreActivityDb_;
  return true;
}

void Detector::onFrame(float levelDb) {
  float rise = levelDb - history_[historyPos_];  // vs kRiseFrames frames ago
  history_[historyPos_] = levelDb;
  historyPos_ = (historyPos_ + 1) % kRiseFrames;

  if (count_ < warmupSamples_) {
    floorDb_ = levelDb;
    return;
  }

  uint64_t sinceOnset = count_ - onset_;
  bool refractory = onset_ != 0 && sinceOnset < refractorySamples_;

  bool loudEnough = levelDb >= floorDb_ + cfg_.triggerDb && levelDb >= cfg_.minLevelDbfs;

  if (active_) {
    // Noise floor stays frozen until the event has died out.
    bool retrigger = !refractory && !capturing_ && loudEnough && rise >= cfg_.minRiseDb &&
                     levelDb >= ringDb_ + cfg_.retriggerDb;
    ringDb_ += ringAlpha_ * (levelDb - ringDb_);
    if (!retrigger) {
      bool released = !refractory && levelDb < floorDb_ + cfg_.releaseDb;
      if (released || sinceOnset >= maxEventSamples_) active_ = false;
      return;
    }
  }

  if (!refractory && loudEnough && rise >= cfg_.minRiseDb) {
    active_ = true;
    ringDb_ = levelDb;
    capturing_ = true;
    onset_ = count_ - kFrameSamples;  // start of the triggering frame
    eventFloorDb_ = floorDb_;
    float pre = 0.0f;  // the current frame is activity_[activityPos_ - 1]
    for (uint32_t k = kPreActivityGapFrames + 1; k <= kPreActivityFrames; k++) {
      pre += activity_[(activityPos_ + kActivityHistoryFrames - k) % kActivityHistoryFrames];
    }
    pre /= kPreActivityFrames - kPreActivityGapFrames;
    eventPreActivityDb_ = 10.0f * log10f(pre + 1e-20f) - floorDb_;
    return;
  }

  floorDb_ += (levelDb > floorDb_ ? floorRiseAlpha_ : floorFallAlpha_) * (levelDb - floorDb_);
}

}  // namespace tap
