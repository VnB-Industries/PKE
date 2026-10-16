#include "pattern.h"

#include <cmath>

namespace tap {

void PatternMatcher::addTap(uint32_t ms) {
  if (count_ == kMaxTaps) {
    overflow_ = true;
    taps_[kMaxTaps - 1] = ms;  // keep the latest time for the silence timeout
    return;
  }
  taps_[count_++] = ms;
}

bool PatternMatcher::poll(uint32_t nowMs, SequenceResult& out) {
  if (count_ == 0 || static_cast<int32_t>(nowMs - taps_[count_ - 1]) < static_cast<int32_t>(sequenceEndMs_)) {
    return false;
  }
  out.tapCount = overflow_ ? kMaxTaps + 1 : count_;
  for (uint8_t i = 0; i + 1 < count_; i++) out.gapsMs[i] = taps_[i + 1] - taps_[i];
  out.match = nullptr;
  for (uint8_t p = 0; p < patternCount_ && !overflow_; p++) {
    if (matches(patterns_[p])) {
      out.match = &patterns_[p];
      break;
    }
  }
  count_ = 0;
  overflow_ = false;
  return true;
}

bool PatternMatcher::matches(const TapPattern& p) const {
  if (p.tapCount != count_) return false;
  uint8_t gapCount = count_ - 1;
  float total = 0.0f, relativeTotal = 0.0f;
  for (uint8_t i = 0; i < gapCount; i++) {
    uint32_t gap = taps_[i + 1] - taps_[i];
    if (gap < p.minGapMs || gap > p.maxGapMs) return false;
    total += gap;
    relativeTotal += p.relativeGaps[i];
  }
  for (uint8_t i = 0; i < gapCount; i++) {
    float expected = p.relativeGaps[i] / relativeTotal;
    float actual = (taps_[i + 1] - taps_[i]) / total;
    if (fabsf(actual - expected) > p.tolerance * expected) return false;
  }
  return true;
}

}  // namespace tap
