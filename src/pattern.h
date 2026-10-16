#pragma once
// Timing pattern recognition. Validated taps are collected into a sequence
// that ends after `sequenceEndMs` of silence; the whole sequence is then
// compared with each pattern: same tap count, every gap within absolute
// bounds, and gaps normalized by the total duration within tolerance, so
// tapping faster or slower does not matter.

#include <cstdint>

#include "tap_config.h"

namespace tap {

struct TapPattern {
  const char* name;
  uint8_t tapCount;
  float relativeGaps[kMaxTaps - 1];  // e.g. {1, 2} for tap-tap--tap
  float tolerance;                   // max relative error per normalized gap
  uint32_t minGapMs, maxGapMs;
};

struct SequenceResult {
  uint8_t tapCount;  // kMaxTaps + 1 when the sequence overflowed
  uint32_t gapsMs[kMaxTaps - 1];
  const TapPattern* match;  // nullptr if no pattern matched
};

class PatternMatcher {
 public:
  PatternMatcher(const TapPattern* patterns, uint8_t count, uint32_t sequenceEndMs)
      : patterns_(patterns), patternCount_(count), sequenceEndMs_(sequenceEndMs) {}

  void addTap(uint32_t ms);

  // Returns true when a sequence has just ended; `out` then describes it.
  bool poll(uint32_t nowMs, SequenceResult& out);

 private:
  bool matches(const TapPattern& p) const;

  const TapPattern* patterns_;
  uint8_t patternCount_;
  uint32_t sequenceEndMs_;
  uint32_t taps_[kMaxTaps];
  uint8_t count_ = 0;
  bool overflow_ = false;
};

}  // namespace tap
