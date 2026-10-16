#pragma once
// Minimal per-sample filters.

#include <cmath>

namespace tap {

constexpr float kPi = 3.14159265358979f;

// One-pole DC blocker: y[n] = x[n] - x[n-1] + r * y[n-1].
class DcBlocker {
 public:
  void setup(float cutoffHz, float sampleRate) { r_ = 1.0f - 2.0f * kPi * cutoffHz / sampleRate; }

  float process(float x) {
    float y = x - x1_ + r_ * y1_;
    x1_ = x;
    y1_ = y;
    return y;
  }

 private:
  float r_ = 0.995f;
  float x1_ = 0.0f;
  float y1_ = 0.0f;
};

// RBJ cookbook biquad, transposed direct form II.
class Biquad {
 public:
  void highPass(float hz, float sampleRate, float q = 0.7071f) { design(hz, sampleRate, q, true); }
  void lowPass(float hz, float sampleRate, float q = 0.7071f) { design(hz, sampleRate, q, false); }

  float process(float x) {
    float y = b0_ * x + z1_;
    z1_ = b1_ * x - a1_ * y + z2_;
    z2_ = b2_ * x - a2_ * y;
    return y;
  }

 private:
  void design(float hz, float sampleRate, float q, bool high) {
    float w0 = 2.0f * kPi * hz / sampleRate;
    float c = cosf(w0);
    float alpha = sinf(w0) / (2.0f * q);
    float a0 = 1.0f + alpha;
    float k = (high ? 1.0f + c : 1.0f - c) / 2.0f;
    b0_ = k / a0;
    b1_ = (high ? -2.0f * k : 2.0f * k) / a0;
    b2_ = k / a0;
    a1_ = -2.0f * c / a0;
    a2_ = (1.0f - alpha) / a0;
  }

  float b0_ = 1.0f, b1_ = 0.0f, b2_ = 0.0f, a1_ = 0.0f, a2_ = 0.0f;
  float z1_ = 0.0f, z2_ = 0.0f;
};

}  // namespace tap
