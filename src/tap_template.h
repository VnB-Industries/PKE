#pragma once
// Tap spectral template. Regenerate with tools/build_template.py.
// Built from 245 tap events. Per-band spread (std):
//   0.057 0.078 0.055 0.049 0.050 0.050 0.047 0.045 0.060 0.068 0.059 0.050 0.034 0.034 0.043 0.057

#include "tap_config.h"

namespace tap {

constexpr bool kHasTapTemplate = true;
constexpr float kTapTemplate[kBandCount] = {
    0.22200f, 0.18077f, 0.23067f, 0.27757f, 0.25839f, 0.22469f, 0.17510f, 0.16166f, 0.10307f, -0.00801f, -0.14123f, -0.25365f, -0.30227f, -0.33702f, -0.38191f, -0.40983f,
};

}  // namespace tap
