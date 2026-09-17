#pragma once

// Target publishing rule, kept free of ESPHome headers so the host tests can check it.

#include <cmath>

namespace esphome::dfrobot_mmwave::protocol {

/// Every finite reading is published, so sensor filters such as throttle_with_priority see each one; a NaN
/// ("no target") is published once until a finite reading arrives again.
struct NanRepeatFilter {
  bool next(float v) {
    const bool is_nan = std::isnan(v);
    const bool publish = !(is_nan && this->last_nan_);
    this->last_nan_ = is_nan;
    return publish;
  }

 protected:
  bool last_nan_{false};
};

}  // namespace esphome::dfrobot_mmwave::protocol
