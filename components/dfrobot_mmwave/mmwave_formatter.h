#pragma once

// Wire string builders. Pure C++17, no ESPHome includes, no heap.

#include <cstddef>
#include <cstdint>

namespace esphome::dfrobot_mmwave::protocol {

static constexpr size_t MAX_COMMAND_LEN = 63;  // without terminator

/// "%.3f" with trailing zeros and a trailing dot removed (6.000 -> "6", 0.025 -> "0.025").
/// Returns the number of characters written (0 if it does not fit).
size_t fmt_float(float value, char *out, size_t cap);

/// Builds "<cmd>[ <prefix>][ <arg>...]". Integers (int_mask bit set) are rounded and printed
/// with %u. Returns the length, or 0 if the result would exceed MAX_COMMAND_LEN or `cap`.
size_t fmt_command(char *out, size_t cap, const char *cmd, int prefix, const float *args, uint8_t nargs,
                   uint8_t int_mask);

/// Quantise down to a grid (SEN0395 range: the radar rounds down to n * 0.15 m).
float quantize_down(float value, float step);

}  // namespace esphome::dfrobot_mmwave::protocol
