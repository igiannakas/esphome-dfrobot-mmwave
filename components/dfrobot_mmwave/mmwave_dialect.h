#pragma once

// Per-model protocol differences and setting limits. Plain C++17 without ESPHome headers, so the host tests
// can compile it.

#include <cstdint>

namespace esphome::dfrobot_mmwave::protocol {

enum class Model : uint8_t { MODEL_SEN0395 = 0, MODEL_SEN0609 = 1, MODEL_SEN0610 = 2 };
static constexpr uint8_t MODEL_COUNT = 3;

/// C4001 working mode. SEN0395 is always PRESENCE.
enum class WorkMode : uint8_t { WORK_MODE_PRESENCE = 0, WORK_MODE_SPEED = 1, WORK_MODE_UNKNOWN = 2 };

struct Limits {
  float lo;
  float hi;
  float step;
};

// Everything that differs between the radars. One entry per model. Setting limits are the ones the
// firmware keeps (verified by read-back on a SEN0609 and a SEN0395) and are mirrored in NUMBER_LIMITS in
// __init__.py, which host_tests/check_limits_sync.py enforces. min_range's ceiling and max_range's floor
// are one step inside the radar's own so the sliders can never ask for an empty window (min == max).
struct Dialect {
  Model model;
  const char *name;
  const char *prompt;        // printed without newline
  const char *presence_tag;  // 5-char sentence tag after '$'
  const char *target_tag;
  uint32_t default_baud;
  uint16_t min_gap_ms;     // minimum time between two sends
  uint16_t save_gap_ms;    // minimum time after saveConfig before the next send
  uint16_t reset_wait_ms;  // silence after resetSystem before the boot delay starts
  uint16_t boot_delay_ms;  // power-on time before the radar answers commands
  bool start_resume_arg;   // "sensorStart 1" (C4001) vs "sensorStart"
  bool has_out_pin;
  bool has_modes;         // setRunApp / $DFHPD vs $DFDMD
  bool has_multi_target;  // $JYRPO bursts of up to 8 targets
  bool range_quantized;   // SEN0395 rounds ranges down to its step
  Limits min_range;
  Limits max_range;
  Limits trigger_range;
  Limits on_latency;
  Limits off_latency;
  Limits inhibit;
  Limits uart_period;
};

// clang-format off
inline constexpr Dialect DIALECTS[MODEL_COUNT] = {
    {Model::MODEL_SEN0395, "SEN0395", "leapMMW:/> ", "JYBSS", "JYRPO", 115200, 1000, 3000, 1500, 3000,
     false, true, false, true, true,
     /* min_range */ {0.0f, 9.3f, 0.15f},   /* max_range */ {0.15f, 9.45f, 0.15f}, /* trigger_range */ {0.0f, 0.0f, 0.1f},
     /* on_latency */ {0.0f, 100.0f, 0.025f}, /* off_latency */ {0.5f, 1500.0f, 0.1f}, /* inhibit */ {0.0f, 0.0f, 0.1f},
     /* uart_period */ {0.025f, 1500.0f, 0.025f}},
    // C4001 firmware: min range floor 0.3 m, max/trigger range floor 2.4 m on both models (measured), on-latency ceiling 2 s, off-latency
    // floor 2 s, inhibit 0.3..60 s, report period floor 0.2 s; the speed app reports ranges of 0..26 m.
    {Model::MODEL_SEN0609, "SEN0609", "DFRobot:/> ", "DFHPD", "DFDMD", 9600, 200, 800, 1500, 3000,
     true, true, true, false, false,
     /* min_range */ {0.3f, 25.9f, 0.1f}, /* max_range */ {2.4f, 26.0f, 0.1f}, /* trigger_range */ {2.4f, 25.0f, 0.1f},
     /* on_latency */ {0.0f, 2.0f, 0.01f}, /* off_latency */ {2.0f, 1500.0f, 0.5f}, /* inhibit */ {0.3f, 60.0f, 0.1f},
     /* uart_period */ {0.2f, 1500.0f, 0.025f}},
    {Model::MODEL_SEN0610, "SEN0610", "DFRobot:/> ", "DFHPD", "DFDMD", 9600, 200, 800, 1500, 3000,
     true, false, true, false, false,
     /* min_range */ {0.3f, 11.9f, 0.1f}, /* max_range */ {2.4f, 12.0f, 0.1f}, /* trigger_range */ {2.4f, 12.0f, 0.1f},
     /* on_latency */ {0.0f, 2.0f, 0.01f}, /* off_latency */ {2.0f, 1500.0f, 0.5f}, /* inhibit */ {0.3f, 60.0f, 0.1f},
     /* uart_period */ {0.2f, 1500.0f, 0.025f}},
};
// clang-format on

inline const Dialect &get_dialect(Model m) {
  auto i = static_cast<uint8_t>(m);
  return DIALECTS[i < MODEL_COUNT ? i : 0];
}

}  // namespace esphome::dfrobot_mmwave::protocol
