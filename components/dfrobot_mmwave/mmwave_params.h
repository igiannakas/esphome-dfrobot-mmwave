#pragma once

// Parameter and command-group registry. Pure C++17, no ESPHome includes.

#include <cstdint>

#include "mmwave_dialect.h"

namespace esphome::dfrobot_mmwave::protocol {

// Order is significant only for COUNT; the Python side mirrors these names (ParamId.X).
enum class ParamId : uint8_t {
  PARAM_ID_RANGE_MIN,
  PARAM_ID_RANGE_MAX,
  PARAM_ID_TRIG_RANGE,
  PARAM_ID_SENSITIVITY,
  PARAM_ID_HOLD_SENS,
  PARAM_ID_TRIG_SENS,
  PARAM_ID_LATENCY_ON,
  PARAM_ID_LATENCY_OFF,
  PARAM_ID_INHIBIT,
  PARAM_ID_MICRO_MOTION,
  PARAM_ID_THR_FACTOR,
  PARAM_ID_LED,
  PARAM_ID_UART_PRESENCE_EN,
  PARAM_ID_UART_TARGET_EN,
  PARAM_ID_UART_REPORT_PERIOD,
  PARAM_ID_WORK_MODE,
  PARAM_ID_COUNT
};
static constexpr uint8_t PARAM_COUNT = static_cast<uint8_t>(ParamId::PARAM_ID_COUNT);

// A group is one set command plus one get command. Enum order is the order of the set commands inside a
// transaction. RUN_APP is never part of one: a work-mode change runs as its own operation.
enum class GroupId : uint8_t {
  GROUP_ID_RUN_APP,
  GROUP_ID_RANGE,
  GROUP_ID_TRIG_RANGE,
  GROUP_ID_SENSITIVITY,
  GROUP_ID_SENS_SPLIT,
  GROUP_ID_LATENCY,
  GROUP_ID_INHIBIT,
  GROUP_ID_MICRO_MOTION,
  GROUP_ID_THR_FACTOR,
  GROUP_ID_LED,
  GROUP_ID_UART_OUT1,
  GROUP_ID_UART_OUT2,
  GROUP_ID_COUNT
};
static constexpr uint8_t GROUP_COUNT = static_cast<uint8_t>(GroupId::GROUP_ID_COUNT);

static constexpr uint8_t MODE_MASK_PRESENCE = 1;
static constexpr uint8_t MODE_MASK_SPEED = 2;
static constexpr uint8_t MODE_MASK_BOTH = 3;
static constexpr uint8_t MODEL_MASK_SEN0395 = 1;
static constexpr uint8_t MODEL_MASK_SEN0609 = 2;
static constexpr uint8_t MODEL_MASK_SEN0610 = 4;
static constexpr uint8_t MODEL_MASK_C4001 = 6;
static constexpr uint8_t MODEL_MASK_ALL = 7;

inline uint8_t model_bit(Model m) { return static_cast<uint8_t>(1u << static_cast<uint8_t>(m)); }
inline uint8_t idx(ParamId p) { return static_cast<uint8_t>(p); }
inline uint8_t idx(GroupId g) { return static_cast<uint8_t>(g); }

enum class Support : uint8_t { SUPPORT_NO, SUPPORT_YES, SUPPORT_PROBE };

struct ParamSpec {
  ParamId id;
  const char *name;
  GroupId group;
  uint8_t index;  // position inside the group's argument tuple
  bool is_integer;
  uint8_t modes;         // ModeMask (C4001 only; SEN0395 is always presence)
  uint8_t models;        // ModelMask: models that have it at all
  uint8_t probe_models;  // ModelMask: models where it only exists if the boot probe answers
};

struct GroupSpec {
  GroupId id;
  const char *set_cmd;  // nullptr: not settable
  const char *get_cmd;  // nullptr: not readable
  uint8_t nargs;        // arguments of the set command (after the fixed prefix)
  uint8_t min_response_args;
  int8_t prefix;       // fixed first argument ("setLedMode 1 x"), -1 = none
  bool set_has_reply;  // false for setRunApp
  uint8_t int_mask;    // bit i set: argument i is formatted as an integer
};

// setUartOutput report mode: 0 = periodic only, 1 = a line on every change plus the periodic one.
// The component always writes 1; a period above PASSIVE_PERIOD_MIN means passive (no reports at all).
static constexpr uint8_t REPORT_MODE_ON_CHANGE = 1;
static constexpr float PASSIVE_PERIOD_MIN = 1500.0f;
static constexpr float SENSITIVITY_UNCHANGED = 255.0f;

const ParamSpec &param_spec(ParamId id);
const GroupSpec &group_spec(GroupId id);
const char *param_name(ParamId id);
Limits param_limits(Model m, ParamId id);
/// Rounds a value to as many decimals as `step` has (0.1 -> 1, 0.025 -> 3), not onto the step grid.
float round_to_step_decimals(float value, float step);
Support param_support(Model m, ParamId id);
Support group_support(Model m, GroupId id);
/// Union of the member modes of a group on this model (MODE_MASK_BOTH on SEN0395).
uint8_t group_modes(Model m, GroupId id);
/// True if the parameter can be read back with a get command.
bool param_readable(ParamId id);

}  // namespace esphome::dfrobot_mmwave::protocol
