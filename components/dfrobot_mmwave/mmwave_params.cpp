// Parameter and command-group tables: the one place a setting is defined.

#include "mmwave_params.h"

namespace esphome::dfrobot_mmwave::protocol {

// clang-format off
static constexpr ParamSpec PARAM_SPECS[PARAM_COUNT] = {
  // id                                  name                      group                           idx int   modes               models              probe
  {ParamId::PARAM_ID_RANGE_MIN,          "min_range",              GroupId::GROUP_ID_RANGE,        0, false, MODE_MASK_BOTH,     MODEL_MASK_ALL,     0},
  {ParamId::PARAM_ID_RANGE_MAX,          "max_range",              GroupId::GROUP_ID_RANGE,        1, false, MODE_MASK_BOTH,     MODEL_MASK_ALL,     0},
  {ParamId::PARAM_ID_TRIG_RANGE,         "trigger_range",          GroupId::GROUP_ID_TRIG_RANGE,   0, false, MODE_MASK_PRESENCE, MODEL_MASK_C4001,   0},
  {ParamId::PARAM_ID_SENSITIVITY,        "sensitivity",            GroupId::GROUP_ID_SENSITIVITY,  0, true,  MODE_MASK_BOTH,     MODEL_MASK_SEN0395, 0},
  {ParamId::PARAM_ID_HOLD_SENS,          "hold_sensitivity",       GroupId::GROUP_ID_SENS_SPLIT,   0, true,  MODE_MASK_PRESENCE, MODEL_MASK_C4001,   0},
  {ParamId::PARAM_ID_TRIG_SENS,          "trigger_sensitivity",    GroupId::GROUP_ID_SENS_SPLIT,   1, true,  MODE_MASK_PRESENCE, MODEL_MASK_C4001,   0},
  {ParamId::PARAM_ID_LATENCY_ON,         "on_latency",             GroupId::GROUP_ID_LATENCY,      0, false, MODE_MASK_PRESENCE, MODEL_MASK_ALL,     0},
  {ParamId::PARAM_ID_LATENCY_OFF,        "off_latency",            GroupId::GROUP_ID_LATENCY,      1, false, MODE_MASK_PRESENCE, MODEL_MASK_ALL,     0},
  {ParamId::PARAM_ID_INHIBIT,            "inhibit_time",           GroupId::GROUP_ID_INHIBIT,      0, false, MODE_MASK_PRESENCE, MODEL_MASK_C4001,   0},
  {ParamId::PARAM_ID_MICRO_MOTION,       "speed_micro_motion",     GroupId::GROUP_ID_MICRO_MOTION, 0, true,  MODE_MASK_SPEED,    MODEL_MASK_C4001,   0},
  {ParamId::PARAM_ID_THR_FACTOR,         "speed_threshold_factor", GroupId::GROUP_ID_THR_FACTOR,   0, true,  MODE_MASK_SPEED,    MODEL_MASK_C4001,   0},
  {ParamId::PARAM_ID_LED,                "led",                    GroupId::GROUP_ID_LED,          0, true,  MODE_MASK_BOTH,     MODEL_MASK_ALL,     MODEL_MASK_C4001},
  {ParamId::PARAM_ID_UART_PRESENCE_EN,   "uart_presence_report",   GroupId::GROUP_ID_UART_OUT1,    0, true,  MODE_MASK_PRESENCE, MODEL_MASK_ALL,     MODEL_MASK_C4001},
  {ParamId::PARAM_ID_UART_TARGET_EN,     "uart_target_report",     GroupId::GROUP_ID_UART_OUT2,    0, true,  MODE_MASK_BOTH,     MODEL_MASK_SEN0395, 0},
  {ParamId::PARAM_ID_UART_REPORT_PERIOD, "uart_report_period",     GroupId::GROUP_ID_UART_OUT1,    1, false, MODE_MASK_PRESENCE, MODEL_MASK_ALL,     MODEL_MASK_C4001},
  {ParamId::PARAM_ID_WORK_MODE,          "work_mode",              GroupId::GROUP_ID_RUN_APP,      0, true,  MODE_MASK_BOTH,     MODEL_MASK_C4001,   0},
};

static constexpr GroupSpec GROUP_SPECS[GROUP_COUNT] = {
  // id                      set               get               nargs minresp prefix reply  int_mask
  {GroupId::GROUP_ID_RUN_APP,       "setRunApp",      nullptr,          1,    1,      -1,    false, 0b001},
  {GroupId::GROUP_ID_RANGE,         "setRange",       "getRange",       2,    2,      -1,    true,  0b000},
  {GroupId::GROUP_ID_TRIG_RANGE,    "setTrigRange",   "getTrigRange",   1,    1,      -1,    true,  0b000},
  {GroupId::GROUP_ID_SENSITIVITY,   "setSensitivity", "getSensitivity", 1,    1,      -1,    true,  0b001},
  {GroupId::GROUP_ID_SENS_SPLIT,    "setSensitivity", "getSensitivity", 2,    2,      -1,    true,  0b011},
  {GroupId::GROUP_ID_LATENCY,       "setLatency",     "getLatency",     2,    2,      -1,    true,  0b000},
  {GroupId::GROUP_ID_INHIBIT,       "setInhibit",     "getInhibit",     1,    1,      -1,    true,  0b000},
  {GroupId::GROUP_ID_MICRO_MOTION,  "setMicroMotion", "getMicroMotion", 1,    1,      -1,    true,  0b001},
  {GroupId::GROUP_ID_THR_FACTOR,    "setThrFactor",   "getThrFactor",   1,    1,      -1,    true,  0b001},
  {GroupId::GROUP_ID_LED,           "setLedMode",     "getLedMode",     1,    1,       1,    true,  0b001},
  {GroupId::GROUP_ID_UART_OUT1,     "setUartOutput",  "getUartOutput",  3,    1,       1,    true,  0b011},
  {GroupId::GROUP_ID_UART_OUT2,     "setUartOutput",  "getUartOutput",  3,    1,       2,    true,  0b011},
};
// clang-format on

const ParamSpec &param_spec(ParamId id) {
  auto i = idx(id);
  return PARAM_SPECS[i < PARAM_COUNT ? i : 0];
}

const GroupSpec &group_spec(GroupId id) {
  auto i = idx(id);
  return GROUP_SPECS[i < GROUP_COUNT ? i : 0];
}

const char *param_name(ParamId id) { return param_spec(id).name; }

bool param_readable(ParamId id) { return group_spec(param_spec(id).group).get_cmd != nullptr; }

Support param_support(Model m, ParamId id) {
  const ParamSpec &s = param_spec(id);
  uint8_t bit = model_bit(m);
  if ((s.models & bit) == 0)
    return Support::SUPPORT_NO;
  if ((s.probe_models & bit) != 0)
    return Support::SUPPORT_PROBE;
  return Support::SUPPORT_YES;
}

Support group_support(Model m, GroupId id) {
  Support best = Support::SUPPORT_NO;
  for (uint8_t i = 0; i < PARAM_COUNT; i++) {
    if (PARAM_SPECS[i].group != id)
      continue;
    Support s = param_support(m, PARAM_SPECS[i].id);
    if (s == Support::SUPPORT_YES)
      return Support::SUPPORT_YES;
    if (s == Support::SUPPORT_PROBE)
      best = Support::SUPPORT_PROBE;
  }
  return best;
}

uint8_t group_modes(Model m, GroupId id) {
  if (!get_dialect(m).has_modes)
    return MODE_MASK_BOTH;
  uint8_t modes = 0;
  for (uint8_t i = 0; i < PARAM_COUNT; i++) {
    if (PARAM_SPECS[i].group == id)
      modes |= PARAM_SPECS[i].modes;
  }
  return modes == 0 ? MODE_MASK_BOTH : modes;
}

Limits param_limits(Model m, ParamId id) {
  const Dialect &d = get_dialect(m);
  switch (id) {
    case ParamId::PARAM_ID_RANGE_MIN:
      return d.min_range;
    case ParamId::PARAM_ID_RANGE_MAX:
      return d.max_range;
    case ParamId::PARAM_ID_TRIG_RANGE:
      return d.trigger_range;
    case ParamId::PARAM_ID_SENSITIVITY:
    case ParamId::PARAM_ID_HOLD_SENS:
    case ParamId::PARAM_ID_TRIG_SENS:
      return {0.0f, 9.0f, 1.0f};
    case ParamId::PARAM_ID_LATENCY_ON:
      return d.on_latency;
    case ParamId::PARAM_ID_LATENCY_OFF:
      return d.off_latency;
    case ParamId::PARAM_ID_INHIBIT:
      return d.inhibit;
    case ParamId::PARAM_ID_THR_FACTOR:
      return {0.0f, 65535.0f, 1.0f};
    case ParamId::PARAM_ID_UART_REPORT_PERIOD:
      return d.uart_period;
    case ParamId::PARAM_ID_MICRO_MOTION:
    case ParamId::PARAM_ID_LED:
    case ParamId::PARAM_ID_UART_PRESENCE_EN:
    case ParamId::PARAM_ID_UART_TARGET_EN:
    case ParamId::PARAM_ID_WORK_MODE:
    default:
      return {0.0f, 1.0f, 1.0f};
  }
}

}  // namespace esphome::dfrobot_mmwave::protocol
