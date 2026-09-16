// Prints param_limits() for every model and number setting, one "model key lo hi step" line each.
#include <cstdio>

#include "../mmwave_params.h"

using namespace esphome::dfrobot_mmwave::protocol;

int main() {
  static constexpr ParamId NUMBERS[] = {
      ParamId::PARAM_ID_RANGE_MIN,   ParamId::PARAM_ID_RANGE_MAX,          ParamId::PARAM_ID_TRIG_RANGE,
      ParamId::PARAM_ID_SENSITIVITY, ParamId::PARAM_ID_HOLD_SENS,          ParamId::PARAM_ID_TRIG_SENS,
      ParamId::PARAM_ID_LATENCY_ON,  ParamId::PARAM_ID_LATENCY_OFF,        ParamId::PARAM_ID_INHIBIT,
      ParamId::PARAM_ID_THR_FACTOR,  ParamId::PARAM_ID_UART_REPORT_PERIOD,
  };
  for (uint8_t m = 0; m < MODEL_COUNT; m++) {
    auto model = static_cast<Model>(m);
    for (ParamId id : NUMBERS) {
      if (param_support(model, id) == Support::SUPPORT_NO)
        continue;
      Limits l = param_limits(model, id);
      std::printf("%s %s %g %g %g\n", get_dialect(model).name, param_name(id), l.lo, l.hi, l.step);
    }
  }
  return 0;
}
