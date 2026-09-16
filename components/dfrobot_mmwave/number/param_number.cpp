// Number entity for one radar setting.

#include "param_number.h"

namespace esphome::dfrobot_mmwave {

void ParamNumber::control(float value) {
  this->publish_state(value);  // show the request now; the hub confirms by read-back or reverts
  this->parent_->control_param(this->id_, value);
}

}  // namespace esphome::dfrobot_mmwave
