// Select entity for one radar setting.

#include "param_select.h"

namespace esphome::dfrobot_mmwave {

void ParamSelect::control(size_t index) {
  this->publish_state(index);
  this->parent_->control_param(this->id_, static_cast<float>(index));
}

}  // namespace esphome::dfrobot_mmwave
