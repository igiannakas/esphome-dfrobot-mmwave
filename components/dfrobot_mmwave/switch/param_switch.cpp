// Switch entity for one radar setting, or for starting and stopping the radar.

#include "param_switch.h"

namespace esphome::dfrobot_mmwave {

void ParamSwitch::write_state(bool state) {
  this->publish_state(state);
  this->parent_->control_param(this->id_, state ? 1.0f : 0.0f);
}

void RunningSwitch::write_state(bool state) {
  this->publish_state(state);
  this->parent_->request_running(state);
}

}  // namespace esphome::dfrobot_mmwave
