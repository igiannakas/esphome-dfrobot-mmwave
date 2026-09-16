#pragma once

// Buttons that trigger one hub action (apply, refresh, restart, factory reset).

#include "esphome/components/button/button.h"
#include "../dfrobot_mmwave.h"

namespace esphome::dfrobot_mmwave {

class ActionButton final : public button::Button, public Parented<DfrobotMmwave> {
 public:
  explicit ActionButton(ButtonAction action) : action_(action) {}

 protected:
  void press_action() override { this->parent_->press(this->action_); }

  ButtonAction action_;
};

}  // namespace esphome::dfrobot_mmwave
