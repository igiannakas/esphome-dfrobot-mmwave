#pragma once

// Switch entity for one radar setting, or for starting and stopping the radar.

#include "esphome/components/switch/switch.h"
#include "../dfrobot_mmwave.h"

namespace esphome::dfrobot_mmwave {

class ParamSwitch final : public switch_::Switch, public Parented<DfrobotMmwave> {
 public:
  explicit ParamSwitch(protocol::ParamId id) : id_(id) {}

 protected:
  void write_state(bool state) override;

  protocol::ParamId id_;
};

class RunningSwitch final : public switch_::Switch, public Parented<DfrobotMmwave> {
 protected:
  void write_state(bool state) override;
};

}  // namespace esphome::dfrobot_mmwave
