#pragma once

// Number entity for one radar setting.

#include "esphome/components/number/number.h"
#include "../dfrobot_mmwave.h"

namespace esphome::dfrobot_mmwave {

class ParamNumber final : public number::Number, public Parented<DfrobotMmwave> {
 public:
  explicit ParamNumber(protocol::ParamId id) : id_(id) {}

 protected:
  void control(float value) override;

  protocol::ParamId id_;
};

}  // namespace esphome::dfrobot_mmwave
