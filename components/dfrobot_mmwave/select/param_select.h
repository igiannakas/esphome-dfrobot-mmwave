#pragma once

// Select entity for one radar setting (option index is the radar value).

#include "esphome/components/select/select.h"
#include "../dfrobot_mmwave.h"

namespace esphome::dfrobot_mmwave {

class ParamSelect final : public select::Select, public Parented<DfrobotMmwave> {
 public:
  explicit ParamSelect(protocol::ParamId id) : id_(id) {}

 protected:
  void control(size_t index) override;

  protocol::ParamId id_;
};

}  // namespace esphome::dfrobot_mmwave
