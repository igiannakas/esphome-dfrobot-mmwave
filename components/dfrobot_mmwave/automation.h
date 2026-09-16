#pragma once

// Actions: refresh, restart, factory_reset and set_parameter.

#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "dfrobot_mmwave.h"

namespace esphome::dfrobot_mmwave {

template<typename... Ts> class RefreshAction final : public Action<Ts...>, public Parented<DfrobotMmwave> {
 public:
  void play(const Ts &...x) override { this->parent_->request_refresh(); }
};

template<typename... Ts> class RestartAction final : public Action<Ts...>, public Parented<DfrobotMmwave> {
 public:
  void play(const Ts &...x) override { this->parent_->request_restart(); }
};

template<typename... Ts> class FactoryResetAction final : public Action<Ts...>, public Parented<DfrobotMmwave> {
 public:
  void play(const Ts &...x) override { this->parent_->request_factory_reset(); }
};

template<typename... Ts> class SetParameterAction final : public Action<Ts...>, public Parented<DfrobotMmwave> {
 public:
  explicit SetParameterAction(protocol::ParamId id) : id_(id) {}
  TEMPLATABLE_VALUE(float, value)

  void play(const Ts &...x) override { this->parent_->set_parameter(this->id_, this->value_.value(x...)); }

 protected:
  protocol::ParamId id_;
};

}  // namespace esphome::dfrobot_mmwave
