#pragma once

// ESPHome hub: owns the UART, the presence pin and the entities, and adapts them to protocol::Engine.

#include <array>
#include <cmath>

#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#ifdef USE_BINARY_SENSOR
#include "esphome/components/binary_sensor/binary_sensor.h"
#endif
#ifdef USE_SENSOR
#include "esphome/components/sensor/sensor.h"
#endif
#ifdef USE_TEXT_SENSOR
#include "esphome/components/text_sensor/text_sensor.h"
#endif
#ifdef USE_NUMBER
#include "esphome/components/number/number.h"
#endif
#ifdef USE_SELECT
#include "esphome/components/select/select.h"
#endif
#ifdef USE_SWITCH
#include "esphome/components/switch/switch.h"
#endif

#include "mmwave_engine.h"

namespace esphome::dfrobot_mmwave {

enum class ButtonAction : uint8_t {
  BUTTON_ACTION_REFRESH,
  BUTTON_ACTION_RESTART,
  BUTTON_ACTION_FACTORY_RESET,
};

/// Publishes a float only when it changes (NaN == NaN for this purpose).
struct FloatDedup {
  bool next(float v) {
    bool is_nan = std::isnan(v);
    if (this->has_ && (is_nan ? this->last_nan_ : (!this->last_nan_ && v == this->last_))) {
      return false;
    }
    this->has_ = true;
    this->last_nan_ = is_nan;
    this->last_ = v;
    return true;
  }

 protected:
  float last_{0};
  bool has_{false};
  bool last_nan_{false};
};

class DfrobotMmwave final : public Component, public uart::UARTDevice, public protocol::EngineHost {
#ifdef USE_BINARY_SENSOR
  SUB_BINARY_SENSOR(occupancy)
  SUB_BINARY_SENSOR(uart_occupancy)
  SUB_BINARY_SENSOR(out_pin_occupancy)
  SUB_BINARY_SENSOR(link_ok)
#endif
#ifdef USE_SENSOR
  SUB_SENSOR(target_count)
  SUB_SENSOR(target_1_speed)
  SUB_SENSOR(target_1_energy)
#endif
#ifdef USE_TEXT_SENSOR
  SUB_TEXT_SENSOR(software_version)
  SUB_TEXT_SENSOR(hardware_version)
  SUB_TEXT_SENSOR(status)
  SUB_TEXT_SENSOR(last_error)
#endif
#ifdef USE_SWITCH
  SUB_SWITCH(running)
#endif

 public:
  explicit DfrobotMmwave(protocol::Model model) { this->cfg_.model = model; }

  void set_presence_pin(GPIOPin *pin) {
    this->presence_pin_ = pin;
    this->cfg_.has_pin = pin != nullptr;
  }
  void set_target_timeout(uint32_t ms) { this->cfg_.target_timeout_ms = ms; }

#ifdef USE_SENSOR
  void set_target_distance_sensor(uint8_t slot, sensor::Sensor *s) {
    if (slot < protocol::MAX_TARGETS)
      this->target_distance_sensors_[slot] = s;
  }
  void set_target_snr_sensor(uint8_t slot, sensor::Sensor *s) {
    if (slot < protocol::MAX_TARGETS)
      this->target_snr_sensors_[slot] = s;
  }
#endif
#ifdef USE_NUMBER
  void set_param_number(protocol::ParamId id, number::Number *n) { this->numbers_[protocol::idx(id)] = n; }
#endif
#ifdef USE_SELECT
  void set_param_select(protocol::ParamId id, select::Select *s) { this->selects_[protocol::idx(id)] = s; }
#endif
#ifdef USE_SWITCH
  void set_param_switch(protocol::ParamId id, switch_::Switch *s) { this->switches_[protocol::idx(id)] = s; }
#endif

  void setup() override;
  void loop() override;
  void dump_config() override;

  /// From an entity that has already published `value` optimistically.
  void control_param(protocol::ParamId id, float value);
  /// From an action: publishes `value` to the matching entity (if any), then requests it.
  void set_parameter(protocol::ParamId id, float value);
  void press(ButtonAction action);
  void request_refresh() { this->engine_.request_refresh(); }
  void request_restart() { this->engine_.request_restart(); }
  void request_factory_reset() { this->engine_.request_factory_reset(); }
  void request_running(bool on) { this->engine_.request_running(on); }

  // protocol::EngineHost
  void write(const char *data, size_t len) override;
  void on_param(protocol::ParamId id, float value, bool valid) override;
  void on_occupancy(protocol::Presence presence) override;
  void on_uart_occupancy(protocol::Presence presence) override;
  void on_pin_occupancy(bool present) override;
  void on_running(bool running) override;
  void on_link_ok(bool ok) override;
  void on_status(protocol::Status status) override;
  void on_last_error(const char *message) override;
  void on_version(bool hardware, const char *text) override;
  void on_targets(const protocol::TargetFrame &frame) override;
  void log(protocol::LogLevel level, const char *message) override;

 protected:
  protocol::EngineConfig cfg_{};
  protocol::Engine engine_;
  GPIOPin *presence_pin_{nullptr};

#ifdef USE_NUMBER
  std::array<number::Number *, protocol::PARAM_COUNT> numbers_{};
#endif
#ifdef USE_SELECT
  std::array<select::Select *, protocol::PARAM_COUNT> selects_{};
#endif
#ifdef USE_SWITCH
  std::array<switch_::Switch *, protocol::PARAM_COUNT> switches_{};
#endif
#ifdef USE_SENSOR
  std::array<sensor::Sensor *, protocol::MAX_TARGETS> target_distance_sensors_{};
  std::array<sensor::Sensor *, protocol::MAX_TARGETS> target_snr_sensors_{};
  std::array<FloatDedup, protocol::MAX_TARGETS> target_distance_dedup_{};
  std::array<FloatDedup, protocol::MAX_TARGETS> target_snr_dedup_{};
  FloatDedup target_count_dedup_{};
  FloatDedup target_speed_dedup_{};
  FloatDedup target_energy_dedup_{};
#endif
};

}  // namespace esphome::dfrobot_mmwave
