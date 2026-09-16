// ESPHome hub: feeds UART bytes, the pin and time into the engine and publishes its callbacks.

#include "dfrobot_mmwave.h"

#include <algorithm>

#include "esphome/core/application.h"
#include "esphome/core/controller_registry.h"
#include "esphome/core/log.h"

namespace esphome::dfrobot_mmwave {

static const char *const TAG = "dfrobot_mmwave";

using protocol::ParamId;

static constexpr size_t READ_CHUNK = 64;
// Upper bound on bytes handled per loop, so a flood of reports cannot stall the main loop. A SEN0395 with
// target reports at its shortest period sends about 10 kB/s; a loop runs at least every ~50 ms.
static constexpr size_t READ_BUDGET = 512;

#ifdef USE_BINARY_SENSOR
static void publish_presence(binary_sensor::BinarySensor *sens, protocol::Presence presence) {
  if (sens == nullptr)
    return;
  if (presence == protocol::Presence::PRESENCE_UNKNOWN) {
    if (sens->has_state())
      sens->invalidate_state();
  } else {
    sens->publish_state(presence == protocol::Presence::PRESENCE_DETECTED);
  }
}
#endif

void DfrobotMmwave::setup() {
  if (this->presence_pin_ != nullptr)
    this->presence_pin_->setup();
  this->engine_.configure(this->cfg_, this);
#if ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE
  this->engine_.set_log_level(protocol::LogLevel::LOG_LEVEL_VERBOSE);
#elif ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_DEBUG
  this->engine_.set_log_level(protocol::LogLevel::LOG_LEVEL_DEBUG);
#elif ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_INFO
  this->engine_.set_log_level(protocol::LogLevel::LOG_LEVEL_INFO);
#else
  this->engine_.set_log_level(protocol::LogLevel::LOG_LEVEL_WARN);
#endif
  const uint32_t now = millis();
  this->engine_.begin(now);
  if (this->presence_pin_ != nullptr)
    this->engine_.feed_pin(this->presence_pin_->digital_read(), now);
}

void DfrobotMmwave::loop() {
  const uint32_t now = App.get_loop_component_start_time();
  uint8_t buf[READ_CHUNK];
  size_t budget = READ_BUDGET;
  while (budget > 0) {
    size_t avail = this->available();
    if (avail == 0)
      break;
    size_t n = std::min({avail, READ_CHUNK, budget});
    if (!this->read_array(buf, n))
      break;
    for (size_t i = 0; i < n; i++)
      this->engine_.feed(buf[i], now);
    budget -= n;
  }
  if (this->presence_pin_ != nullptr)
    this->engine_.feed_pin(this->presence_pin_->digital_read(), now);
  this->engine_.loop(now);
}

void DfrobotMmwave::dump_config() {
  const protocol::Dialect &d = protocol::get_dialect(this->cfg_.model);
  ESP_LOGCONFIG(TAG,
                "DFRobot mmWave radar:\n"
                "  Model: %s\n"
                "  Target timeout: %" PRIu32 " ms\n"
                "  Status: %s",
                d.name, this->cfg_.target_timeout_ms, protocol::status_str(this->engine_.status()));
  LOG_PIN("  Presence pin: ", this->presence_pin_);
  if (this->engine_.get_mode_known()) {
    ESP_LOGCONFIG(TAG, "  Get commands: %s",
                  this->engine_.get_requires_stop() ? LOG_STR_LITERAL("only while stopped")
                                                    : LOG_STR_LITERAL("work while running"));
  }
  for (uint8_t i = 0; i < protocol::PARAM_COUNT; i++) {
    auto id = static_cast<ParamId>(i);
    if (protocol::param_support(this->cfg_.model, id) != protocol::Support::SUPPORT_PROBE)
      continue;
    if (!this->engine_.get_mode_known()) {
      ESP_LOGCONFIG(TAG, "  %s: not probed yet", protocol::param_name(id));
    } else {
      ESP_LOGCONFIG(TAG, "  %s: %s", protocol::param_name(id),
                    this->engine_.param_available(id) ? LOG_STR_LITERAL("supported")
                                                      : LOG_STR_LITERAL("not supported by this firmware"));
    }
  }
#ifdef USE_BINARY_SENSOR
  LOG_BINARY_SENSOR("  ", "Occupancy", this->occupancy_binary_sensor_);
  LOG_BINARY_SENSOR("  ", "UartOccupancy", this->uart_occupancy_binary_sensor_);
  LOG_BINARY_SENSOR("  ", "OutPinOccupancy", this->out_pin_occupancy_binary_sensor_);
  LOG_BINARY_SENSOR("  ", "LinkOk", this->link_ok_binary_sensor_);
#endif
#ifdef USE_SENSOR
  LOG_SENSOR("  ", "TargetCount", this->target_count_sensor_);
  for (auto *s : this->target_distance_sensors_)
    LOG_SENSOR("  ", "TargetDistance", s);
  for (auto *s : this->target_snr_sensors_)
    LOG_SENSOR("  ", "TargetSnr", s);
  LOG_SENSOR("  ", "Target1Speed", this->target_1_speed_sensor_);
  LOG_SENSOR("  ", "Target1Energy", this->target_1_energy_sensor_);
#endif
#ifdef USE_TEXT_SENSOR
  LOG_TEXT_SENSOR("  ", "SoftwareVersion", this->software_version_text_sensor_);
  LOG_TEXT_SENSOR("  ", "HardwareVersion", this->hardware_version_text_sensor_);
  LOG_TEXT_SENSOR("  ", "Status", this->status_text_sensor_);
  LOG_TEXT_SENSOR("  ", "LastError", this->last_error_text_sensor_);
#endif
#ifdef USE_NUMBER
  for (auto *n : this->numbers_) {
    if (n != nullptr)
      LOG_NUMBER("  ", "Setting", n);
  }
#endif
#ifdef USE_SELECT
  for (auto *s : this->selects_) {
    if (s != nullptr)
      LOG_SELECT("  ", "Setting", s);
  }
#endif
#ifdef USE_SWITCH
  LOG_SWITCH("  ", "Running", this->running_switch_);
  for (auto *s : this->switches_) {
    if (s != nullptr)
      LOG_SWITCH("  ", "Setting", s);
  }
#endif
}

// requests

void DfrobotMmwave::control_param(ParamId id, float value) {
  // same clock as loop(): entity calls happen inside another component's loop
  this->engine_.set_desired(id, value, App.get_loop_component_start_time());
}

void DfrobotMmwave::set_parameter(ParamId id, float value) {
  const uint8_t i = protocol::idx(id);
  if (i >= protocol::PARAM_COUNT)
    return;
#ifdef USE_NUMBER
  if (this->numbers_[i] != nullptr)
    this->numbers_[i]->publish_state(value);
#endif
#ifdef USE_SELECT
  if (this->selects_[i] != nullptr && value >= 0 && static_cast<size_t>(std::lround(value)) < this->selects_[i]->size())
    this->selects_[i]->publish_state(static_cast<size_t>(std::lround(value)));
#endif
#ifdef USE_SWITCH
  if (this->switches_[i] != nullptr)
    this->switches_[i]->publish_state(value >= 0.5f);
#endif
  this->control_param(id, value);
}

void DfrobotMmwave::press(ButtonAction action) {
  switch (action) {
    case ButtonAction::BUTTON_ACTION_REFRESH:
      this->engine_.request_refresh();
      break;
    case ButtonAction::BUTTON_ACTION_RESTART:
      this->engine_.request_restart();
      break;
    case ButtonAction::BUTTON_ACTION_FACTORY_RESET:
      this->engine_.request_factory_reset();
      break;
  }
}

// engine callbacks

void DfrobotMmwave::write(const char *data, size_t len) {
  this->write_array(reinterpret_cast<const uint8_t *>(data), len);
}

void DfrobotMmwave::on_param(ParamId id, float value, bool valid) {
  const uint8_t i = protocol::idx(id);
  if (i >= protocol::PARAM_COUNT)
    return;
#ifdef USE_NUMBER
  if (this->numbers_[i] != nullptr)
    this->numbers_[i]->publish_state(valid ? value : NAN);
#endif
#ifdef USE_SELECT
  if (select::Select *sel = this->selects_[i]; sel != nullptr) {
    auto index = static_cast<size_t>(std::lround(value));
    if (valid && value >= 0 && index < sel->size()) {
      sel->publish_state(index);
    } else if (!valid && sel->has_state()) {
      // Select has no invalidate_state(); clearing the flag and notifying reports it as unknown
      sel->set_has_state(false);
#ifdef USE_CONTROLLER_REGISTRY
      ControllerRegistry::notify_select_update(sel);
#endif
    }
  }
#endif
#ifdef USE_SWITCH
  // A switch has no "unknown" state in the native API, so one of the other work mode keeps its last value.
  if (this->switches_[i] != nullptr && valid)
    this->switches_[i]->publish_state(value >= 0.5f);
#endif
}

void DfrobotMmwave::on_occupancy(protocol::Presence presence) {
#ifdef USE_BINARY_SENSOR
  publish_presence(this->occupancy_binary_sensor_, presence);
#endif
}

void DfrobotMmwave::on_uart_occupancy(protocol::Presence presence) {
#ifdef USE_BINARY_SENSOR
  publish_presence(this->uart_occupancy_binary_sensor_, presence);
#endif
}

void DfrobotMmwave::on_pin_occupancy(bool present) {
#ifdef USE_BINARY_SENSOR
  if (this->out_pin_occupancy_binary_sensor_ != nullptr)
    this->out_pin_occupancy_binary_sensor_->publish_state(present);
#endif
}

void DfrobotMmwave::on_running(bool running) {
#ifdef USE_SWITCH
  if (this->running_switch_ != nullptr)
    this->running_switch_->publish_state(running);
#endif
}

void DfrobotMmwave::on_link_ok(bool ok) {
#ifdef USE_BINARY_SENSOR
  if (this->link_ok_binary_sensor_ != nullptr)
    this->link_ok_binary_sensor_->publish_state(ok);
#endif
}

void DfrobotMmwave::on_status(protocol::Status status) {
#ifdef USE_TEXT_SENSOR
  if (this->status_text_sensor_ != nullptr)
    this->status_text_sensor_->publish_state(protocol::status_str(status));
#endif
}

void DfrobotMmwave::on_last_error(const char *message) {
#ifdef USE_TEXT_SENSOR
  if (this->last_error_text_sensor_ != nullptr)
    this->last_error_text_sensor_->publish_state(message);
#endif
}

void DfrobotMmwave::on_version(bool hardware, const char *text) {
#ifdef USE_TEXT_SENSOR
  text_sensor::TextSensor *ts = hardware ? this->hardware_version_text_sensor_ : this->software_version_text_sensor_;
  if (ts != nullptr)
    ts->publish_state(text);
#endif
}

void DfrobotMmwave::on_targets(const protocol::TargetFrame &frame) {
#ifdef USE_SENSOR
  // NAN publishes "unknown": a C4001 in presence mode has no target data at all
  const float count = frame.available ? static_cast<float>(frame.count) : NAN;
  if (this->target_count_sensor_ != nullptr && this->target_count_dedup_.next(count))
    this->target_count_sensor_->publish_state(count);
  for (uint8_t i = 0; i < protocol::MAX_TARGETS; i++) {
    if (this->target_distance_sensors_[i] != nullptr && this->target_distance_dedup_[i].next(frame.distance[i]))
      this->target_distance_sensors_[i]->publish_state(frame.distance[i]);
    if (this->target_snr_sensors_[i] != nullptr && this->target_snr_dedup_[i].next(frame.snr[i]))
      this->target_snr_sensors_[i]->publish_state(frame.snr[i]);
  }
  if (this->target_1_speed_sensor_ != nullptr && this->target_speed_dedup_.next(frame.speed))
    this->target_1_speed_sensor_->publish_state(frame.speed);
  if (this->target_1_energy_sensor_ != nullptr && this->target_energy_dedup_.next(frame.energy))
    this->target_1_energy_sensor_->publish_state(frame.energy);
#endif
}

void DfrobotMmwave::log(protocol::LogLevel level, const char *message) {
  switch (level) {
    case protocol::LogLevel::LOG_LEVEL_ERROR:
      ESP_LOGE(TAG, "%s", message);
      break;
    case protocol::LogLevel::LOG_LEVEL_WARN:
      ESP_LOGW(TAG, "%s", message);
      break;
    case protocol::LogLevel::LOG_LEVEL_INFO:
      ESP_LOGI(TAG, "%s", message);
      break;
    case protocol::LogLevel::LOG_LEVEL_DEBUG:
      ESP_LOGD(TAG, "%s", message);
      break;
    default:
      ESP_LOGV(TAG, "%s", message);
      break;
  }
}

}  // namespace esphome::dfrobot_mmwave
