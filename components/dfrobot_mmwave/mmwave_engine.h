#pragma once

// Protocol engine: lifecycle FSM, command driving, transactions with read-back, sentence
// handling and occupancy from the OUT pin and the UART reports. Pure C++17, no ESPHome includes, no heap. The ESPHome
// hub (dfrobot_mmwave.h) is a thin adapter that feeds bytes/pin/time in and publishes callbacks.

#include <cstddef>
#include <cstdint>

#include "mmwave_command_queue.h"
#include "mmwave_dialect.h"
#include "mmwave_formatter.h"
#include "mmwave_line_reader.h"
#include "mmwave_params.h"
#include "mmwave_responses.h"

namespace esphome::dfrobot_mmwave::protocol {

enum class Status : uint8_t {
  STATUS_BOOT_WAIT,
  STATUS_PROBING,
  STATUS_READING,
  STATUS_APPLYING,
  STATUS_RUNNING,
  STATUS_STOPPED,
  STATUS_LINK_LOST,
  STATUS_UNSUPPORTED_FIRMWARE,
};
const char *status_str(Status s);

/// Occupancy as published: unknown while no presence source is live.
enum class Presence : uint8_t { PRESENCE_UNKNOWN, PRESENCE_CLEAR, PRESENCE_DETECTED };
enum class LogLevel : uint8_t {
  LOG_LEVEL_NONE = 0,
  LOG_LEVEL_ERROR = 1,
  LOG_LEVEL_WARN = 2,
  LOG_LEVEL_INFO = 3,
  LOG_LEVEL_DEBUG = 4,
  LOG_LEVEL_VERBOSE = 5
};

struct EngineConfig {
  Model model{Model::MODEL_SEN0609};
  bool has_pin{false};
  uint32_t target_timeout_ms{2000};
  uint32_t debounce_ms{750};  // quiet time after the last change before a transaction starts
};

static constexpr uint8_t MAX_TARGETS = 8;

struct TargetFrame {
  bool available{true};  // false: C4001 in presence mode, where there are no target reports
  uint8_t count{0};
  float distance[MAX_TARGETS]{};
  float snr[MAX_TARGETS]{};
  float speed{0};
  float energy{0};
};

class EngineHost {
 public:
  virtual ~EngineHost() = default;
  virtual void write(const char *data, size_t len) = 0;
  virtual void on_param(ParamId /*id*/, float /*value*/, bool /*valid*/) {}
  virtual void on_occupancy(Presence /*presence*/) {}
  virtual void on_uart_occupancy(Presence /*presence*/) {}
  virtual void on_pin_occupancy(bool /*present*/) {}
  virtual void on_running(bool /*running*/) {}
  virtual void on_link_ok(bool /*ok*/) {}
  virtual void on_status(Status /*status*/) {}
  virtual void on_last_error(const char * /*message*/) {}
  virtual void on_version(bool /*hardware*/, const char * /*text*/) {}
  virtual void on_targets(const TargetFrame & /*frame*/) {}
  virtual void log(LogLevel /*level*/, const char * /*message*/) {}
};

class Engine {
 public:
  static constexpr uint16_t TIMEOUT_MS = 1500;
  static constexpr uint16_t SAVE_TIMEOUT_MS = 3500;
  static constexpr uint16_t TAIL_MS = 200;
  static constexpr uint16_t VERSION_TAIL_MS = 500;
  static constexpr uint32_t BACKOFF_MIN_MS = 5000;
  static constexpr uint32_t BACKOFF_MAX_MS = 60000;
  static constexpr uint32_t MODE_CONFIRM_MS = 5000;
  static constexpr uint32_t MODE_SWITCH_WAIT_MS = 3000;
  static constexpr uint32_t LINK_TIMEOUT_MS = 30000;  // silence before a getSWV ping

  Engine() = default;
  Engine(const EngineConfig &config, EngineHost *host) { this->configure(config, host); }
  /// Must be called before begin().
  void configure(const EngineConfig &config, EngineHost *host) {
    this->cfg_ = config;
    this->dialect_ = &get_dialect(config.model);
    this->host_ = host;
  }

  void set_log_level(LogLevel level) { this->log_level_ = level; }

  void begin(uint32_t now);
  void feed(uint8_t byte, uint32_t now);
  void feed_pin(bool present, uint32_t now);
  void loop(uint32_t now);

  /// User request. The caller has already published `value` optimistically. Returns false (and
  /// republishes the confirmed value) if the request is rejected before reaching the radar.
  bool set_desired(ParamId id, float value, uint32_t now);
  void request_refresh() { this->refresh_req_ = true; }
  void request_restart() { this->restart_req_ = true; }
  void request_factory_reset() { this->factory_req_ = true; }
  void request_running(bool on) {
    this->run_req_ = true;
    this->run_req_on_ = on;
  }

  // introspection (dump_config, tests)
  const EngineConfig &config() const { return this->cfg_; }
  const Dialect &dialect() const { return *this->dialect_; }
  Status status() const { return this->status_; }
  bool param_available(ParamId id) const;
  const ParamState &param(ParamId id) const { return this->params_[id]; }
  bool get_requires_stop() const { return this->get_requires_stop_; }
  bool get_mode_known() const { return this->get_mode_known_; }
  WorkMode work_mode() const { return this->work_mode_; }
  bool busy() const { return this->op_ != Op::OP_NONE; }
  /// Changes not yet confirmed by the radar (queued, or in a running transaction).
  bool config_pending() const;
  bool running() const { return this->running_; }
  uint32_t save_count() const { return this->save_count_; }
  uint32_t transaction_count() const { return this->txn_count_; }
  uint32_t overflow_count() const { return this->reader_.overflow_count(); }
  Presence occupancy() const { return this->occupancy_; }
  /// How long the UART presence source stays live without a new report.
  uint32_t uart_presence_timeout_ms() const;
  const char *last_error() const { return this->last_error_; }

 protected:
  enum class Life : uint8_t { LIFE_BOOT_WAIT, LIFE_PROBE, LIFE_BACKOFF, LIFE_RUNNING };
  enum class Op : uint8_t {
    OP_NONE,
    OP_PROBE,
    OP_BOOT_READ,
    OP_READ,
    OP_TXN,
    OP_FACTORY_RESET,
    OP_RUN_CTRL,
    OP_RESTART,
    OP_PING,         // one getSWV to tell a quiet radar from a dead link
    OP_MODE_SWITCH,  // sensorStop, setRunApp, wait for the new app; an identify-and-read follows
  };
  enum class Result : uint8_t {
    RESULT_OK,
    RESULT_ERROR,
    RESULT_UNRECOGNIZED,
    RESULT_TIMEOUT,
    RESULT_UNSAVED,
    RESULT_NOT_STOPPED,
    RESULT_MALFORMED,  // a Response whose values could not all be parsed
  };

  struct InFlight {
    bool active{false};
    Command cmd{};
    uint32_t sent_ms{0};
    uint16_t timeout_ms{0};
    bool no_reply{false};  // completes OK when the timeout expires
    bool tail{false};      // terminal line seen, waiting briefly for Done/prompt
    uint32_t tail_start{0};
    uint16_t tail_ms{0};
    Result tail_result{Result::RESULT_OK};
    uint8_t nvals{0};
    float vals[MAX_RESPONSE_VALUES]{};
    bool got_values{false};
  };

  // bytes / lines
  void on_line_(const char *line, size_t len);
  void on_prompt_();
  void on_sentence_(const char *line, const Reply &r);
  void on_reply_(const char *line, const Reply &r);

  // command driving
  bool enqueue_(CmdId id, GroupId group = GroupId::GROUP_ID_COUNT, uint8_t flags = CMD_FLAG_NONE);
  bool enqueue_cmd_(const Command &c);
  void drive_queue_();
  void send_(Command c);
  void write_line_(const char *text);
  void complete_(Result result);
  void start_tail_(Result result, uint16_t ms);
  void on_done_(const Command &c, Result result);
  void on_marker_(const Command &c);
  void op_failed_(const Command &c, const char *why);

  // lifecycle / operations
  void schedule_();
  void start_probe_();
  void start_identify_and_read_();
  void start_read_(bool standing);
  void start_txn_();
  uint16_t couple_ranges_();
  void start_factory_reset_();
  void start_mode_switch_();
  void finish_mode_switch_();
  void finish_read_(bool boot);
  void finish_txn_();
  void finish_failure_();
  void begin_op_errors_();
  void clear_error_if_clean_();
  void confirm_mode_();
  void enter_backoff_(const char *why);
  void reset_radar_state_();
  void enqueue_gets_(uint16_t group_mask);
  WorkMode read_mode_() const;
  bool param_in_current_mode_(ParamId id) const;
  bool param_in_mode_(ParamId id, WorkMode mode) const;
  void set_mode_error_(ParamId id, WorkMode mode);
  void check_report_after_recovery_();
  uint16_t readable_groups_() const;
  bool group_usable_(GroupId g) const;
  bool build_set_(GroupId g, Command &c);
  bool apply_response_(GroupId g, const float *vals, uint8_t nvals);
  float value_for_(ParamId id) const;  // snapshot value, else reported, else NAN
  void publish_param_(ParamId id);

  // sentences / presence
  void handle_targets_(const char *line, const Reply &r);
  void set_work_mode_(WorkMode m);
  void set_uart_presence_(bool present, bool from_report);
  void drop_uart_presence_();
  void check_uart_presence_stale_();
  void update_occupancy_();
  void check_targets_decay_();
  void check_link_();
  void publish_idle_targets_();

  // state publishing
  void set_status_(Status s);
  void set_running_(bool running);
  void set_link_ok_(bool ok);
  void set_error_(const char *fmt, ...) __attribute__((format(printf, 2, 3)));
  void logf_(LogLevel level, const char *fmt, ...) __attribute__((format(printf, 3, 4)));

  EngineConfig cfg_;
  const Dialect *dialect_{&DIALECTS[0]};
  EngineHost *host_{nullptr};
  LogLevel log_level_{LogLevel::LOG_LEVEL_DEBUG};

  LineReader reader_;
  CommandQueue queue_;
  ParamStore params_;
  InFlight inflight_;
  uint32_t now_{0};
  uint32_t last_tx_ms_{0};
  uint16_t gap_ms_{0};
  char last_cmd_[MAX_COMMAND_LEN + 1]{};

  // lifecycle
  Life life_{Life::LIFE_BOOT_WAIT};
  Op op_{Op::OP_NONE};
  Status status_{Status::STATUS_BOOT_WAIT};
  bool status_published_{false};
  uint32_t boot_start_ms_{0};
  uint32_t backoff_start_ms_{0};
  uint32_t backoff_wait_ms_{0};
  uint32_t backoff_ms_{BACKOFF_MIN_MS};
  bool ever_running_{false};
  bool probe_answered_{false};
  bool unsupported_firmware_{false};
  bool get_requires_stop_{false};
  bool get_mode_known_{false};
  bool first_read_phase_{false};
  bool prompt_checked_{false};
  bool tag_warned_{false};
  bool report_mode_logged_{false};  // "UART report mode is periodic/passive" logged once per boot

  // requests
  bool refresh_req_{false};
  bool restart_req_{false};
  bool factory_req_{false};
  bool run_req_{false};
  bool run_req_on_{true};
  bool mode_read_req_{false};
  uint32_t last_change_ms_{0};

  // operation state
  uint16_t read_ok_mask_{0};
  uint16_t read_mask_{0};  // groups requested in the current read/readback
  uint16_t set_ok_mask_{0};
  uint16_t set_fail_mask_{0};
  bool op_error_{false};         // set_error_() was called during the current operation
  bool save_error_seen_{false};  // saveConfig answered Error in the current transaction
  bool error_published_{false};  // last_error has been published at least once
  bool stopped_by_op_{false};
  bool restart_after_op_{true};
  bool failing_{false};
  bool snapshot_has_[PARAM_COUNT]{};
  float snapshot_[PARAM_COUNT]{};
  float prev_reported_[PARAM_COUNT]{};
  bool prev_valid_[PARAM_COUNT]{};
  char fail_reason_[48]{};
  uint32_t save_count_{0};
  uint32_t txn_count_{0};

  // radar state
  bool running_{false};
  bool running_known_{false};
  bool user_stopped_{false};
  uint32_t last_rx_ms_{0};
  uint32_t last_sentence_ms_{0};
  bool sentence_seen_{false};
  WorkMode work_mode_{WorkMode::WORK_MODE_UNKNOWN};
  bool mode_confirm_pending_{false};
  bool mode_confirm_has_request_{false};
  float mode_requested_{0};
  uint32_t mode_confirm_start_ms_{0};
  uint32_t mode_sentences_since_start_{0};
  bool link_ok_{false};
  bool link_ok_published_{false};
  bool quiet_logged_{false};
  bool report_passive_{false};    // radar reported a passive UART output period: no reports at all
  bool recover_start_{false};     // link was lost after running: start the radar after the recovery read
  bool stop_unconfirmed_{false};  // a sensorStop timed out, so the radar may be stopped
  bool report_check_armed_{false};
  uint32_t report_check_deadline_ms_{0};
  bool mode_switch_pending_{false};  // an identify-and-read after setRunApp is still running
  bool mode_switch_rejected_{false};  // setRunApp was refused; the error is already set
  WorkMode mode_switch_target_{WorkMode::WORK_MODE_UNKNOWN};
  bool mode_wait_active_{false};
  bool mode_wait_seen_{false};
  uint32_t mode_wait_start_ms_{0};
  WorkMode last_read_mode_{WorkMode::WORK_MODE_UNKNOWN};

  // presence: occupancy is on when any live source reports presence
  bool pin_live_{false};
  bool pin_present_{false};
  bool uart_live_{false};  // a report arrived within uart_presence_timeout_ms()
  bool uart_present_{false};
  uint32_t last_uart_report_ms_{0};
  Presence occupancy_{Presence::PRESENCE_UNKNOWN};

  // targets
  uint8_t cycle_n_{0};
  uint8_t cycle_filled_{0};
  TargetFrame cycle_{};
  uint32_t last_target_ms_{0};
  bool targets_zero_published_{false};

  char last_error_[96]{};
};

}  // namespace esphome::dfrobot_mmwave::protocol
