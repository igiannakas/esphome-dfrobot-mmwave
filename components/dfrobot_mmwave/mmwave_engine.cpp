// Protocol engine implementation; see mmwave_engine.h for what it owns.

#include "mmwave_engine.h"

#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>

namespace esphome::dfrobot_mmwave::protocol {

static uint16_t gbit(GroupId g) { return static_cast<uint16_t>(1u << idx(g)); }
static bool finite(float v) { return std::isfinite(v); }

const char *status_str(Status s) {
  switch (s) {
    case Status::STATUS_BOOT_WAIT:
      return "boot_wait";
    case Status::STATUS_PROBING:
      return "probing";
    case Status::STATUS_READING:
      return "reading";
    case Status::STATUS_APPLYING:
      return "applying";
    case Status::STATUS_RUNNING:
      return "running";
    case Status::STATUS_STOPPED:
      return "stopped";
    case Status::STATUS_LINK_LOST:
      return "link_lost";
    case Status::STATUS_UNSUPPORTED_FIRMWARE:
      return "unsupported_firmware";
    default:
      return "unknown";
  }
}

static const char *cmd_str(CmdId id) {
  switch (id) {
    case CmdId::CMD_ID_GET_SWV:
      return "getSWV";
    case CmdId::CMD_ID_GET_HWV:
      return "getHWV";
    case CmdId::CMD_ID_GET_RUN_APP:
      return "getRunApp";
    case CmdId::CMD_ID_GET_GROUP:
      return "get";
    case CmdId::CMD_ID_SET_GROUP:
      return "set";
    case CmdId::CMD_ID_STOP:
      return "sensorStop";
    case CmdId::CMD_ID_START:
      return "sensorStart";
    case CmdId::CMD_ID_SAVE:
      return "saveConfig";
    case CmdId::CMD_ID_RESET_CFG:
      return "resetCfg";
    case CmdId::CMD_ID_RESET_SYSTEM:
      return "resetSystem";
    default:
      return "marker";
  }
}

// logging / publishing helpers

void Engine::logf_(LogLevel level, const char *fmt, ...) {
  if (this->host_ == nullptr || level > this->log_level_)
    return;
  char buf[192];
  va_list args;
  va_start(args, fmt);
  std::vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  this->host_->log(level, buf);
}

void Engine::set_error_(const char *fmt, ...) {
  this->op_error_ = true;
  va_list args;
  va_start(args, fmt);
  std::vsnprintf(this->last_error_, sizeof(this->last_error_), fmt, args);
  va_end(args);
  if (this->host_ == nullptr)
    return;
  if (this->log_level_ >= LogLevel::LOG_LEVEL_WARN)
    this->host_->log(LogLevel::LOG_LEVEL_WARN, this->last_error_);
  this->error_published_ = true;
  this->host_->on_last_error(this->last_error_);
}

void Engine::begin_op_errors_() {
  this->op_error_ = false;
  this->save_error_seen_ = false;
}

// Called at the end of a read or transaction: a clean operation clears a stale last_error.
void Engine::clear_error_if_clean_() {
  if (this->op_error_ || (this->error_published_ && this->last_error_[0] == '\0'))
    return;
  this->last_error_[0] = '\0';
  this->error_published_ = true;
  if (this->host_ != nullptr)
    this->host_->on_last_error(this->last_error_);
}

void Engine::set_status_(Status s) {
  if (this->status_published_ && s == this->status_)
    return;
  this->status_ = s;
  this->status_published_ = true;
  this->logf_(LogLevel::LOG_LEVEL_DEBUG, "status: %s", status_str(s));
  this->host_->on_status(s);
}

void Engine::set_running_(bool running) {
  bool changed = !this->running_known_ || running != this->running_;
  this->running_known_ = true;
  this->running_ = running;
  if (changed)
    this->host_->on_running(running);
}

void Engine::set_link_ok_(bool ok) {
  if (this->link_ok_published_ && ok == this->link_ok_)
    return;
  this->link_ok_ = ok;
  this->link_ok_published_ = true;
  this->host_->on_link_ok(ok);
}

bool Engine::config_pending() const {
  return this->params_.any_dirty() || this->op_ == Op::OP_TXN || this->op_ == Op::OP_FACTORY_RESET ||
         this->op_ == Op::OP_MODE_SWITCH || this->mode_switch_pending_;
}

// A setting of the other work mode is published as unavailable: the radar has no current value for it.
void Engine::publish_param_(ParamId id) {
  if (param_support(this->cfg_.model, id) == Support::SUPPORT_NO)
    return;
  const ParamState &s = this->params_[id];
  // the radar can keep more decimals than the setting's step (a C4001 inhibit time read back as 3.101)
  const float shown = round_to_step_decimals(s.reported, param_limits(this->cfg_.model, id).step);
  this->host_->on_param(id, shown, s.reported_valid && this->param_in_current_mode_(id));
}

bool Engine::param_in_mode_(ParamId id, WorkMode mode) const {
  if (!this->dialect_->has_modes || mode == WorkMode::WORK_MODE_UNKNOWN)
    return true;
  uint8_t bit = mode == WorkMode::WORK_MODE_SPEED ? MODE_MASK_SPEED : MODE_MASK_PRESENCE;
  return (param_spec(id).modes & bit) != 0;
}

void Engine::set_mode_error_(ParamId id, WorkMode mode) {
  if (param_spec(id).group == GroupId::GROUP_ID_UART_OUT1) {
    // C4001 firmware: getUartOutput/setUartOutput answer Error in the speed app
    this->set_error_("uart output settings are presence-mode only");
  } else {
    this->set_error_("%s is only available in %s mode", param_name(id),
                     mode == WorkMode::WORK_MODE_SPEED ? "presence" : "speed_and_distance");
  }
}

bool Engine::param_in_current_mode_(ParamId id) const {
  if (!this->dialect_->has_modes || this->work_mode_ == WorkMode::WORK_MODE_UNKNOWN)
    return true;
  uint8_t bit = this->work_mode_ == WorkMode::WORK_MODE_SPEED ? MODE_MASK_SPEED : MODE_MASK_PRESENCE;
  return (param_spec(id).modes & bit) != 0;
}

// public API

void Engine::begin(uint32_t now) {
  this->now_ = now;
  this->boot_start_ms_ = now;
  this->last_rx_ms_ = now;
  this->last_target_ms_ = now;
  this->life_ = Life::LIFE_BOOT_WAIT;
  this->set_status_(Status::STATUS_BOOT_WAIT);
}

void Engine::feed(uint8_t byte, uint32_t now) {
  this->now_ = now;
  switch (this->reader_.feed(byte)) {
    case LineReader::Event::EVENT_LINE:
      this->on_line_(this->reader_.line(), this->reader_.length());
      break;
    case LineReader::Event::EVENT_PROMPT:
      this->on_prompt_();
      break;
    case LineReader::Event::EVENT_OVERFLOW:
      this->logf_(LogLevel::LOG_LEVEL_DEBUG, "RX line too long, discarded (%u so far)",
                  static_cast<unsigned>(this->reader_.overflow_count()));
      break;
    default:
      break;
  }
}

void Engine::feed_pin(bool present, uint32_t now) {
  this->now_ = now;
  if (!this->cfg_.has_pin || (this->pin_live_ && present == this->pin_present_))
    return;
  // the OUT pin is a push-pull output of the radar, so no debounce
  this->pin_live_ = true;
  this->pin_present_ = present;
  this->host_->on_pin_occupancy(present);
  this->update_occupancy_();
}

bool Engine::param_available(ParamId id) const {
  Support s = param_support(this->cfg_.model, id);
  if (s == Support::SUPPORT_NO)
    return false;
  if (s == Support::SUPPORT_PROBE && this->params_[id].unsupported)
    return false;
  return true;
}

bool Engine::set_desired(ParamId id, float value, uint32_t now) {
  if (this->host_ == nullptr)
    return false;  // not configured yet: nothing to validate against or publish to
  this->now_ = now;
  const char *name = param_name(id);
  const ParamSpec &spec = param_spec(id);
  if (param_support(this->cfg_.model, id) == Support::SUPPORT_NO) {
    this->set_error_("%s is not supported by this radar/firmware", name);
    this->publish_param_(id);
    return false;
  }
  if (this->unsupported_firmware_) {
    this->set_error_("%s: radar firmware not supported, settings are read-only", name);
    this->publish_param_(id);
    return false;
  }
  // checked before the capability marks: a probe that failed in the other app is not a verdict
  if (this->dialect_->has_modes && id != ParamId::PARAM_ID_WORK_MODE) {
    // a requested or running switch counts as done: the change is applied once the new app has been read
    WorkMode eff = this->work_mode_;
    const ParamState &wm = this->params_[ParamId::PARAM_ID_WORK_MODE];
    if (wm.dirty) {
      eff = wm.desired >= 0.5f ? WorkMode::WORK_MODE_SPEED : WorkMode::WORK_MODE_PRESENCE;
    } else if (this->mode_switch_pending_ && !this->mode_switch_rejected_) {
      eff = this->mode_switch_target_;
    }
    if (!this->param_in_mode_(id, eff)) {
      this->set_mode_error_(id, eff);
      this->publish_param_(id);
      return false;
    }
  }
  if (!this->param_available(id)) {
    this->set_error_("%s is not supported by this radar/firmware", name);
    this->publish_param_(id);
    return false;
  }
  Limits l = param_limits(this->cfg_.model, id);
  if (!finite(value) || value < l.lo - 1e-4f || value > l.hi + 1e-4f) {
    this->set_error_("%s: %.3f is outside %.3f..%.3f", name, value, l.lo, l.hi);
    this->publish_param_(id);
    return false;
  }
  if (spec.is_integer)
    value = std::round(value);
  if (id == ParamId::PARAM_ID_UART_PRESENCE_EN && value < 0.5f && !this->cfg_.has_pin) {
    this->logf_(LogLevel::LOG_LEVEL_WARN, "disabling the UART presence report: occupancy will no longer update");
  }
  ParamState &s = this->params_[id];
  s.desired = value;
  s.dirty = true;
  this->last_change_ms_ = now;
  this->logf_(LogLevel::LOG_LEVEL_DEBUG, "requested %s = %g", name, value);
  return true;
}

void Engine::loop(uint32_t now) {
  this->now_ = now;

  // in-flight command timers
  if (this->inflight_.active) {
    if (this->inflight_.tail) {
      if (now - this->inflight_.tail_start >= this->inflight_.tail_ms)
        this->complete_(this->inflight_.tail_result);
    } else if (now - this->inflight_.sent_ms >= this->inflight_.timeout_ms) {
      if (this->inflight_.no_reply) {
        this->complete_(Result::RESULT_OK);
      } else if (this->inflight_.cmd.retries > 0) {
        Command c = this->inflight_.cmd;
        c.retries--;
        this->logf_(LogLevel::LOG_LEVEL_DEBUG, "no reply to '%s', retrying", this->last_cmd_);
        this->send_(c);
      } else {
        this->complete_(Result::RESULT_TIMEOUT);
      }
    }
  }

  this->drive_queue_();
  if (!this->inflight_.active && this->queue_.empty()) {
    if (this->op_ != Op::OP_NONE || this->life_ == Life::LIFE_PROBE) {
      // every operation ends with a marker; reaching here means one was lost (queue overflow)
      this->logf_(LogLevel::LOG_LEVEL_ERROR, "internal: operation ended without its end marker");
      this->failing_ = false;
      this->enter_backoff_("internal error");
    }
    this->schedule_();
    this->drive_queue_();
  }

  // work mode confirmation deadline
  if (this->mode_confirm_pending_ && now - this->mode_confirm_start_ms_ >= MODE_CONFIRM_MS) {
    this->mode_confirm_pending_ = false;
    this->publish_param_(ParamId::PARAM_ID_WORK_MODE);
    if (this->mode_confirm_has_request_)
      this->set_error_("work_mode: no report after the change, could not confirm the mode");
  }

  this->check_targets_decay_();
  this->check_uart_presence_stale_();
  this->check_report_after_recovery_();
  this->check_link_();
}

// bytes and lines

void Engine::on_line_(const char *line, size_t len) {
  this->last_rx_ms_ = this->now_;
  Reply r = classify(line, len, this->last_cmd_);
  if (r.kind == ReplyKind::REPLY_KIND_SENTENCE) {
    this->logf_(LogLevel::LOG_LEVEL_VERBOSE, "RX: %s", line);
    this->on_sentence_(line, r);
    return;
  }
  if (r.kind == ReplyKind::REPLY_KIND_ECHO) {
    this->logf_(LogLevel::LOG_LEVEL_VERBOSE, "echo: %s", line);
    return;
  }
  this->logf_(LogLevel::LOG_LEVEL_DEBUG, "RX: %s", line);
  if (!this->inflight_.active) {
    this->logf_(LogLevel::LOG_LEVEL_VERBOSE, "no command pending, ignored (%s)", reply_kind_str(r.kind));
    return;
  }
  this->on_reply_(line, r);
}

void Engine::on_prompt_() {
  this->last_rx_ms_ = this->now_;
  if (!this->prompt_checked_) {
    this->prompt_checked_ = true;
    // compare the name part ("leapMMW" / "DFRobot")
    char expected[16];
    const char *p = this->dialect_->prompt;
    size_t n = 0;
    while (p[n] != '\0' && p[n] != ':' && n < sizeof(expected) - 1) {
      expected[n] = p[n];
      n++;
    }
    expected[n] = '\0';
    if (std::strstr(this->reader_.prompt(), expected) == nullptr) {
      this->logf_(LogLevel::LOG_LEVEL_WARN, "radar prompt '%s' does not match a %s ('%s'): check `model:`",
                  this->reader_.prompt(), this->dialect_->name, this->dialect_->prompt);
    }
  }
  if (this->inflight_.active && this->inflight_.tail)
    this->complete_(this->inflight_.tail_result);
}

void Engine::on_reply_(const char *line, const Reply &r) {
  ReplyKind k = r.kind;
  if (k == ReplyKind::REPLY_KIND_UNKNOWN)
    return;
  if (k == ReplyKind::REPLY_KIND_SAVE_COMPLETE) {
    this->logf_(LogLevel::LOG_LEVEL_DEBUG, "radar saved its configuration");
    return;
  }
  if (k == ReplyKind::REPLY_KIND_FACTORY_RESET_DONE) {
    this->logf_(LogLevel::LOG_LEVEL_DEBUG, "radar restored its factory settings");
    return;
  }
  if (this->inflight_.tail) {
    // the terminal line was already seen; a trailing Done/Error (or anything else) ends the wait
    this->complete_(this->inflight_.tail_result);
    return;
  }
  const Command &c = this->inflight_.cmd;
  switch (c.id) {
    case CmdId::CMD_ID_GET_SWV:
    case CmdId::CMD_ID_GET_HWV:
      if (k == ReplyKind::REPLY_KIND_VERSION_SW || k == ReplyKind::REPLY_KIND_VERSION_HW ||
          k == ReplyKind::REPLY_KIND_RESPONSE) {
        char text[48];
        size_t n = r.text_len < sizeof(text) - 1 ? r.text_len : sizeof(text) - 1;
        std::memcpy(text, line + r.text_off, n);
        text[n] = '\0';
        bool hw = k == ReplyKind::REPLY_KIND_VERSION_HW ||
                  (k == ReplyKind::REPLY_KIND_RESPONSE && c.id == CmdId::CMD_ID_GET_HWV);
        if (this->op_ != Op::OP_PING) {
          this->logf_(LogLevel::LOG_LEVEL_INFO, "radar %s version: %s", hw ? "hardware" : "software", text);
          this->host_->on_version(hw, text);
        }
        this->inflight_.got_values = true;
        this->start_tail_(Result::RESULT_OK, VERSION_TAIL_MS);
      } else if (k == ReplyKind::REPLY_KIND_DONE) {
        this->complete_(Result::RESULT_OK);
      } else if (k == ReplyKind::REPLY_KIND_ERROR) {
        this->complete_(Result::RESULT_ERROR);
      } else if (k == ReplyKind::REPLY_KIND_UNRECOGNIZED) {
        this->start_tail_(Result::RESULT_UNRECOGNIZED, TAIL_MS);
      }
      break;

    case CmdId::CMD_ID_GET_GROUP:
    case CmdId::CMD_ID_GET_RUN_APP:
      if (k == ReplyKind::REPLY_KIND_RESPONSE) {
        if (r.nvals == 0 || r.bad_value) {
          // a corrupted line, not the radar's answer: nothing from it is applied
          this->logf_(LogLevel::LOG_LEVEL_WARN, "unparseable Response to '%s': %s", this->last_cmd_, line);
          this->start_tail_(Result::RESULT_MALFORMED, TAIL_MS);
        } else {
          this->inflight_.nvals = r.nvals;
          for (uint8_t i = 0; i < r.nvals; i++)
            this->inflight_.vals[i] = r.vals[i];
          this->inflight_.got_values = true;
          this->start_tail_(Result::RESULT_OK, TAIL_MS);
        }
      } else if (k == ReplyKind::REPLY_KIND_ERROR) {
        this->complete_(Result::RESULT_ERROR);
      } else if (k == ReplyKind::REPLY_KIND_UNRECOGNIZED) {
        this->start_tail_(Result::RESULT_UNRECOGNIZED, TAIL_MS);
      } else if (k == ReplyKind::REPLY_KIND_DONE) {
        this->logf_(LogLevel::LOG_LEVEL_DEBUG, "'%s' answered Done without a Response", this->last_cmd_);
        this->complete_(Result::RESULT_ERROR);
      } else if (k == ReplyKind::REPLY_KIND_NOT_STOPPED) {
        this->start_tail_(Result::RESULT_ERROR, TAIL_MS);
      }
      break;

    case CmdId::CMD_ID_SET_GROUP:
      if (k == ReplyKind::REPLY_KIND_DONE) {
        this->complete_(Result::RESULT_OK);
      } else if (k == ReplyKind::REPLY_KIND_ERROR) {
        this->complete_(Result::RESULT_ERROR);
      } else if (k == ReplyKind::REPLY_KIND_NOT_STOPPED) {
        this->set_running_(true);
        this->start_tail_(Result::RESULT_ERROR, TAIL_MS);
      } else if (k == ReplyKind::REPLY_KIND_UNRECOGNIZED) {
        this->start_tail_(Result::RESULT_UNRECOGNIZED, TAIL_MS);
      }
      break;

    case CmdId::CMD_ID_STOP:
      if (k == ReplyKind::REPLY_KIND_DONE || k == ReplyKind::REPLY_KIND_ERROR) {
        this->complete_(Result::RESULT_OK);  // Error = not running = stopped
      } else if (k == ReplyKind::REPLY_KIND_ALREADY_STOPPED) {
        this->start_tail_(Result::RESULT_OK, TAIL_MS);
      }
      break;

    case CmdId::CMD_ID_START:
      if (k == ReplyKind::REPLY_KIND_DONE) {
        this->complete_(Result::RESULT_OK);
      } else if (k == ReplyKind::REPLY_KIND_ALREADY_STARTED) {
        this->start_tail_(Result::RESULT_OK, TAIL_MS);
      } else if (k == ReplyKind::REPLY_KIND_UNSAVED_CANT_START) {
        this->start_tail_(Result::RESULT_UNSAVED, TAIL_MS);
      } else if (k == ReplyKind::REPLY_KIND_ERROR) {
        this->complete_(Result::RESULT_ERROR);
      }
      break;

    case CmdId::CMD_ID_SAVE:
      if (k == ReplyKind::REPLY_KIND_DONE) {
        this->complete_(Result::RESULT_OK);
      } else if (k == ReplyKind::REPLY_KIND_NOTHING_TO_SAVE) {
        this->start_tail_(Result::RESULT_OK, TAIL_MS);
      } else if (k == ReplyKind::REPLY_KIND_ERROR) {
        this->complete_(Result::RESULT_ERROR);
      } else if (k == ReplyKind::REPLY_KIND_NOT_STOPPED) {
        this->set_running_(true);
        this->start_tail_(Result::RESULT_NOT_STOPPED, TAIL_MS);
      }
      break;

    case CmdId::CMD_ID_RESET_CFG:
      if (k == ReplyKind::REPLY_KIND_DONE) {
        this->complete_(Result::RESULT_OK);
      } else if (k == ReplyKind::REPLY_KIND_ERROR) {
        this->complete_(Result::RESULT_ERROR);
      } else if (k == ReplyKind::REPLY_KIND_UNRECOGNIZED) {
        this->start_tail_(Result::RESULT_UNRECOGNIZED, TAIL_MS);
      }
      break;

    default:
      break;
  }
}

// command driving

bool Engine::enqueue_(CmdId id, GroupId group, uint8_t flags) {
  Command c;
  c.id = id;
  c.group = group;
  c.flags = flags;
  switch (id) {
    case CmdId::CMD_ID_SAVE:
      c.retries = 1;
      break;
    case CmdId::CMD_ID_RESET_SYSTEM:
    case CmdId::CMD_ID_FLUSH:
    // capability probes: a firmware that silently ignores an unknown command must not cost 3 timeouts
    case CmdId::CMD_ID_GET_RUN_APP:
      c.retries = 0;
      break;
    case CmdId::CMD_ID_GET_GROUP:
      c.retries = group_support(this->cfg_.model, group) == Support::SUPPORT_PROBE ? 0 : 2;
      break;
    default:
      c.retries = is_marker(id) ? 0 : 2;
      break;
  }
  return this->enqueue_cmd_(c);
}

bool Engine::enqueue_cmd_(const Command &c) {
  if (!this->queue_.push_back(c)) {
    this->logf_(LogLevel::LOG_LEVEL_ERROR, "command queue full, dropping %s", cmd_str(c.id));
    return false;
  }
  return true;
}

void Engine::drive_queue_() {
  if (this->mode_wait_active_) {
    if (!this->mode_wait_seen_ && this->now_ - this->mode_wait_start_ms_ < MODE_SWITCH_WAIT_MS)
      return;
    this->mode_wait_active_ = false;
    const bool reported = this->mode_wait_seen_;  // the presence inference below marks the wait as seen
    if (reported) {
      this->logf_(LogLevel::LOG_LEVEL_DEBUG, "new app is reporting");
    } else if (this->mode_switch_target_ == WorkMode::WORK_MODE_PRESENCE) {
      // C4001 firmware: the speed app is never silent, and the presence app prints nothing while
      // uart_presence_report is off, so silence after setRunApp 0 means the presence app
      this->logf_(LogLevel::LOG_LEVEL_DEBUG, "no report in %us: presence app",
                  static_cast<unsigned>(MODE_SWITCH_WAIT_MS / 1000));
      this->set_work_mode_(WorkMode::WORK_MODE_PRESENCE);
    } else {
      this->logf_(LogLevel::LOG_LEVEL_DEBUG, "no report from the new app yet");
    }
    if (!reported) {
      // Silence does not prove that setRunApp was carried out: a command lost on the way leaves the radar stopped
      // in the old app, where it is just as silent. Start it before the identify-and-read; a radar that already
      // runs answers that it is started, and one that was still stopped starts in whatever app it is in, so the
      // reports and reads that follow show the mode it is really in.
      Command start;
      start.id = CmdId::CMD_ID_START;
      start.retries = 1;
      start.flags = CMD_FLAG_RECOVERY;
      this->queue_.push_front(start);
    }
  }
  for (uint8_t guard = 0; guard < 16 && !this->inflight_.active && !this->queue_.empty(); guard++) {
    const Command &f = this->queue_.front();
    if (is_marker(f.id)) {
      Command c = this->queue_.pop_front();
      this->on_marker_(c);
      if (this->mode_wait_active_)
        return;  // MARK_MODE_WAIT: hold the rest of the queue until the new app reports or the wait expires
      continue;
    }
    if (f.id == CmdId::CMD_ID_SAVE && (f.flags & CMD_FLAG_OPTIONAL_SAVE) != 0 && this->set_ok_mask_ == 0) {
      this->queue_.pop_front();
      this->logf_(LogLevel::LOG_LEVEL_DEBUG, "no set command succeeded, skipping saveConfig");
      continue;
    }
    if (this->now_ - this->last_tx_ms_ < this->gap_ms_)
      return;
    Command c = this->queue_.pop_front();
    this->send_(c);
  }
}

void Engine::write_line_(const char *text) {
  size_t n = std::strlen(text);
  if (n > 0)
    this->host_->write(text, n);
  this->host_->write("\r\n", 2);
}

void Engine::send_(Command c) {
  char buf[MAX_COMMAND_LEN + 1];
  size_t n = 0;
  uint16_t timeout = TIMEOUT_MS;
  uint16_t gap = this->dialect_->min_gap_ms;
  bool no_reply = false;
  float one = 1.0f;
  float zero = 0.0f;
  buf[0] = '\0';

  switch (c.id) {
    case CmdId::CMD_ID_GET_SWV:
      n = fmt_command(buf, sizeof(buf), "getSWV", -1, nullptr, 0, 0);
      break;
    case CmdId::CMD_ID_GET_HWV:
      n = fmt_command(buf, sizeof(buf), "getHWV", -1, nullptr, 0, 0);
      break;
    case CmdId::CMD_ID_GET_RUN_APP:
      n = fmt_command(buf, sizeof(buf), "getRunApp", -1, nullptr, 0, 0);
      break;
    case CmdId::CMD_ID_GET_GROUP: {
      const GroupSpec &gs = group_spec(c.group);
      if (gs.get_cmd != nullptr)
        n = fmt_command(buf, sizeof(buf), gs.get_cmd, gs.prefix, nullptr, 0, 0);
      break;
    }
    case CmdId::CMD_ID_SET_GROUP: {
      const GroupSpec &gs = group_spec(c.group);
      if (gs.set_cmd != nullptr)
        n = fmt_command(buf, sizeof(buf), gs.set_cmd, gs.prefix, c.args, c.nargs, gs.int_mask);
      if (!gs.set_has_reply) {
        no_reply = true;
        timeout = this->dialect_->min_gap_ms;
      }
      break;
    }
    case CmdId::CMD_ID_STOP:
      n = fmt_command(buf, sizeof(buf), "sensorStop", -1, nullptr, 0, 0);
      break;
    case CmdId::CMD_ID_START:
      if (this->dialect_->start_resume_arg) {
        n = fmt_command(buf, sizeof(buf), "sensorStart", -1, &one, 1, 1);
      } else {
        n = fmt_command(buf, sizeof(buf), "sensorStart", -1, nullptr, 0, 0);
      }
      break;
    case CmdId::CMD_ID_SAVE:
      n = fmt_command(buf, sizeof(buf), "saveConfig", -1, nullptr, 0, 0);
      timeout = SAVE_TIMEOUT_MS;
      gap = this->dialect_->save_gap_ms;
      break;
    case CmdId::CMD_ID_RESET_CFG:
      n = fmt_command(buf, sizeof(buf), "resetCfg", -1, nullptr, 0, 0);
      gap = this->dialect_->save_gap_ms;
      break;
    case CmdId::CMD_ID_RESET_SYSTEM:
      n = fmt_command(buf, sizeof(buf), "resetSystem", -1, &zero, 1, 1);
      no_reply = true;
      timeout = this->dialect_->reset_wait_ms;
      break;
    case CmdId::CMD_ID_FLUSH:
      no_reply = true;
      timeout = this->dialect_->min_gap_ms;
      break;
    default:
      return;
  }

  this->inflight_ = InFlight{};
  this->inflight_.active = true;
  this->inflight_.cmd = c;

  if (n == 0 && c.id != CmdId::CMD_ID_FLUSH) {
    this->logf_(LogLevel::LOG_LEVEL_ERROR, "could not format %s command", cmd_str(c.id));
    this->complete_(Result::RESULT_ERROR);
    return;
  }
  if (c.id != CmdId::CMD_ID_FLUSH)
    std::memcpy(this->last_cmd_, buf, n + 1);
  this->logf_(LogLevel::LOG_LEVEL_DEBUG, "TX: %s", c.id == CmdId::CMD_ID_FLUSH ? "<terminator>" : buf);
  this->write_line_(buf);
  this->inflight_.sent_ms = this->now_;
  this->inflight_.timeout_ms = timeout;
  this->inflight_.no_reply = no_reply;
  this->last_tx_ms_ = this->now_;
  this->gap_ms_ = gap;
}

void Engine::start_tail_(Result result, uint16_t ms) {
  this->inflight_.tail = true;
  this->inflight_.tail_result = result;
  this->inflight_.tail_start = this->now_;
  this->inflight_.tail_ms = ms;
}

void Engine::complete_(Result result) {
  if (!this->inflight_.active)
    return;
  Command c = this->inflight_.cmd;
  this->inflight_.active = false;
  this->inflight_.tail = false;
  if (result == Result::RESULT_MALFORMED && (c.flags & CMD_FLAG_REPARSE) == 0) {
    c.flags |= CMD_FLAG_REPARSE;
    if (this->queue_.push_front(c))
      return;  // asked once more before the read counts as failed
  }
  this->on_done_(c, result);
}

void Engine::on_done_(const Command &c, Result result) {
  const char *rs = result == Result::RESULT_OK            ? "ok"
                   : result == Result::RESULT_ERROR       ? "Error"
                   : result == Result::RESULT_TIMEOUT     ? "timeout"
                   : result == Result::RESULT_UNSAVED     ? "unsaved"
                   : result == Result::RESULT_NOT_STOPPED ? "not stopped"
                   : result == Result::RESULT_MALFORMED   ? "unparseable"
                                                          : "not recognized";
  switch (c.id) {
    case CmdId::CMD_ID_GET_SWV:
      if (this->op_ == Op::OP_PING) {
        if (result == Result::RESULT_TIMEOUT) {
          this->queue_.clear();
          this->enter_backoff_("no data from the radar");
        } else {
          this->last_rx_ms_ = this->now_;
          if (!this->quiet_logged_) {
            this->quiet_logged_ = true;
            this->logf_(LogLevel::LOG_LEVEL_DEBUG, "radar is quiet but answers commands");
          }
        }
      } else if (result == Result::RESULT_TIMEOUT) {
        this->queue_.clear();
        this->op_ = Op::OP_NONE;
        this->write_line_("");  // flush a half-typed line
        if (this->sentence_seen_ && this->now_ - this->last_sentence_ms_ < 5000) {
          this->logf_(LogLevel::LOG_LEVEL_WARN,
                      "radar sends reports but does not answer commands: check the ESP TX -> radar RX wire");
        }
        this->enter_backoff_("no answer to getSWV");
      } else {
        this->probe_answered_ = true;
      }
      break;

    case CmdId::CMD_ID_GET_HWV:
      break;

    case CmdId::CMD_ID_GET_RUN_APP:
      if (result == Result::RESULT_OK && this->inflight_.got_values) {
        this->logf_(LogLevel::LOG_LEVEL_DEBUG, "getRunApp answered %g", this->inflight_.vals[0]);
      } else {
        this->logf_(LogLevel::LOG_LEVEL_DEBUG, "getRunApp not supported (%s); work mode is inferred from reports", rs);
      }
      break;

    case CmdId::CMD_ID_GET_GROUP: {
      const GroupSpec &gs = group_spec(c.group);
      bool probe = group_support(this->cfg_.model, c.group) == Support::SUPPORT_PROBE;
      bool boot = this->op_ == Op::OP_BOOT_READ;
      auto mark_unsupported = [this, &c]() {
        for (uint8_t i = 0; i < PARAM_COUNT; i++) {
          auto p = static_cast<ParamId>(i);
          if (param_spec(p).group == c.group && param_support(this->cfg_.model, p) == Support::SUPPORT_PROBE)
            this->params_[p].unsupported = true;
        }
      };
      if (result == Result::RESULT_OK && this->inflight_.got_values) {
        if (this->apply_response_(c.group, this->inflight_.vals, this->inflight_.nvals)) {
          this->read_ok_mask_ |= gbit(c.group);
        } else {
          this->logf_(LogLevel::LOG_LEVEL_WARN, "short Response to %s (%u values)", gs.get_cmd,
                      static_cast<unsigned>(this->inflight_.nvals));
          if (probe && boot)
            mark_unsupported();
        }
      } else if (result == Result::RESULT_MALFORMED) {
        // unparseable twice: a failed read, never proof that the firmware lacks the setting
        if (!this->first_read_phase_)
          this->op_failed_(c, "unparseable response");
      } else if (probe && boot) {
        mark_unsupported();
        this->logf_(LogLevel::LOG_LEVEL_INFO, "%s: %s, not supported by this firmware", gs.get_cmd, rs);
      } else if (result == Result::RESULT_TIMEOUT) {
        if (!this->first_read_phase_)
          this->op_failed_(c, "get timed out");
      } else if (this->op_ == Op::OP_READ && c.group == GroupId::GROUP_ID_RANGE && !this->get_requires_stop_ &&
                 this->running_) {
        this->get_requires_stop_ = true;
        this->get_mode_known_ = true;
        this->logf_(LogLevel::LOG_LEVEL_INFO, "radar only answers get commands while stopped");
      } else if (this->dialect_->has_modes && group_modes(this->cfg_.model, c.group) != MODE_MASK_BOTH) {
        this->logf_(LogLevel::LOG_LEVEL_DEBUG, "%s: %s (not available in the current mode)", gs.get_cmd, rs);
      } else if (!this->first_read_phase_) {
        this->logf_(LogLevel::LOG_LEVEL_WARN, "%s answered %s", gs.get_cmd, rs);
      }
      break;
    }

    case CmdId::CMD_ID_SET_GROUP:
      if (c.group == GroupId::GROUP_ID_RUN_APP) {
        if (result == Result::RESULT_OK) {
          // C4001 firmware: setRunApp restarts the radar into the new app at once and persists by itself
          this->set_running_(true);
          this->mode_wait_seen_ = false;
        } else {
          // The radar is still in the old app, stopped by our sensorStop. Skip the wait for the new app: the
          // identify-and-read that follows starts the radar again and publishes the mode it is really in.
          this->mode_switch_rejected_ = true;
          if (!this->queue_.empty() && this->queue_.front().id == CmdId::CMD_ID_MARK_MODE_WAIT)
            this->queue_.pop_front();
          this->set_error_("work_mode: radar rejected '%s' (%s)", this->last_cmd_, rs);
        }
      } else if (result == Result::RESULT_OK) {
        this->set_ok_mask_ |= gbit(c.group);
      } else if (result == Result::RESULT_TIMEOUT) {
        this->op_failed_(c, "set command timed out");
      } else {
        this->set_fail_mask_ |= gbit(c.group);
        this->set_error_("radar rejected '%s' (%s)", this->last_cmd_, rs);
      }
      break;

    case CmdId::CMD_ID_STOP:
      if (result == Result::RESULT_TIMEOUT) {
        // the radar may have received it even though no reply arrived (TX wire only)
        this->stop_unconfirmed_ = true;
        this->op_failed_(c, "sensorStop timed out");
      } else {
        this->set_running_(false);
      }
      break;

    case CmdId::CMD_ID_START:
      if (result == Result::RESULT_OK || (result == Result::RESULT_ERROR && (c.flags & CMD_FLAG_RECOVERY) != 0)) {
        if (result == Result::RESULT_ERROR)
          this->logf_(LogLevel::LOG_LEVEL_DEBUG, "sensorStart answered Error: the radar was already running");
        this->set_running_(true);
        this->stop_unconfirmed_ = false;
        this->mode_sentences_since_start_ = 0;
      } else if (result == Result::RESULT_UNSAVED && (c.flags & CMD_FLAG_START_RETRY) == 0) {
        this->logf_(LogLevel::LOG_LEVEL_WARN, "radar refuses to start with unsaved changes, saving first");
        Command start = c;
        start.flags |= CMD_FLAG_START_RETRY;
        start.retries = 1;
        Command save;
        save.id = CmdId::CMD_ID_SAVE;
        save.retries = 1;
        this->queue_.push_front(start);
        this->queue_.push_front(save);
      } else if (result == Result::RESULT_TIMEOUT) {
        if ((c.flags & CMD_FLAG_FAILURE_PATH) == 0)
          this->op_failed_(c, "sensorStart timed out");
      } else {
        this->set_error_("sensorStart answered %s", rs);
      }
      break;

    case CmdId::CMD_ID_SAVE:
      if (result == Result::RESULT_OK) {
        this->save_count_++;
      } else if (result == Result::RESULT_TIMEOUT) {
        this->op_failed_(c, "saveConfig timed out");
      } else if (result == Result::RESULT_ERROR && (this->op_ == Op::OP_TXN || this->op_ == Op::OP_FACTORY_RESET)) {
        // C4001 firmware: a bare Error means nothing changed; finish_txn_() decides after the read-back
        this->save_error_seen_ = true;
      } else if (result == Result::RESULT_NOT_STOPPED) {
        this->set_error_("saveConfig answered 'sensor is not stopped'");
      } else {
        this->set_error_("saveConfig answered %s", rs);
      }
      break;

    case CmdId::CMD_ID_RESET_CFG:
      if (result == Result::RESULT_TIMEOUT) {
        this->op_failed_(c, "resetCfg timed out");
      } else if (result != Result::RESULT_OK) {
        this->set_error_("resetCfg answered %s", rs);
      }
      break;

    case CmdId::CMD_ID_RESET_SYSTEM:
      this->queue_.clear();
      this->op_ = Op::OP_NONE;
      this->reset_radar_state_();
      this->life_ = Life::LIFE_BOOT_WAIT;
      this->boot_start_ms_ = this->now_;
      this->set_status_(Status::STATUS_BOOT_WAIT);
      this->logf_(LogLevel::LOG_LEVEL_INFO, "radar restarting");
      break;

    default:
      break;
  }
}

void Engine::op_failed_(const Command &c, const char *why) {
  if (this->failing_)
    return;
  this->failing_ = true;
  std::snprintf(this->fail_reason_, sizeof(this->fail_reason_), "%s", why);
  this->logf_(LogLevel::LOG_LEVEL_WARN, "%s", why);
  this->queue_.clear();
  if (this->stopped_by_op_ && this->restart_after_op_ && c.id != CmdId::CMD_ID_START && c.id != CmdId::CMD_ID_STOP) {
    Command start;
    start.id = CmdId::CMD_ID_START;
    start.retries = 1;
    start.flags = CMD_FLAG_FAILURE_PATH;
    this->queue_.push_back(start);
  }
  Command mark;
  mark.id = CmdId::CMD_ID_MARK_FAIL_END;
  this->queue_.push_back(mark);
}

void Engine::on_marker_(const Command &c) {
  switch (c.id) {
    case CmdId::CMD_ID_MARK_PROBE_DONE:
      this->start_identify_and_read_();
      break;
    case CmdId::CMD_ID_MARK_FIRST_READ: {
      this->first_read_phase_ = false;
      // the remaining gets are chosen for the mode known now
      this->last_read_mode_ = this->read_mode_();
      this->mode_read_req_ = false;
      uint16_t rest = this->readable_groups_();
      if (this->read_ok_mask_ & gbit(GroupId::GROUP_ID_RANGE)) {
        bool known_stopped = this->running_known_ && !this->running_;
        if (!known_stopped) {
          this->get_requires_stop_ = false;
          this->get_mode_known_ = true;
          this->logf_(LogLevel::LOG_LEVEL_DEBUG, "radar answers get commands while running");
        }
        this->enqueue_gets_(static_cast<uint16_t>(rest & ~gbit(GroupId::GROUP_ID_RANGE)));
        // A failed operation or a lost link may have left the radar stopped without any reply saying so.
        if ((known_stopped || this->recover_start_ || this->stop_unconfirmed_) && !this->user_stopped_) {
          this->logf_(LogLevel::LOG_LEVEL_INFO, "starting the radar after %s",
                      known_stopped ? "an interrupted operation" : "a link loss");
          this->enqueue_(CmdId::CMD_ID_START, GroupId::GROUP_ID_COUNT, CMD_FLAG_RECOVERY);
        }
        this->recover_start_ = false;
      } else {
        this->get_requires_stop_ = true;
        this->get_mode_known_ = true;
        this->logf_(LogLevel::LOG_LEVEL_INFO, "getRange not answered while running; reading with the radar stopped");
        this->stopped_by_op_ = true;
        this->restart_after_op_ = !this->user_stopped_;
        this->enqueue_(CmdId::CMD_ID_STOP);
        this->enqueue_gets_(rest);
        if (this->restart_after_op_)
          this->enqueue_(CmdId::CMD_ID_START);
        this->recover_start_ = false;
      }
      this->enqueue_(CmdId::CMD_ID_MARK_READ_END);
      break;
    }
    case CmdId::CMD_ID_MARK_READ_END:
      this->finish_read_(this->op_ == Op::OP_BOOT_READ);
      break;
    case CmdId::CMD_ID_MARK_TXN_END:
      this->finish_txn_();
      break;
    case CmdId::CMD_ID_MARK_RUN_DONE:
      this->op_ = Op::OP_NONE;
      this->set_status_(this->running_ ? Status::STATUS_RUNNING : Status::STATUS_STOPPED);
      break;
    case CmdId::CMD_ID_MARK_FAIL_END:
      this->finish_failure_();
      break;
    case CmdId::CMD_ID_MARK_PING_END:
      this->op_ = Op::OP_NONE;
      break;
    case CmdId::CMD_ID_MARK_MODE_WAIT:
      this->mode_wait_active_ = true;
      this->mode_wait_start_ms_ = this->now_;
      break;
    default:
      break;
  }
}

// lifecycle and operations

void Engine::schedule_() {
  switch (this->life_) {
    case Life::LIFE_BOOT_WAIT:
      if (this->now_ - this->boot_start_ms_ >= this->dialect_->boot_delay_ms)
        this->start_probe_();
      return;
    case Life::LIFE_BACKOFF:
      if (this->now_ - this->backoff_start_ms_ >= this->backoff_wait_ms_)
        this->start_probe_();
      return;
    case Life::LIFE_PROBE:
      return;
    case Life::LIFE_RUNNING:
      break;
  }
  if (this->op_ != Op::OP_NONE)
    return;

  if (this->restart_req_) {
    this->restart_req_ = false;
    this->op_ = Op::OP_RESTART;
    this->enqueue_(CmdId::CMD_ID_RESET_SYSTEM);
    return;
  }
  if (this->factory_req_) {
    this->factory_req_ = false;
    if (this->unsupported_firmware_) {
      this->set_error_("factory reset: radar firmware not supported");
    } else {
      this->start_factory_reset_();
    }
    return;
  }
  if (this->run_req_) {
    this->run_req_ = false;
    this->op_ = Op::OP_RUN_CTRL;
    this->user_stopped_ = !this->run_req_on_;
    this->enqueue_(this->run_req_on_ ? CmdId::CMD_ID_START : CmdId::CMD_ID_STOP);
    this->enqueue_(CmdId::CMD_ID_MARK_RUN_DONE);
    return;
  }
  bool dirty = this->params_.any_dirty();
  // changes made close together are coalesced into one transaction
  if (dirty && !this->unsupported_firmware_ && this->now_ - this->last_change_ms_ >= this->cfg_.debounce_ms) {
    // a work-mode change is never batched: the other settings belong to the new app
    if (this->params_[ParamId::PARAM_ID_WORK_MODE].dirty) {
      this->start_mode_switch_();
    } else {
      this->start_txn_();
    }
    return;
  }
  if (this->unsupported_firmware_) {
    this->refresh_req_ = false;
    this->mode_read_req_ = false;
    return;
  }
  if (this->refresh_req_) {
    this->refresh_req_ = false;
    this->params_.clear_dirty();  // unapplied changes revert to the radar's values
    if (this->get_requires_stop_)
      this->logf_(LogLevel::LOG_LEVEL_WARN,
                  "re-reading settings: this radar needs a brief stop to answer get commands");
    this->start_read_(true);
    return;
  }
  if (this->mode_read_req_ && !dirty && !this->get_requires_stop_) {
    this->start_read_(false);
  }
}

void Engine::start_probe_() {
  this->life_ = Life::LIFE_PROBE;
  this->op_ = Op::OP_PROBE;
  this->queue_.clear();
  this->probe_answered_ = false;
  this->failing_ = false;
  if (this->status_ != Status::STATUS_LINK_LOST)
    this->set_status_(Status::STATUS_PROBING);
  this->enqueue_(CmdId::CMD_ID_GET_SWV);
  this->enqueue_(CmdId::CMD_ID_MARK_PROBE_DONE);
}

void Engine::start_identify_and_read_() {
  if (!this->mode_switch_pending_)
    this->begin_op_errors_();
  // capability marks are re-probed on every identify-and-read: a get that failed in the other app
  // must not hide a setting forever
  for (uint8_t i = 0; i < PARAM_COUNT; i++)
    this->params_[static_cast<ParamId>(i)].unsupported = false;
  this->last_read_mode_ = this->read_mode_();
  this->mode_read_req_ = false;
  this->op_ = Op::OP_BOOT_READ;
  this->set_status_(Status::STATUS_READING);
  this->read_ok_mask_ = 0;
  this->read_mask_ = 0;
  this->stopped_by_op_ = false;
  this->restart_after_op_ = true;
  this->failing_ = false;
  this->first_read_phase_ = true;
  this->enqueue_(CmdId::CMD_ID_GET_HWV);
  if (this->dialect_->has_modes)
    this->enqueue_(CmdId::CMD_ID_GET_RUN_APP);
  this->enqueue_(CmdId::CMD_ID_GET_GROUP, GroupId::GROUP_ID_RANGE);
  this->read_mask_ |= gbit(GroupId::GROUP_ID_RANGE);
  this->enqueue_(CmdId::CMD_ID_MARK_FIRST_READ);
}

void Engine::start_read_(bool user_requested) {
  this->begin_op_errors_();
  this->mode_read_req_ = false;
  this->op_ = Op::OP_READ;
  this->read_ok_mask_ = 0;
  this->read_mask_ = 0;
  this->failing_ = false;
  this->first_read_phase_ = false;
  for (uint8_t i = 0; i < PARAM_COUNT; i++) {
    this->prev_reported_[i] = this->params_[static_cast<ParamId>(i)].reported;
    this->prev_valid_[i] = this->params_[static_cast<ParamId>(i)].reported_valid;
  }
  uint16_t groups = this->readable_groups_();
  if (this->get_requires_stop_) {
    this->set_status_(Status::STATUS_READING);
    this->stopped_by_op_ = true;
    this->restart_after_op_ = !this->user_stopped_;
    this->enqueue_(CmdId::CMD_ID_STOP);
    this->enqueue_gets_(groups);
    if (this->restart_after_op_)
      this->enqueue_(CmdId::CMD_ID_START);
  } else {
    this->stopped_by_op_ = false;
    if (user_requested)
      this->set_status_(Status::STATUS_READING);
    this->enqueue_gets_(groups);
  }
  this->last_read_mode_ = this->read_mode_();
  this->enqueue_(CmdId::CMD_ID_MARK_READ_END);
}

WorkMode Engine::read_mode_() const {
  if (!this->dialect_->has_modes)
    return WorkMode::WORK_MODE_UNKNOWN;
  // C4001 firmware: the speed app streams $DFDMD continuously, so no report yet means the presence app
  return this->work_mode_ == WorkMode::WORK_MODE_UNKNOWN ? WorkMode::WORK_MODE_PRESENCE : this->work_mode_;
}

// Groups of the other work mode are skipped: the C4001 answers Error to them, which is not a failure.
void Engine::enqueue_gets_(uint16_t mask) {
  WorkMode mode = this->read_mode_();
  for (uint8_t i = 0; i < GROUP_COUNT; i++) {
    auto g = static_cast<GroupId>(i);
    if ((mask & gbit(g)) == 0 || group_spec(g).get_cmd == nullptr || !this->group_usable_(g))
      continue;
    if (mode != WorkMode::WORK_MODE_UNKNOWN) {
      uint8_t bit = mode == WorkMode::WORK_MODE_SPEED ? MODE_MASK_SPEED : MODE_MASK_PRESENCE;
      if ((group_modes(this->cfg_.model, g) & bit) == 0)
        continue;
    }
    if (this->enqueue_(CmdId::CMD_ID_GET_GROUP, g))
      this->read_mask_ |= gbit(g);
  }
}

uint16_t Engine::readable_groups_() const {
  uint16_t mask = 0;
  for (uint8_t i = 0; i < GROUP_COUNT; i++) {
    auto g = static_cast<GroupId>(i);
    if (group_spec(g).get_cmd != nullptr && group_support(this->cfg_.model, g) != Support::SUPPORT_NO)
      mask |= gbit(g);
  }
  return mask;
}

bool Engine::group_usable_(GroupId g) const {
  Support gs = group_support(this->cfg_.model, g);
  if (gs == Support::SUPPORT_NO)
    return false;
  for (uint8_t i = 0; i < PARAM_COUNT; i++) {
    auto p = static_cast<ParamId>(i);
    if (param_spec(p).group != g)
      continue;
    Support ps = param_support(this->cfg_.model, p);
    if (ps == Support::SUPPORT_YES || (ps == Support::SUPPORT_PROBE && !this->params_[p].unsupported))
      return true;
  }
  return false;
}

void Engine::finish_read_(bool boot) {
  for (uint8_t i = 0; i < PARAM_COUNT; i++) {
    auto p = static_cast<ParamId>(i);
    if (param_support(this->cfg_.model, p) == Support::SUPPORT_NO || p == ParamId::PARAM_ID_WORK_MODE)
      continue;
    if ((this->read_ok_mask_ & gbit(param_spec(p).group)) == 0)
      continue;
    const ParamState &s = this->params_[p];
    if (!boot && this->op_ == Op::OP_READ && this->prev_valid_[i]) {
      Limits l = param_limits(this->cfg_.model, p);
      if (std::fabs(this->prev_reported_[i] - s.reported) > l.step / 2.0f) {
        this->logf_(LogLevel::LOG_LEVEL_INFO, "radar reports %s = %g (previously %g)", param_name(p), s.reported,
                    this->prev_reported_[i]);
      }
    }
    if (!s.dirty)
      this->publish_param_(p);
  }
  if (boot) {
    if ((this->read_ok_mask_ & gbit(GroupId::GROUP_ID_RANGE)) == 0) {
      this->unsupported_firmware_ = true;
      this->set_error_("radar does not answer getRange: firmware not supported, settings disabled");
    } else if (this->unsupported_firmware_) {
      this->unsupported_firmware_ = false;
      this->logf_(LogLevel::LOG_LEVEL_INFO, "radar answers getRange: settings enabled");
    }
    for (uint8_t i = 0; i < PARAM_COUNT; i++) {
      auto p = static_cast<ParamId>(i);
      if (param_support(this->cfg_.model, p) == Support::SUPPORT_PROBE && this->params_[p].unsupported)
        this->logf_(LogLevel::LOG_LEVEL_INFO, "%s: not supported by this firmware", param_name(p));
    }
    this->ever_running_ = true;
    this->backoff_ms_ = BACKOFF_MIN_MS;
    this->life_ = Life::LIFE_RUNNING;
    this->set_link_ok_(true);
    if (this->mode_switch_pending_)
      this->finish_mode_switch_();
    this->stop_unconfirmed_ = false;
    if (this->dialect_->has_modes) {
      // C4001 firmware: the speed app is never silent, so a live radar that has printed nothing for the mode-switch
      // wait is in the presence app (with uart_presence_report off it prints nothing at all). This read proves the
      // link; the publish gives the work_mode select a value even when no sentence ever sets it.
      bool silent = !this->sentence_seen_ || this->now_ - this->last_sentence_ms_ >= MODE_SWITCH_WAIT_MS;
      // a silent radar never confirms that it runs, so only a radar known to be stopped is excluded
      bool known_stopped = this->running_known_ && !this->running_;
      if (this->work_mode_ != WorkMode::WORK_MODE_PRESENCE && silent && !known_stopped && !this->user_stopped_)
        this->set_work_mode_(WorkMode::WORK_MODE_PRESENCE);
      this->publish_param_(ParamId::PARAM_ID_WORK_MODE);
    }

    const ParamState &en = this->params_[ParamId::PARAM_ID_UART_PRESENCE_EN];
    const ParamState &period = this->params_[ParamId::PARAM_ID_UART_REPORT_PERIOD];
    this->report_check_armed_ = this->dialect_->has_modes && this->read_mode_() == WorkMode::WORK_MODE_PRESENCE &&
                                !this->user_stopped_ && en.reported_valid && en.reported >= 0.5f &&
                                !this->report_passive_ && period.reported_valid && period.reported < 5.0f;
    if (this->report_check_armed_)
      this->report_check_deadline_ms_ = this->now_ + static_cast<uint32_t>(period.reported * 2000.0f) + 1000;
  }
  this->op_ = Op::OP_NONE;
  this->stopped_by_op_ = false;
  this->clear_error_if_clean_();
  if (this->unsupported_firmware_) {
    this->set_status_(Status::STATUS_UNSUPPORTED_FIRMWARE);
  } else {
    this->set_status_(this->running_known_ && !this->running_ ? Status::STATUS_STOPPED : Status::STATUS_RUNNING);
  }
}

// transactions

float Engine::value_for_(ParamId id) const {
  if (this->snapshot_has_[idx(id)])
    return this->snapshot_[idx(id)];
  return this->params_.reported_or_nan(id);
}

bool Engine::build_set_(GroupId g, Command &c) {
  const GroupSpec &gs = group_spec(g);
  c = Command{};
  c.id = CmdId::CMD_ID_SET_GROUP;
  c.group = g;
  c.retries = gs.set_has_reply ? 2 : 0;
  if (gs.set_cmd == nullptr || !this->group_usable_(g))
    return false;

  auto single = [this, &c](ParamId p) {
    float v = this->value_for_(p);
    if (!finite(v)) {
      this->set_error_("cannot set %s: value unknown", param_name(p));
      return false;
    }
    c.args[0] = v;
    c.nargs = 1;
    return true;
  };

  switch (g) {
    case GroupId::GROUP_ID_RUN_APP:
      return single(ParamId::PARAM_ID_WORK_MODE);
    case GroupId::GROUP_ID_TRIG_RANGE:
      return single(ParamId::PARAM_ID_TRIG_RANGE);
    case GroupId::GROUP_ID_SENSITIVITY:
      return single(ParamId::PARAM_ID_SENSITIVITY);
    case GroupId::GROUP_ID_INHIBIT:
      return single(ParamId::PARAM_ID_INHIBIT);
    case GroupId::GROUP_ID_MICRO_MOTION:
      return single(ParamId::PARAM_ID_MICRO_MOTION);
    case GroupId::GROUP_ID_THR_FACTOR:
      return single(ParamId::PARAM_ID_THR_FACTOR);
    case GroupId::GROUP_ID_LED:
      if (!single(ParamId::PARAM_ID_LED))
        return false;
      c.args[0] = c.args[0] >= 0.5f ? 0.0f : 1.0f;  // LED on = wire value 0 (blink)
      return true;
    case GroupId::GROUP_ID_RANGE: {
      float mn = this->value_for_(ParamId::PARAM_ID_RANGE_MIN);
      float mx = this->value_for_(ParamId::PARAM_ID_RANGE_MAX);
      if (!finite(mn) || !finite(mx)) {
        this->set_error_("cannot set range: %s unknown", finite(mn) ? "max_range" : "min_range");
        return false;
      }
      if (this->dialect_->range_quantized) {
        mn = quantize_down(mn, this->dialect_->min_range.step);
        mx = quantize_down(mx, this->dialect_->max_range.step);
        // compare the read-back against what is actually sent
        if (this->snapshot_has_[idx(ParamId::PARAM_ID_RANGE_MIN)])
          this->snapshot_[idx(ParamId::PARAM_ID_RANGE_MIN)] = mn;
        if (this->snapshot_has_[idx(ParamId::PARAM_ID_RANGE_MAX)])
          this->snapshot_[idx(ParamId::PARAM_ID_RANGE_MAX)] = mx;
      }
      if (!(mn < mx)) {
        bool min_req = this->snapshot_has_[idx(ParamId::PARAM_ID_RANGE_MIN)];
        bool max_req = this->snapshot_has_[idx(ParamId::PARAM_ID_RANGE_MAX)];
        this->set_error_("min_range %.2f must be below max_range %.2f; reverted %s", mn, mx,
                         min_req && max_req ? "min_range and max_range"
                         : min_req          ? "min_range"
                                            : "max_range");
        return false;
      }
      c.args[0] = mn;
      c.args[1] = mx;
      c.nargs = 2;
      return true;
    }
    case GroupId::GROUP_ID_SENS_SPLIT: {
      bool h = this->snapshot_has_[idx(ParamId::PARAM_ID_HOLD_SENS)];
      bool t = this->snapshot_has_[idx(ParamId::PARAM_ID_TRIG_SENS)];
      if (!h && !t)
        return false;
      c.args[0] = h ? this->snapshot_[idx(ParamId::PARAM_ID_HOLD_SENS)] : SENSITIVITY_UNCHANGED;
      c.args[1] = t ? this->snapshot_[idx(ParamId::PARAM_ID_TRIG_SENS)] : SENSITIVITY_UNCHANGED;
      c.nargs = 2;
      return true;
    }
    case GroupId::GROUP_ID_LATENCY: {
      float on = this->value_for_(ParamId::PARAM_ID_LATENCY_ON);
      float off = this->value_for_(ParamId::PARAM_ID_LATENCY_OFF);
      if (!finite(on) || !finite(off)) {
        this->set_error_("cannot set latency: %s unknown", finite(on) ? "off_latency" : "on_latency");
        return false;
      }
      c.args[0] = on;
      c.args[1] = off;
      c.nargs = 2;
      return true;
    }
    case GroupId::GROUP_ID_UART_OUT1:
    case GroupId::GROUP_ID_UART_OUT2: {
      ParamId en_id =
          g == GroupId::GROUP_ID_UART_OUT1 ? ParamId::PARAM_ID_UART_PRESENCE_EN : ParamId::PARAM_ID_UART_TARGET_EN;
      float en = this->value_for_(en_id);
      if (!finite(en)) {
        this->set_error_("cannot set UART output: %s unknown", param_name(en_id));
        return false;
      }
      c.args[0] = en;
      c.nargs = 1;
      // always on-change: a line on every presence change, and the periodic line proves the stream is alive
      float period = this->value_for_(ParamId::PARAM_ID_UART_REPORT_PERIOD);
      if (finite(period)) {
        c.args[1] = REPORT_MODE_ON_CHANGE;
        c.args[2] = period;
        c.nargs = 3;
      } else if (this->snapshot_has_[idx(ParamId::PARAM_ID_UART_REPORT_PERIOD)]) {
        this->set_error_("cannot set UART report period: value unknown");
        if (!this->snapshot_has_[idx(en_id)])
          return false;
      }
      // with no known period (a passive radar) only the enable flag is written and the radar keeps its mode
      return true;
    }
    default:
      return false;
  }
}

void Engine::start_txn_() {
  this->begin_op_errors_();
  this->txn_count_++;
  this->op_ = Op::OP_TXN;
  this->failing_ = false;
  this->set_ok_mask_ = 0;
  this->set_fail_mask_ = 0;
  this->read_ok_mask_ = 0;
  this->read_mask_ = 0;

  uint16_t groups = 0;
  for (uint8_t i = 0; i < PARAM_COUNT; i++) {
    auto p = static_cast<ParamId>(i);
    ParamState &s = this->params_[p];
    if (s.dirty && !this->param_in_current_mode_(p)) {
      s.dirty = false;
      this->set_mode_error_(p, this->work_mode_);
      this->publish_param_(p);
    }
    this->snapshot_has_[i] = s.dirty;
    if (s.dirty) {
      this->snapshot_[i] = s.desired;
      s.dirty = false;
      groups |= gbit(param_spec(p).group);
    }
  }
  groups |= this->couple_ranges_();
  if (this->snapshot_has_[idx(ParamId::PARAM_ID_UART_REPORT_PERIOD)] &&
      this->group_usable_(GroupId::GROUP_ID_UART_OUT2) && finite(this->value_for_(ParamId::PARAM_ID_UART_TARGET_EN))) {
    groups |= gbit(GroupId::GROUP_ID_UART_OUT2);
  }

  Command sets[GROUP_COUNT];
  uint8_t nsets = 0;
  for (uint8_t i = 0; i < GROUP_COUNT; i++) {
    auto g = static_cast<GroupId>(i);
    if ((groups & gbit(g)) == 0)
      continue;
    Command c;
    if (this->build_set_(g, c)) {
      sets[nsets++] = c;
    } else {
      this->set_fail_mask_ |= gbit(g);
    }
  }

  if (nsets == 0) {
    this->logf_(LogLevel::LOG_LEVEL_WARN, "transaction: nothing could be sent");
    for (uint8_t i = 0; i < PARAM_COUNT; i++) {
      if (this->snapshot_has_[i]) {
        this->snapshot_has_[i] = false;
        this->publish_param_(static_cast<ParamId>(i));
      }
    }
    this->op_ = Op::OP_NONE;
    return;
  }

  this->logf_(LogLevel::LOG_LEVEL_DEBUG, "transaction #%u: %u set command(s)", static_cast<unsigned>(this->txn_count_),
              static_cast<unsigned>(nsets));
  this->set_status_(Status::STATUS_APPLYING);
  this->stopped_by_op_ = true;
  this->restart_after_op_ = !this->user_stopped_;
  this->enqueue_(CmdId::CMD_ID_STOP);
  for (uint8_t i = 0; i < nsets; i++)
    this->enqueue_cmd_(sets[i]);
  this->enqueue_(CmdId::CMD_ID_SAVE, GroupId::GROUP_ID_COUNT, CMD_FLAG_OPTIONAL_SAVE);

  uint16_t readback = 0;
  for (uint8_t i = 0; i < GROUP_COUNT; i++) {
    auto g = static_cast<GroupId>(i);
    if ((groups & gbit(g)) != 0 && group_spec(g).get_cmd != nullptr)
      readback |= gbit(g);
  }
  if (this->get_requires_stop_) {
    this->enqueue_gets_(readback);
    if (this->restart_after_op_)
      this->enqueue_(CmdId::CMD_ID_START);
  } else {
    if (this->restart_after_op_)
      this->enqueue_(CmdId::CMD_ID_START);
    this->enqueue_gets_(readback);
  }
  this->enqueue_(CmdId::CMD_ID_MARK_TXN_END);
}

// A trigger distance beyond max_range can never be reached, so a change to one of the two that would leave them
// contradicting each other moves the other with it (C4001 presence app). The partner joins the snapshot, is
// published at once so both sliders move together, and is confirmed by the normal read-back. A value that was
// requested explicitly is never overridden, and a partner that its own limits would keep out of line is left
// alone: the read-back then reports what the radar kept. Returns the groups the coupling added.
uint16_t Engine::couple_ranges_() {
  const ParamId trig_id = ParamId::PARAM_ID_TRIG_RANGE;
  const ParamId max_id = ParamId::PARAM_ID_RANGE_MAX;
  const bool trig_req = this->snapshot_has_[idx(trig_id)];
  const bool max_req = this->snapshot_has_[idx(max_id)];
  if (trig_req == max_req || !this->group_usable_(GroupId::GROUP_ID_TRIG_RANGE) ||
      !this->param_in_current_mode_(trig_id))
    return 0;
  const float trig = this->value_for_(trig_id);
  const float mx = this->value_for_(max_id);
  if (!finite(trig) || !finite(mx))
    return 0;
  static constexpr float EPS = 1e-3f;
  ParamId adj_id;
  float adj;
  if (trig_req) {
    if (trig <= mx + EPS)
      return 0;
    adj_id = max_id;
    adj = trig;
  } else {
    if (mx >= trig - EPS)
      return 0;
    const float mn = this->value_for_(ParamId::PARAM_ID_RANGE_MIN);
    if (finite(mn) && !(mn < mx))
      return 0;  // the range itself is refused; the trigger range stays as it is
    adj_id = trig_id;
    adj = mx;
  }
  const Limits l = param_limits(this->cfg_.model, adj_id);
  if (adj < l.lo)
    adj = l.lo;
  if (adj > l.hi)
    adj = l.hi;
  if (trig_req ? adj < trig - EPS : adj > mx + EPS)
    return 0;
  this->snapshot_[idx(adj_id)] = adj;
  this->snapshot_has_[idx(adj_id)] = true;
  this->host_->on_param(adj_id, adj, true);
  if (trig_req) {
    this->logf_(LogLevel::LOG_LEVEL_INFO, "trigger_range %g pushes max_range up to %g", trig, adj);
  } else {
    this->logf_(LogLevel::LOG_LEVEL_INFO, "max_range %g pulls trigger_range down to %g", mx, adj);
  }
  return gbit(param_spec(adj_id).group);
}

void Engine::start_factory_reset_() {
  this->begin_op_errors_();
  this->txn_count_++;
  this->op_ = Op::OP_FACTORY_RESET;
  this->failing_ = false;
  this->params_.clear_dirty();
  for (uint8_t i = 0; i < PARAM_COUNT; i++) {
    this->snapshot_has_[i] = false;
    this->prev_reported_[i] = this->params_[static_cast<ParamId>(i)].reported;
    this->prev_valid_[i] = this->params_[static_cast<ParamId>(i)].reported_valid;
  }
  this->set_ok_mask_ = 1;
  this->set_fail_mask_ = 0;
  this->read_ok_mask_ = 0;
  this->read_mask_ = 0;
  this->set_status_(Status::STATUS_APPLYING);
  this->logf_(LogLevel::LOG_LEVEL_INFO, "factory reset");
  this->stopped_by_op_ = true;
  this->restart_after_op_ = !this->user_stopped_;
  this->enqueue_(CmdId::CMD_ID_STOP);
  this->enqueue_(CmdId::CMD_ID_RESET_CFG);
  this->enqueue_(CmdId::CMD_ID_SAVE);
  uint16_t readback = this->readable_groups_();
  if (this->get_requires_stop_) {
    this->enqueue_gets_(readback);
    if (this->restart_after_op_)
      this->enqueue_(CmdId::CMD_ID_START);
  } else {
    if (this->restart_after_op_)
      this->enqueue_(CmdId::CMD_ID_START);
    this->enqueue_gets_(readback);
  }
  this->enqueue_(CmdId::CMD_ID_MARK_TXN_END);
}

// C4001 firmware: setRunApp restarts the radar into the other app at once and keeps the choice without
// saveConfig (a saveConfig afterwards answers "sensor is not stopped"). Each app has its own settings and
// version string, so the switch ends with a full identify-and-read instead of a read-back.
void Engine::start_mode_switch_() {
  this->begin_op_errors_();
  ParamState &wm = this->params_[ParamId::PARAM_ID_WORK_MODE];
  wm.dirty = false;
  WorkMode target = wm.desired >= 0.5f ? WorkMode::WORK_MODE_SPEED : WorkMode::WORK_MODE_PRESENCE;
  if (target == this->work_mode_) {
    this->logf_(LogLevel::LOG_LEVEL_DEBUG, "work_mode: radar is already in that mode");
    this->publish_param_(ParamId::PARAM_ID_WORK_MODE);
    return;
  }
  this->logf_(LogLevel::LOG_LEVEL_INFO, "switching to %s mode",
              target == WorkMode::WORK_MODE_SPEED ? "speed_and_distance" : "presence");
  this->op_ = Op::OP_MODE_SWITCH;
  this->failing_ = false;
  this->mode_switch_target_ = target;
  this->mode_switch_pending_ = true;
  this->mode_switch_rejected_ = false;
  this->mode_confirm_pending_ = false;
  this->user_stopped_ = false;
  this->stopped_by_op_ = true;
  this->restart_after_op_ = false;
  this->set_status_(Status::STATUS_APPLYING);

  this->enqueue_(CmdId::CMD_ID_STOP);
  Command run_app;
  run_app.id = CmdId::CMD_ID_SET_GROUP;
  run_app.group = GroupId::GROUP_ID_RUN_APP;
  run_app.args[0] = target == WorkMode::WORK_MODE_SPEED ? 1.0f : 0.0f;
  run_app.nargs = 1;
  this->enqueue_cmd_(run_app);
  this->enqueue_(CmdId::CMD_ID_MARK_MODE_WAIT);
  this->enqueue_(CmdId::CMD_ID_GET_SWV);
  this->enqueue_(CmdId::CMD_ID_MARK_PROBE_DONE);
}

void Engine::finish_mode_switch_() {
  this->mode_switch_pending_ = false;
  this->publish_param_(ParamId::PARAM_ID_WORK_MODE);
  if (this->mode_switch_rejected_) {
    this->mode_switch_rejected_ = false;  // reported when the radar refused it
    return;
  }
  const char *want = this->mode_switch_target_ == WorkMode::WORK_MODE_SPEED ? "speed_and_distance" : "presence";
  if (this->work_mode_ == this->mode_switch_target_) {
    this->logf_(LogLevel::LOG_LEVEL_DEBUG, "work_mode confirmed");
  } else if (this->work_mode_ == WorkMode::WORK_MODE_UNKNOWN) {
    this->set_error_("work_mode: requested %s, but the radar has not reported since", want);
  } else {
    this->set_error_("work_mode: requested %s, radar reports %s", want,
                     this->work_mode_ == WorkMode::WORK_MODE_SPEED ? "speed_and_distance" : "presence");
  }
}

void Engine::finish_txn_() {
  bool factory = this->op_ == Op::OP_FACTORY_RESET;

  // invariant: read-back always publishes the radar's value, whatever was requested
  for (uint8_t i = 0; i < PARAM_COUNT; i++) {
    auto p = static_cast<ParamId>(i);
    if (param_support(this->cfg_.model, p) == Support::SUPPORT_NO || p == ParamId::PARAM_ID_WORK_MODE)
      continue;
    if ((this->read_ok_mask_ & gbit(param_spec(p).group)) == 0)
      continue;
    const ParamState &s = this->params_[p];
    if (factory && this->prev_valid_[i] && std::fabs(this->prev_reported_[i] - s.reported) > 1e-4f)
      this->logf_(LogLevel::LOG_LEVEL_INFO, "factory reset: %s = %g (was %g)", param_name(p), s.reported,
                  this->prev_reported_[i]);
    this->publish_param_(p);  // always, even if changed again meanwhile: the next round republishes
  }

  // compare what was sent with what the radar kept
  for (uint8_t i = 0; i < PARAM_COUNT; i++) {
    if (!this->snapshot_has_[i])
      continue;
    auto p = static_cast<ParamId>(i);
    if (p == ParamId::PARAM_ID_WORK_MODE)
      continue;
    const ParamState &s = this->params_[p];
    const char *name = param_name(p);
    GroupId g = param_spec(p).group;
    if (p == ParamId::PARAM_ID_UART_REPORT_PERIOD && !s.reported_valid && this->report_passive_)
      continue;  // period is meaningless in passive mode
    if ((this->read_ok_mask_ & gbit(g)) == 0 || !s.reported_valid) {
      this->publish_param_(p);  // revert to the last confirmed value
      this->set_error_("%s: could not read the value back, showing the last confirmed value", name);
      continue;
    }
    Limits l = param_limits(this->cfg_.model, p);
    float tol = l.step / 2.0f;
    if (tol < 0.0005f)
      tol = 0.0005f;
    if (std::fabs(s.reported - this->snapshot_[i]) > tol) {
      char req[16];
      char got[16];
      fmt_float(this->snapshot_[i], req, sizeof(req));
      fmt_float(s.reported, got, sizeof(got));
      this->set_error_("%s: requested %s, radar reports %s", name, req, got);
    } else if (this->set_fail_mask_ & gbit(g)) {
      this->logf_(LogLevel::LOG_LEVEL_WARN, "%s: set command failed but the radar already has the requested value",
                  name);
    } else {
      this->logf_(LogLevel::LOG_LEVEL_DEBUG, "%s confirmed", name);
    }
  }

  if (this->save_error_seen_) {
    bool all_read_back = this->read_ok_mask_ != 0;
    for (uint8_t i = 0; i < PARAM_COUNT && all_read_back; i++) {
      auto p = static_cast<ParamId>(i);
      if (this->snapshot_has_[i] && p != ParamId::PARAM_ID_WORK_MODE &&
          (this->read_ok_mask_ & gbit(param_spec(p).group)) == 0)
        all_read_back = false;
    }
    if (all_read_back && this->set_fail_mask_ == 0) {
      this->logf_(LogLevel::LOG_LEVEL_DEBUG,
                  "saveConfig answered Error; radar already holds these values (nothing to save)");
    } else {
      this->set_error_("saveConfig answered Error");
    }
  }

  if (factory && this->dialect_->has_modes) {
    this->mode_confirm_pending_ = true;
    this->mode_confirm_has_request_ = false;
    this->mode_confirm_start_ms_ = this->now_;
  }

  for (bool &b : this->snapshot_has_)
    b = false;
  this->op_ = Op::OP_NONE;
  this->stopped_by_op_ = false;
  this->clear_error_if_clean_();
  this->set_status_(this->running_ || !this->running_known_ ? Status::STATUS_RUNNING : Status::STATUS_STOPPED);
}

void Engine::confirm_mode_() {
  if (!this->mode_confirm_pending_)
    return;
  this->mode_confirm_pending_ = false;
  this->publish_param_(ParamId::PARAM_ID_WORK_MODE);
  if (this->mode_confirm_has_request_) {
    auto requested = this->mode_requested_ >= 0.5f ? WorkMode::WORK_MODE_SPEED : WorkMode::WORK_MODE_PRESENCE;
    if (requested != this->work_mode_) {
      this->set_error_("work_mode: requested %s, radar reports %s",
                       requested == WorkMode::WORK_MODE_SPEED ? "speed_and_distance" : "presence",
                       this->work_mode_ == WorkMode::WORK_MODE_SPEED ? "speed_and_distance" : "presence");
    } else {
      this->logf_(LogLevel::LOG_LEVEL_DEBUG, "work_mode confirmed");
    }
  }
}

void Engine::finish_failure_() {
  for (uint8_t i = 0; i < PARAM_COUNT; i++) {
    if (!this->snapshot_has_[i])
      continue;
    this->snapshot_has_[i] = false;
    auto p = static_cast<ParamId>(i);
    if (!this->params_[p].dirty)
      this->publish_param_(p);  // revert
  }
  this->set_error_("operation failed: %s", this->fail_reason_);
  this->failing_ = false;
  this->stopped_by_op_ = false;
  this->mode_confirm_pending_ = false;
  this->enter_backoff_(this->fail_reason_);
}

void Engine::enter_backoff_(const char *why) {
  this->queue_.clear();
  this->inflight_.active = false;
  this->op_ = Op::OP_NONE;
  this->life_ = Life::LIFE_BACKOFF;
  this->backoff_start_ms_ = this->now_;
  this->backoff_wait_ms_ = this->backoff_ms_;
  this->backoff_ms_ = this->backoff_ms_ * 2 > BACKOFF_MAX_MS ? BACKOFF_MAX_MS : this->backoff_ms_ * 2;
  this->set_link_ok_(false);
  this->set_status_(Status::STATUS_LINK_LOST);
  if (this->ever_running_)
    this->recover_start_ = true;
  this->mode_wait_active_ = false;
  this->mode_switch_pending_ = false;
  this->mode_switch_rejected_ = false;
  this->report_check_armed_ = false;
  this->drop_uart_presence_();
  this->logf_(LogLevel::LOG_LEVEL_WARN, "radar link lost (%s); probing again in %us", why,
              static_cast<unsigned>(this->backoff_wait_ms_ / 1000));
}

void Engine::reset_radar_state_() {
  this->get_requires_stop_ = false;
  this->get_mode_known_ = false;
  this->unsupported_firmware_ = false;
  this->user_stopped_ = false;
  this->prompt_checked_ = false;
  this->mode_confirm_pending_ = false;
  this->work_mode_ = WorkMode::WORK_MODE_UNKNOWN;
  this->running_known_ = false;
  this->recover_start_ = false;
  this->stop_unconfirmed_ = false;
  this->mode_wait_active_ = false;
  this->mode_switch_pending_ = false;
  this->mode_switch_rejected_ = false;
  this->report_check_armed_ = false;
  this->drop_uart_presence_();
  for (uint8_t i = 0; i < PARAM_COUNT; i++)
    this->params_[static_cast<ParamId>(i)].unsupported = false;
}

// The pin stays live (it keeps occupancy working); the UART source is live again with the next report.
void Engine::drop_uart_presence_() {
  if (!this->uart_live_)
    return;
  this->uart_live_ = false;
  this->host_->on_uart_occupancy(Presence::PRESENCE_UNKNOWN);
  this->update_occupancy_();
}

bool Engine::apply_response_(GroupId g, const float *vals, uint8_t nvals) {
  const GroupSpec &gs = group_spec(g);
  uint8_t off = 0;
  if (gs.prefix >= 0 && nvals >= 1 + gs.min_response_args && std::lround(vals[0]) == gs.prefix)
    off = 1;
  if (nvals < off + gs.min_response_args)
    return false;
  uint8_t n = static_cast<uint8_t>(nvals - off);
  const float *v = vals + off;
  auto set = [this](ParamId p, float value) {
    ParamState &s = this->params_[p];
    s.reported = param_spec(p).is_integer ? std::round(value) : value;
    s.reported_valid = true;
    s.reported_at_ms = this->now_;
    s.unsupported = false;
  };
  switch (g) {
    case GroupId::GROUP_ID_RANGE:
      set(ParamId::PARAM_ID_RANGE_MIN, v[0]);
      set(ParamId::PARAM_ID_RANGE_MAX, v[1]);
      break;
    case GroupId::GROUP_ID_TRIG_RANGE:
      set(ParamId::PARAM_ID_TRIG_RANGE, v[0]);
      break;
    case GroupId::GROUP_ID_SENSITIVITY:
      set(ParamId::PARAM_ID_SENSITIVITY, v[0]);
      break;
    case GroupId::GROUP_ID_SENS_SPLIT:
      set(ParamId::PARAM_ID_HOLD_SENS, v[0]);
      set(ParamId::PARAM_ID_TRIG_SENS, v[1]);
      break;
    case GroupId::GROUP_ID_LATENCY:
      set(ParamId::PARAM_ID_LATENCY_ON, v[0]);
      set(ParamId::PARAM_ID_LATENCY_OFF, v[1]);
      break;
    case GroupId::GROUP_ID_INHIBIT:
      set(ParamId::PARAM_ID_INHIBIT, v[0]);
      break;
    case GroupId::GROUP_ID_MICRO_MOTION:
      set(ParamId::PARAM_ID_MICRO_MOTION, v[0]);
      break;
    case GroupId::GROUP_ID_THR_FACTOR:
      set(ParamId::PARAM_ID_THR_FACTOR, v[0]);
      break;
    case GroupId::GROUP_ID_LED:
      // wire value 0 = blink (the factory state), 1 = off; the setting is "LED on"
      set(ParamId::PARAM_ID_LED, v[0] < 0.5f ? 1.0f : 0.0f);
      break;
    case GroupId::GROUP_ID_UART_OUT1: {
      set(ParamId::PARAM_ID_UART_PRESENCE_EN, v[0]);
      const char *stored_mode = nullptr;
      if (n >= 3) {
        this->report_passive_ = v[2] > PASSIVE_PERIOD_MIN;
        if (this->report_passive_) {
          this->params_[ParamId::PARAM_ID_UART_REPORT_PERIOD].reported_valid = false;
          stored_mode = "passive";
        } else {
          set(ParamId::PARAM_ID_UART_REPORT_PERIOD, v[2]);
          if (std::lround(v[1]) != REPORT_MODE_ON_CHANGE)
            stored_mode = "periodic";
        }
      } else if (n == 2 && std::lround(v[1]) != REPORT_MODE_ON_CHANGE) {
        stored_mode = "periodic";
      }
      // the radar's stored state, not a failure: the next write of a UART setting switches it
      if (stored_mode != nullptr && !this->report_mode_logged_) {
        this->report_mode_logged_ = true;
        this->logf_(LogLevel::LOG_LEVEL_INFO,
                    "UART report mode is %s; change the report period once to switch it to on-change", stored_mode);
      }
      break;
    }
    case GroupId::GROUP_ID_UART_OUT2:
      set(ParamId::PARAM_ID_UART_TARGET_EN, v[0]);
      break;
    default:
      return false;
  }
  return true;
}

// sentences, targets, presence

void Engine::on_sentence_(const char *line, const Reply &r) {
  this->last_sentence_ms_ = this->now_;
  this->sentence_seen_ = true;
  this->quiet_logged_ = false;
  this->report_check_armed_ = false;
  if (!this->running_ || !this->running_known_) {
    this->set_running_(true);
    if (this->user_stopped_) {
      this->user_stopped_ = false;
      this->logf_(LogLevel::LOG_LEVEL_INFO, "radar is running again (started externally)");
    }
    if (this->status_ == Status::STATUS_STOPPED)
      this->set_status_(Status::STATUS_RUNNING);
  }

  const bool c4001 = this->dialect_->has_modes;
  bool presence =
      (r.sentence == SentenceId::SENTENCE_ID_JYBSS && !c4001) || (r.sentence == SentenceId::SENTENCE_ID_DFHPD && c4001);
  bool target =
      (r.sentence == SentenceId::SENTENCE_ID_JYRPO && !c4001) || (r.sentence == SentenceId::SENTENCE_ID_DFDMD && c4001);
  if (!presence && !target) {
    if (!this->tag_warned_) {
      this->tag_warned_ = true;
      this->logf_(LogLevel::LOG_LEVEL_WARN, "unexpected report '%.6s' for a %s: check `model:`", line,
                  this->dialect_->name);
    }
    return;
  }
  if (presence) {
    float p;
    if (field_number(line, r, 0, &p)) {
      if (c4001)
        this->set_work_mode_(WorkMode::WORK_MODE_PRESENCE);
      this->set_uart_presence_(p >= 0.5f, true);
    }
  } else {
    if (c4001)
      this->set_work_mode_(WorkMode::WORK_MODE_SPEED);
    this->handle_targets_(line, r);
  }
}

void Engine::handle_targets_(const char *line, const Reply &r) {
  TargetFrame f;
  for (uint8_t i = 0; i < MAX_TARGETS; i++) {
    f.distance[i] = NAN;
    f.snr[i] = NAN;
  }
  f.speed = NAN;
  f.energy = NAN;

  if (r.sentence == SentenceId::SENTENCE_ID_DFDMD) {
    float n;
    if (!field_number(line, r, 0, &n))
      return;
    long count = std::lround(n);
    if (count == 1) {
      float dist;
      float speed;
      float energy;
      if (!field_number(line, r, 2, &dist) || !field_number(line, r, 3, &speed) || !field_number(line, r, 4, &energy)) {
        this->logf_(LogLevel::LOG_LEVEL_VERBOSE, "incomplete $DFDMD dropped");
        return;
      }
      f.count = 1;
      f.distance[0] = dist;
      f.speed = speed;  // signed: positive moves away, negative approaches
      f.energy = energy;
      this->host_->on_targets(f);
      this->last_target_ms_ = this->now_;
      this->targets_zero_published_ = false;
      this->set_uart_presence_(true, true);
    } else if (count == 0) {
      f.count = 0;
      this->host_->on_targets(f);
      this->last_target_ms_ = this->now_;
      this->targets_zero_published_ = true;
      this->set_uart_presence_(false, true);
    } else {
      // the C4001 tracks one target; anything else is a corrupted line and must not keep old data alive
      this->logf_(LogLevel::LOG_LEVEL_VERBOSE, "bad $DFDMD target count %ld, dropped", count);
    }
    return;
  }

  // $JYRPO,n,i,dist, ,snr, , *  one line per target
  float nf;
  float i_f;
  float dist;
  float snr;
  bool ok = field_number(line, r, 0, &nf) && field_number(line, r, 1, &i_f) && field_number(line, r, 2, &dist);
  if (!field_number(line, r, 4, &snr))
    snr = NAN;
  long n = ok ? std::lround(nf) : 0;
  long i = ok ? std::lround(i_f) : 0;
  if (!ok || n < 1 || n > MAX_TARGETS || i < 1 || i > n) {
    this->cycle_n_ = 0;
    this->cycle_filled_ = 0;
    this->logf_(LogLevel::LOG_LEVEL_VERBOSE, "bad $JYRPO, cycle discarded");
    return;
  }
  if (i == 1) {
    this->cycle_n_ = static_cast<uint8_t>(n);
    this->cycle_filled_ = 0;
    this->cycle_ = f;
  } else if (this->cycle_n_ != n || i != this->cycle_filled_ + 1) {
    this->cycle_n_ = 0;
    this->cycle_filled_ = 0;
    this->logf_(LogLevel::LOG_LEVEL_VERBOSE, "$JYRPO out of sequence, cycle discarded");
    return;
  }
  this->cycle_.distance[i - 1] = dist;
  this->cycle_.snr[i - 1] = snr;
  this->cycle_filled_ = static_cast<uint8_t>(i);
  if (i == n) {
    this->cycle_.count = static_cast<uint8_t>(n);
    this->host_->on_targets(this->cycle_);
    this->last_target_ms_ = this->now_;
    this->targets_zero_published_ = false;
    this->cycle_n_ = 0;
    this->cycle_filled_ = 0;
  }
}

void Engine::publish_idle_targets_() {
  TargetFrame f;
  // C4001 firmware: the presence app sends no target reports at all
  f.available = !(this->dialect_->has_modes && this->work_mode_ == WorkMode::WORK_MODE_PRESENCE);
  for (uint8_t i = 0; i < MAX_TARGETS; i++) {
    f.distance[i] = NAN;
    f.snr[i] = NAN;
  }
  f.speed = NAN;
  f.energy = NAN;
  f.count = 0;
  this->host_->on_targets(f);
}

// $JYRPO has no "no target" form, and $DFDMD may simply stop, so target data decays after target_timeout.
void Engine::check_targets_decay_() {
  bool speed = this->dialect_->has_modes && this->work_mode_ == WorkMode::WORK_MODE_SPEED;
  if ((!this->dialect_->has_multi_target && !speed) || this->targets_zero_published_ ||
      this->life_ == Life::LIFE_BOOT_WAIT)
    return;
  if (this->now_ - this->last_target_ms_ >= this->cfg_.target_timeout_ms) {
    this->targets_zero_published_ = true;
    this->cycle_n_ = 0;
    this->cycle_filled_ = 0;
    this->publish_idle_targets_();
    if (speed)
      this->set_uart_presence_(false, false);
  }
}

// C4001: one start if the radar stays silent after an identify-and-read although it should report.
void Engine::check_report_after_recovery_() {
  if (!this->report_check_armed_ || this->now_ < this->report_check_deadline_ms_)
    return;
  if (this->op_ != Op::OP_NONE || this->life_ != Life::LIFE_RUNNING || this->inflight_.active || !this->queue_.empty())
    return;
  this->report_check_armed_ = false;
  if (this->user_stopped_)
    return;
  this->logf_(LogLevel::LOG_LEVEL_INFO, "no reports after recovery; starting the radar");
  this->op_ = Op::OP_RUN_CTRL;
  this->enqueue_(CmdId::CMD_ID_START, GroupId::GROUP_ID_COUNT, CMD_FLAG_RECOVERY);
  this->enqueue_(CmdId::CMD_ID_MARK_RUN_DONE);
}

void Engine::set_work_mode_(WorkMode m) {
  if (!this->dialect_->has_modes)
    return;
  if (this->mode_sentences_since_start_ < 0xFFFFFFFFu)
    this->mode_sentences_since_start_++;
  ParamState &s = this->params_[ParamId::PARAM_ID_WORK_MODE];
  if (m != this->work_mode_ || !s.reported_valid) {
    this->work_mode_ = m;
    s.reported = m == WorkMode::WORK_MODE_SPEED ? 1.0f : 0.0f;
    s.reported_valid = true;
    s.reported_at_ms = this->now_;
    this->logf_(LogLevel::LOG_LEVEL_INFO, "radar is in %s mode",
                m == WorkMode::WORK_MODE_SPEED ? "speed_and_distance" : "presence");
    if (!s.dirty && !this->mode_confirm_pending_ && !this->mode_switch_pending_ && this->op_ != Op::OP_TXN &&
        this->op_ != Op::OP_FACTORY_RESET && this->op_ != Op::OP_MODE_SWITCH)
      this->publish_param_(ParamId::PARAM_ID_WORK_MODE);
    // settings of the other mode become unavailable, those of this mode show their last value until re-read
    for (uint8_t i = 0; i < PARAM_COUNT; i++) {
      auto p = static_cast<ParamId>(i);
      const ParamState &ps = this->params_[p];
      if (p != ParamId::PARAM_ID_WORK_MODE && param_spec(p).modes != MODE_MASK_BOTH && ps.reported_valid && !ps.dirty)
        this->publish_param_(p);
    }
    if (m == WorkMode::WORK_MODE_PRESENCE)
      this->publish_idle_targets_();
    // switched outside our control, or learned after the last read chose its groups (possibly during that
    // read): the settings of this mode still have to be read; schedule_() runs it when idle
    if (m != this->last_read_mode_ && !this->mode_switch_pending_)
      this->mode_read_req_ = true;
    this->update_occupancy_();
  }
  if (this->op_ == Op::OP_MODE_SWITCH && m == this->mode_switch_target_)
    this->mode_wait_seen_ = true;
  if (this->mode_confirm_pending_ && this->op_ == Op::OP_NONE)
    this->confirm_mode_();
}

// `from_report`: a report line carried this value, so the source is live again for another timeout.
void Engine::set_uart_presence_(bool present, bool from_report) {
  if (from_report) {
    this->last_uart_report_ms_ = this->now_;
  } else if (!this->uart_live_) {
    return;
  }
  if (this->uart_live_ && this->uart_present_ == present)
    return;
  this->uart_live_ = true;
  this->uart_present_ = present;
  this->host_->on_uart_occupancy(present ? Presence::PRESENCE_DETECTED : Presence::PRESENCE_CLEAR);
  this->update_occupancy_();
}

// Occupancy is on when any live source reports presence, off when every live source reports absence, and
// unknown when no source is live.
void Engine::update_occupancy_() {
  bool any_live = false;
  bool any_present = false;
  if (this->pin_live_) {
    any_live = true;
    any_present |= this->pin_present_;
  }
  if (this->uart_live_) {
    any_live = true;
    any_present |= this->uart_present_;
  }
  Presence p = !any_live     ? Presence::PRESENCE_UNKNOWN
               : any_present ? Presence::PRESENCE_DETECTED
                             : Presence::PRESENCE_CLEAR;
  if (p == this->occupancy_)
    return;
  this->occupancy_ = p;
  this->host_->on_occupancy(p);
}

uint32_t Engine::uart_presence_timeout_ms() const {
  float period = 1.0f;
  // C4001 firmware: the speed app streams $DFDMD about ten times a second whatever the report period says
  bool speed = this->dialect_->has_modes && this->work_mode_ == WorkMode::WORK_MODE_SPEED;
  const ParamState &ps = this->params_[ParamId::PARAM_ID_UART_REPORT_PERIOD];
  if (!speed && ps.reported_valid && ps.reported > 0.0f && ps.reported <= 1500.0f)
    period = ps.reported;
  return static_cast<uint32_t>(period * 3000.0f) + 1000;
}

// A UART presence value is only current while reports keep arriving; a stopped radar or a dead link must not
// hold occupancy. Our own operations pause the reports briefly, so the window restarts when one ends.
void Engine::check_uart_presence_stale_() {
  if (!this->uart_live_)
    return;
  if (this->op_ == Op::OP_TXN || this->op_ == Op::OP_FACTORY_RESET || this->op_ == Op::OP_MODE_SWITCH ||
      this->mode_switch_pending_ || (this->op_ != Op::OP_NONE && this->stopped_by_op_)) {
    this->last_uart_report_ms_ = this->now_;
    return;
  }
  if (this->now_ - this->last_uart_report_ms_ <= this->uart_presence_timeout_ms())
    return;
  this->logf_(LogLevel::LOG_LEVEL_DEBUG, "no UART report for %u s; UART presence source dropped (%s)",
              static_cast<unsigned>((this->now_ - this->last_uart_report_ms_) / 1000),
              this->cfg_.has_pin ? "pin only" : "no presence source left");
  this->drop_uart_presence_();
}

void Engine::check_link_() {
  if (this->life_ != Life::LIFE_RUNNING || this->op_ != Op::OP_NONE || this->inflight_.active || !this->queue_.empty())
    return;
  if (this->now_ - this->last_rx_ms_ < LINK_TIMEOUT_MS)
    return;
  // Checked whatever the report settings say: a stopped radar, reports turned off, passive mode or a long
  // report period are all silent by design, and a streaming radar never gets here. Silence alone is not
  // proof, so ask once; only a ping that times out takes the link down (see on_done_).
  this->op_ = Op::OP_PING;
  Command ping;
  ping.id = CmdId::CMD_ID_GET_SWV;
  ping.retries = 1;
  this->enqueue_cmd_(ping);
  this->enqueue_(CmdId::CMD_ID_MARK_PING_END);
}

}  // namespace esphome::dfrobot_mmwave::protocol
