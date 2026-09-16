#pragma once

// POD command ring buffer and per-parameter state. Pure C++17, no ESPHome includes, no heap.

#include <cmath>
#include <cstdint>

#include "mmwave_params.h"

namespace esphome::dfrobot_mmwave::protocol {

enum class CmdId : uint8_t {
  // wire commands
  CMD_ID_GET_SWV,
  CMD_ID_GET_HWV,
  CMD_ID_GET_RUN_APP,
  CMD_ID_GET_GROUP,
  CMD_ID_SET_GROUP,
  CMD_ID_STOP,
  CMD_ID_START,
  CMD_ID_SAVE,
  CMD_ID_RESET_CFG,
  CMD_ID_RESET_SYSTEM,
  CMD_ID_FLUSH,  // terminator alone: flushes a half-typed line
  // markers: executed when they reach the head of the queue, nothing is sent
  CMD_ID_MARK_PROBE_DONE,
  CMD_ID_MARK_FIRST_READ,
  CMD_ID_MARK_READ_END,
  CMD_ID_MARK_TXN_END,
  CMD_ID_MARK_RUN_DONE,
  CMD_ID_MARK_FAIL_END,
  CMD_ID_MARK_PING_END,
  CMD_ID_MARK_MODE_WAIT,  // pause the queue until the new app reports or MODE_SWITCH_WAIT_MS passes
};

inline bool is_marker(CmdId id) {
  return static_cast<uint8_t>(id) >= static_cast<uint8_t>(CmdId::CMD_ID_MARK_PROBE_DONE);
}

static constexpr uint8_t CMD_FLAG_NONE = 0;
static constexpr uint8_t CMD_FLAG_OPTIONAL_SAVE = 1 << 0;  // skipped if no set of the transaction succeeded
static constexpr uint8_t CMD_FLAG_START_RETRY = 1 << 1;    // start already retried after "can't startSensor"
static constexpr uint8_t CMD_FLAG_FAILURE_PATH = 1 << 2;   // command sent while unwinding a failed operation
static constexpr uint8_t CMD_FLAG_RECOVERY = 1 << 3;       // start sent after a recovery; Error = already running

struct Command {
  CmdId id{CmdId::CMD_ID_FLUSH};
  GroupId group{GroupId::GROUP_ID_COUNT};
  uint8_t nargs{0};
  uint8_t retries{0};
  uint8_t flags{CMD_FLAG_NONE};
  float args[3]{};
};

class CommandQueue {
 public:
  static constexpr uint8_t CAPACITY = 32;

  bool push_back(const Command &c) {
    if (this->count_ >= CAPACITY)
      return false;
    this->items_[(this->head_ + this->count_) % CAPACITY] = c;
    this->count_++;
    return true;
  }
  bool push_front(const Command &c) {
    if (this->count_ >= CAPACITY)
      return false;
    this->head_ = static_cast<uint8_t>((this->head_ + CAPACITY - 1) % CAPACITY);
    this->items_[this->head_] = c;
    this->count_++;
    return true;
  }
  bool empty() const { return this->count_ == 0; }
  uint8_t size() const { return this->count_; }
  const Command &front() const { return this->items_[this->head_]; }
  Command pop_front() {
    Command c = this->items_[this->head_];
    if (this->count_ > 0) {
      this->head_ = static_cast<uint8_t>((this->head_ + 1) % CAPACITY);
      this->count_--;
    }
    return c;
  }
  void clear() {
    this->head_ = 0;
    this->count_ = 0;
  }

 protected:
  Command items_[CAPACITY]{};
  uint8_t head_{0};
  uint8_t count_{0};
};

struct ParamState {
  float reported{NAN};  // what the radar last said
  float desired{NAN};   // what the user last asked for, not yet sent
  uint32_t reported_at_ms{0};
  bool reported_valid{false};
  bool dirty{false};
  bool unsupported{false};  // radar answered Error / not recognized to the capability probe
};

class ParamStore {
 public:
  ParamState &operator[](ParamId id) { return this->p_[idx(id) < PARAM_COUNT ? idx(id) : 0]; }
  const ParamState &operator[](ParamId id) const { return this->p_[idx(id) < PARAM_COUNT ? idx(id) : 0]; }
  bool any_dirty() const {
    for (const auto &p : this->p_) {
      if (p.dirty)
        return true;
    }
    return false;
  }
  void clear_dirty() {
    for (auto &p : this->p_)
      p.dirty = false;
  }
  /// desired if dirty/in snapshot is handled by the caller; this returns reported or NAN.
  float reported_or_nan(ParamId id) const {
    const ParamState &s = (*this)[id];
    return s.reported_valid ? s.reported : NAN;
  }

 protected:
  ParamState p_[PARAM_COUNT]{};
};

}  // namespace esphome::dfrobot_mmwave::protocol
