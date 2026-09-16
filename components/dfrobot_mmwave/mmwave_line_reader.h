#pragma once

// Bytes -> lines / prompts. Pure C++17, no ESPHome includes, no heap.

#include <cstddef>
#include <cstdint>

namespace esphome::dfrobot_mmwave::protocol {

class LineReader {
 public:
  static constexpr size_t CAPACITY = 128;  // max line length is CAPACITY - 1
  static constexpr size_t PROMPT_CAPACITY = 24;

  enum class Event : uint8_t { EVENT_NONE, EVENT_LINE, EVENT_PROMPT, EVENT_OVERFLOW };

  /// Feed one byte. After LINE, line()/length() are valid until the next feed().
  /// After PROMPT, prompt() (without the optional trailing space) is valid until the next PROMPT.
  Event feed(uint8_t byte);

  const char *line() const { return this->buf_; }
  size_t length() const { return this->line_len_; }
  const char *prompt() const { return this->prompt_; }
  uint32_t overflow_count() const { return this->overflows_; }
  void reset();

 protected:
  bool tail_is_prompt_() const;

  char buf_[CAPACITY]{};
  char prompt_[PROMPT_CAPACITY]{};
  size_t len_{0};
  size_t line_len_{0};
  uint32_t overflows_{0};
  bool pending_reset_{false};
  bool discarding_{false};
  bool skip_space_{false};  // swallow one ' ' right after a prompt
};

}  // namespace esphome::dfrobot_mmwave::protocol
