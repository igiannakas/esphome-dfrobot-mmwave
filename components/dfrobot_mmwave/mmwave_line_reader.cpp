// Splits the radar byte stream into lines and prompts.

#include "mmwave_line_reader.h"

#include <cstring>

namespace esphome::dfrobot_mmwave::protocol {

// Both firmwares print "<Name>:/>" without a trailing newline, usually followed by one space that is
// not guaranteed (echo lines like "leapMMW:/>getRange" have been seen). The prompt therefore ends at
// '>'. Matching the generic suffix (instead of one dialect's prompt) lets the engine notice a radar
// that is not the configured model.
static constexpr const char PROMPT_SUFFIX[] = ":/>";
static constexpr size_t PROMPT_SUFFIX_LEN = sizeof(PROMPT_SUFFIX) - 1;

void LineReader::reset() {
  this->len_ = 0;
  this->line_len_ = 0;
  this->buf_[0] = '\0';
  this->pending_reset_ = false;
  this->discarding_ = false;
  this->skip_space_ = false;
}

bool LineReader::tail_is_prompt_() const {
  if (this->len_ < PROMPT_SUFFIX_LEN + 1)
    return false;
  return std::memcmp(this->buf_ + this->len_ - PROMPT_SUFFIX_LEN, PROMPT_SUFFIX, PROMPT_SUFFIX_LEN) == 0;
}

LineReader::Event LineReader::feed(uint8_t byte) {
  if (this->skip_space_) {
    this->skip_space_ = false;
    if (byte == ' ')
      return Event::EVENT_NONE;  // the optional space after the prompt
  }
  if (this->pending_reset_) {
    this->len_ = 0;
    this->line_len_ = 0;
    this->buf_[0] = '\0';
    this->pending_reset_ = false;
  }

  if (byte == '\r' || byte == 0)
    return Event::EVENT_NONE;

  if (byte == '\n') {
    if (this->discarding_) {
      this->discarding_ = false;
      this->len_ = 0;
      this->buf_[0] = '\0';
      return Event::EVENT_NONE;
    }
    // strip trailing spaces
    while (this->len_ > 0 && this->buf_[this->len_ - 1] == ' ')
      this->len_--;
    if (this->len_ == 0)
      return Event::EVENT_NONE;  // empty line
    this->buf_[this->len_] = '\0';
    this->line_len_ = this->len_;
    this->pending_reset_ = true;
    return Event::EVENT_LINE;
  }

  if (byte >= 0x7F)
    byte = '?';

  if (this->len_ >= CAPACITY - 1) {
    // Line too long: drop what we have and ignore bytes until the next newline. Prompt detection
    // keeps working on the rolling remainder so a prompt after garbage still resynchronises.
    this->len_ = 0;
    this->buf_[0] = '\0';
    if (!this->discarding_) {
      this->discarding_ = true;
      this->overflows_++;
      this->buf_[this->len_++] = static_cast<char>(byte);
      this->buf_[this->len_] = '\0';
      return Event::EVENT_OVERFLOW;
    }
  }

  this->buf_[this->len_++] = static_cast<char>(byte);
  this->buf_[this->len_] = '\0';

  if (byte == '>' && this->tail_is_prompt_()) {
    // copy the prompt text (the whole buffer: the prompt is printed at the start of a line)
    size_t n = this->len_ < PROMPT_CAPACITY - 1 ? this->len_ : PROMPT_CAPACITY - 1;
    std::memcpy(this->prompt_, this->buf_ + this->len_ - n, n);
    this->prompt_[n] = '\0';
    this->discarding_ = false;
    this->len_ = 0;
    this->line_len_ = 0;
    this->buf_[0] = '\0';
    this->skip_space_ = true;
    return Event::EVENT_PROMPT;
  }
  return Event::EVENT_NONE;
}

}  // namespace esphome::dfrobot_mmwave::protocol
