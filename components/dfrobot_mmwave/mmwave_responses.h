#pragma once

// Line classification and bounded tokenisers. Pure C++17, no ESPHome includes, no heap.

#include <cstddef>
#include <cstdint>

namespace esphome::dfrobot_mmwave::protocol {

enum class ReplyKind : uint8_t {
  REPLY_KIND_DONE,
  REPLY_KIND_ERROR,
  REPLY_KIND_RESPONSE,
  REPLY_KIND_SENTENCE,
  REPLY_KIND_VERSION_HW,
  REPLY_KIND_VERSION_SW,
  REPLY_KIND_SAVE_COMPLETE,
  REPLY_KIND_FACTORY_RESET_DONE,
  REPLY_KIND_ALREADY_STOPPED,
  REPLY_KIND_ALREADY_STARTED,
  REPLY_KIND_NOT_STOPPED,
  REPLY_KIND_NOTHING_TO_SAVE,
  REPLY_KIND_UNSAVED_CANT_START,
  REPLY_KIND_UNRECOGNIZED,
  REPLY_KIND_ECHO,
  REPLY_KIND_UNKNOWN,
};

enum class SentenceId : uint8_t {
  SENTENCE_ID_NONE,
  SENTENCE_ID_JYBSS,
  SENTENCE_ID_JYRPO,
  SENTENCE_ID_DFHPD,
  SENTENCE_ID_DFDMD,
  SENTENCE_ID_OTHER
};

struct Field {
  uint8_t off{0};
  uint8_t len{0};
};

static constexpr uint8_t MAX_RESPONSE_VALUES = 4;
static constexpr uint8_t MAX_SENTENCE_FIELDS = 8;

struct Reply {
  ReplyKind kind{ReplyKind::REPLY_KIND_UNKNOWN};
  // RESPONSE
  uint8_t nvals{0};
  bool bad_value{false};  // a Response token was not a finite number
  float vals[MAX_RESPONSE_VALUES]{};
  // SENTENCE: fields after the tag, the terminating '*' removed from the last one
  SentenceId sentence{SentenceId::SENTENCE_ID_NONE};
  uint8_t nfields{0};
  Field fields[MAX_SENTENCE_FIELDS]{};
  // VERSION_* / RESPONSE: text after the prefix (into the classified line)
  uint8_t text_off{0};
  uint8_t text_len{0};
};

/// Classify one complete line (no CR/LF). `last_cmd` may be nullptr.
Reply classify(const char *line, size_t len, const char *last_cmd);

const char *reply_kind_str(ReplyKind k);

/// Strict number parser for a token of `len` bytes: surrounding spaces allowed, nothing else.
bool parse_number(const char *s, size_t len, float *out);
/// True if the field is empty or only spaces (a "reserved" field).
bool field_empty(const char *line, const Field &f);
bool field_number(const char *line, const Reply &r, uint8_t index, float *out);

}  // namespace esphome::dfrobot_mmwave::protocol
