// Reply and report sentence classification.

#include "mmwave_responses.h"

#include <cmath>
#include <cstdlib>
#include <cstring>

namespace esphome::dfrobot_mmwave::protocol {

static bool starts_with(const char *line, size_t len, const char *prefix) {
  size_t n = std::strlen(prefix);
  return len >= n && std::memcmp(line, prefix, n) == 0;
}

static bool contains(const char *line, size_t len, const char *needle) {
  size_t n = std::strlen(needle);
  if (n == 0 || len < n)
    return false;
  for (size_t i = 0; i + n <= len; i++) {
    if (std::memcmp(line + i, needle, n) == 0)
      return true;
  }
  return false;
}

static bool equals_trimmed(const char *line, size_t len, const char *other) {
  while (len > 0 && line[0] == ' ') {
    line++;
    len--;
  }
  while (len > 0 && line[len - 1] == ' ')
    len--;
  size_t n = std::strlen(other);
  while (n > 0 && other[n - 1] == ' ')
    n--;
  return n > 0 && len == n && std::memcmp(line, other, n) == 0;
}

bool parse_number(const char *s, size_t len, float *out) {
  while (len > 0 && *s == ' ') {
    s++;
    len--;
  }
  while (len > 0 && s[len - 1] == ' ')
    len--;
  if (len == 0 || len > 23)
    return false;
  char tmp[24];
  std::memcpy(tmp, s, len);
  tmp[len] = '\0';
  // only plain decimal notation (strtof would also accept "nan", "inf", hex)
  for (size_t i = 0; i < len; i++) {
    char c = tmp[i];
    if (!((c >= '0' && c <= '9') || c == '.' || c == '-' || c == '+' || c == 'e' || c == 'E'))
      return false;
  }
  char *end = nullptr;
  float v = std::strtof(tmp, &end);
  if (end != tmp + len || !std::isfinite(v))
    return false;
  *out = v;
  return true;
}

bool field_empty(const char *line, const Field &f) {
  for (uint8_t i = 0; i < f.len; i++) {
    if (line[f.off + i] != ' ')
      return false;
  }
  return true;
}

bool field_number(const char *line, const Reply &r, uint8_t index, float *out) {
  if (index >= r.nfields)
    return false;
  const Field &f = r.fields[index];
  if (field_empty(line, f))
    return false;
  return parse_number(line + f.off, f.len, out);
}

static void parse_response(const char *line, size_t len, Reply &r) {
  static constexpr size_t PREFIX_LEN = 8;  // "Response"
  size_t pos = PREFIX_LEN;
  r.text_off = static_cast<uint8_t>(pos < len ? pos : len);
  while (r.text_off < len && line[r.text_off] == ' ')
    r.text_off++;
  r.text_len = static_cast<uint8_t>(len - r.text_off);
  while (pos < len) {
    while (pos < len && line[pos] == ' ')
      pos++;
    if (pos >= len)
      break;
    size_t start = pos;
    while (pos < len && line[pos] != ' ')
      pos++;
    if (r.nvals >= MAX_RESPONSE_VALUES)
      break;  // extra tokens are ignored
    float v;
    if (parse_number(line + start, pos - start, &v)) {
      r.vals[r.nvals++] = v;
    } else {
      r.bad_value = true;
    }
  }
}

static void parse_sentence(const char *line, size_t len, Reply &r) {
  // $TAG,f0,f1,...,fn*   (fn may be " " followed by '*')
  size_t pos = 1;
  while (pos < len && line[pos] != ',' && line[pos] != '*')
    pos++;
  size_t tag_len = pos - 1;
  if (tag_len == 5) {
    const char *tag = line + 1;
    if (std::memcmp(tag, "JYBSS", 5) == 0) {
      r.sentence = SentenceId::SENTENCE_ID_JYBSS;
    } else if (std::memcmp(tag, "JYRPO", 5) == 0) {
      r.sentence = SentenceId::SENTENCE_ID_JYRPO;
    } else if (std::memcmp(tag, "DFHPD", 5) == 0) {
      r.sentence = SentenceId::SENTENCE_ID_DFHPD;
    } else if (std::memcmp(tag, "DFDMD", 5) == 0) {
      r.sentence = SentenceId::SENTENCE_ID_DFDMD;
    } else {
      r.sentence = SentenceId::SENTENCE_ID_OTHER;
    }
  } else {
    r.sentence = SentenceId::SENTENCE_ID_OTHER;
  }
  if (pos >= len || line[pos] != ',')
    return;                   // no fields
  size_t body_end = len - 1;  // index of '*'
  pos++;
  while (pos <= body_end && r.nfields < MAX_SENTENCE_FIELDS) {
    size_t start = pos;
    while (pos < body_end && line[pos] != ',')
      pos++;
    Field f;
    f.off = static_cast<uint8_t>(start);
    f.len = static_cast<uint8_t>(pos - start);
    r.fields[r.nfields++] = f;
    if (pos >= body_end)
      break;
    pos++;  // skip ','
  }
}

Reply classify(const char *line, size_t len, const char *last_cmd) {
  Reply r;
  if (line == nullptr || len == 0 || len > 255) {
    r.kind = ReplyKind::REPLY_KIND_UNKNOWN;
    return r;
  }
  if (last_cmd != nullptr && last_cmd[0] != '\0' && equals_trimmed(line, len, last_cmd)) {
    r.kind = ReplyKind::REPLY_KIND_ECHO;
    return r;
  }
  if (line[0] == '$') {
    if (len < 7 || line[len - 1] != '*') {
      r.kind = ReplyKind::REPLY_KIND_UNKNOWN;  // truncated sentence
      return r;
    }
    r.kind = ReplyKind::REPLY_KIND_SENTENCE;
    parse_sentence(line, len, r);
    return r;
  }
  if (equals_trimmed(line, len, "Done")) {
    r.kind = ReplyKind::REPLY_KIND_DONE;
    return r;
  }
  if (starts_with(line, len, "Error")) {
    r.kind = ReplyKind::REPLY_KIND_ERROR;
    return r;
  }
  if (starts_with(line, len, "Response")) {
    r.kind = ReplyKind::REPLY_KIND_RESPONSE;
    parse_response(line, len, r);
    return r;
  }
  if (starts_with(line, len, "HardwareVersion") || starts_with(line, len, "SoftwareVersion")) {
    r.kind = line[0] == 'H' ? ReplyKind::REPLY_KIND_VERSION_HW : ReplyKind::REPLY_KIND_VERSION_SW;
    size_t p = 15;
    while (p < len && (line[p] == ':' || line[p] == ' '))
      p++;
    r.text_off = static_cast<uint8_t>(p);
    r.text_len = static_cast<uint8_t>(len - p);
    return r;
  }
  if (contains(line, len, "can't startSensor") || contains(line, len, "isn't save")) {
    r.kind = ReplyKind::REPLY_KIND_UNSAVED_CANT_START;
  } else if (contains(line, len, "save cfg complete")) {
    r.kind = ReplyKind::REPLY_KIND_SAVE_COMPLETE;
  } else if (contains(line, len, "factory reset complete")) {
    r.kind = ReplyKind::REPLY_KIND_FACTORY_RESET_DONE;
  } else if (contains(line, len, "stopped already")) {
    r.kind = ReplyKind::REPLY_KIND_ALREADY_STOPPED;
  } else if (contains(line, len, "started already")) {
    r.kind = ReplyKind::REPLY_KIND_ALREADY_STARTED;
  } else if (contains(line, len, "is not stopped")) {
    r.kind = ReplyKind::REPLY_KIND_NOT_STOPPED;
  } else if (contains(line, len, "no parameter has changed")) {
    r.kind = ReplyKind::REPLY_KIND_NOTHING_TO_SAVE;
  } else if (contains(line, len, "not recognized")) {
    r.kind = ReplyKind::REPLY_KIND_UNRECOGNIZED;
  } else {
    r.kind = ReplyKind::REPLY_KIND_UNKNOWN;
  }
  return r;
}

const char *reply_kind_str(ReplyKind k) {
  switch (k) {
    case ReplyKind::REPLY_KIND_DONE:
      return "Done";
    case ReplyKind::REPLY_KIND_ERROR:
      return "Error";
    case ReplyKind::REPLY_KIND_RESPONSE:
      return "Response";
    case ReplyKind::REPLY_KIND_SENTENCE:
      return "sentence";
    case ReplyKind::REPLY_KIND_VERSION_HW:
      return "hardware version";
    case ReplyKind::REPLY_KIND_VERSION_SW:
      return "software version";
    case ReplyKind::REPLY_KIND_SAVE_COMPLETE:
      return "save complete";
    case ReplyKind::REPLY_KIND_FACTORY_RESET_DONE:
      return "factory reset complete";
    case ReplyKind::REPLY_KIND_ALREADY_STOPPED:
      return "already stopped";
    case ReplyKind::REPLY_KIND_ALREADY_STARTED:
      return "already started";
    case ReplyKind::REPLY_KIND_NOT_STOPPED:
      return "not stopped";
    case ReplyKind::REPLY_KIND_NOTHING_TO_SAVE:
      return "nothing to save";
    case ReplyKind::REPLY_KIND_UNSAVED_CANT_START:
      return "unsaved, can't start";
    case ReplyKind::REPLY_KIND_UNRECOGNIZED:
      return "unrecognized";
    case ReplyKind::REPLY_KIND_ECHO:
      return "echo";
    case ReplyKind::REPLY_KIND_UNKNOWN:
    default:
      return "unknown";
  }
}

}  // namespace esphome::dfrobot_mmwave::protocol
