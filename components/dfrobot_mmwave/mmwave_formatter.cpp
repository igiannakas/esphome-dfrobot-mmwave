// Wire string builders.

#include "mmwave_formatter.h"

#include <cmath>
#include <cstdio>
#include <cstring>

namespace esphome::dfrobot_mmwave::protocol {

size_t fmt_float(float value, char *out, size_t cap) {
  if (cap == 0)
    return 0;
  char tmp[32];
  int n = std::snprintf(tmp, sizeof(tmp), "%.3f", static_cast<double>(value));
  if (n <= 0 || static_cast<size_t>(n) >= sizeof(tmp))
    return 0;
  size_t len = static_cast<size_t>(n);
  if (std::memchr(tmp, '.', len) != nullptr) {
    while (len > 0 && tmp[len - 1] == '0')
      len--;
    if (len > 0 && tmp[len - 1] == '.')
      len--;
  }
  if (len == 2 && tmp[0] == '-' && tmp[1] == '0') {  // "-0.000" -> "0"
    tmp[0] = '0';
    len = 1;
  }
  if (len + 1 > cap)
    return 0;
  std::memcpy(out, tmp, len);
  out[len] = '\0';
  return len;
}

size_t fmt_command(char *out, size_t cap, const char *cmd, int prefix, const float *args, uint8_t nargs,
                   uint8_t int_mask) {
  if (out == nullptr || cap == 0 || cmd == nullptr)
    return 0;
  size_t limit = cap - 1 < MAX_COMMAND_LEN ? cap - 1 : MAX_COMMAND_LEN;
  size_t len = std::strlen(cmd);
  if (len > limit)
    return 0;
  std::memcpy(out, cmd, len);
  char num[32];
  if (prefix >= 0) {
    int n = std::snprintf(num, sizeof(num), " %d", prefix);
    if (n <= 0 || len + static_cast<size_t>(n) > limit)
      return 0;
    std::memcpy(out + len, num, static_cast<size_t>(n));
    len += static_cast<size_t>(n);
  }
  for (uint8_t i = 0; i < nargs; i++) {
    size_t n;
    num[0] = ' ';
    if (int_mask & (1u << i)) {
      float v = args[i] < 0.0f ? 0.0f : args[i];
      int k = std::snprintf(num + 1, sizeof(num) - 1, "%u", static_cast<unsigned>(std::lround(v)));
      if (k <= 0)
        return 0;
      n = static_cast<size_t>(k);
    } else {
      n = fmt_float(args[i], num + 1, sizeof(num) - 1);
      if (n == 0)
        return 0;
    }
    n += 1;
    if (len + n > limit)
      return 0;
    std::memcpy(out + len, num, n);
    len += n;
  }
  out[len] = '\0';
  return len;
}

float quantize_down(float value, float step) {
  if (step <= 0.0f)
    return value;
  // small epsilon so 0.45 (0.4499999) is not pushed down a whole step
  float n = std::floor(value / step + 1e-3f);
  float q = n * step;
  // clean representation (3 decimals, as on the wire)
  return std::round(q * 1000.0f) / 1000.0f;
}

}  // namespace esphome::dfrobot_mmwave::protocol
