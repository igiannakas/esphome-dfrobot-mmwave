#pragma once
// Minimal test harness for the host-only protocol tests (no external dependencies).

#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <string>
#include <vector>

namespace mini_test {

struct Registry {
  std::vector<std::pair<const char *, std::function<void()>>> tests;
  int failures = 0;
  int checks = 0;
  static Registry &get() {
    static Registry r;
    return r;
  }
};

struct Register {
  Register(const char *name, std::function<void()> fn) { Registry::get().tests.emplace_back(name, std::move(fn)); }
};

inline void fail(const char *file, int line, const std::string &msg) {
  Registry::get().failures++;
  std::printf("  FAIL %s:%d: %s\n", file, line, msg.c_str());
}

}  // namespace mini_test

#define TEST(name) \
  static void test_##name(); \
  static mini_test::Register reg_##name(#name, test_##name); \
  static void test_##name()

#define CHECK(cond) \
  do { \
    mini_test::Registry::get().checks++; \
    if (!(cond)) \
      mini_test::fail(__FILE__, __LINE__, #cond); \
  } while (0)

#define CHECK_STR(a, b) \
  do { \
    mini_test::Registry::get().checks++; \
    std::string sa(a), sb(b); \
    if (sa != sb) \
      mini_test::fail(__FILE__, __LINE__, "\"" + sa + "\" != \"" + sb + "\""); \
  } while (0)

#define CHECK_NEAR(a, b, eps) \
  do { \
    mini_test::Registry::get().checks++; \
    double va = (a), vb = (b); \
    if (!(std::fabs(va - vb) <= (eps))) \
      mini_test::fail(__FILE__, __LINE__, \
                      std::string(#a) + " = " + std::to_string(va) + ", expected " + std::to_string(vb)); \
  } while (0)

#define CHECK_CONTAINS(haystack, needle) \
  do { \
    mini_test::Registry::get().checks++; \
    std::string h(haystack); \
    if (h.find(needle) == std::string::npos) \
      mini_test::fail(__FILE__, __LINE__, "\"" + h + "\" does not contain \"" + std::string(needle) + "\""); \
  } while (0)
