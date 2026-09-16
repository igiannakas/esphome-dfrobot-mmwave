// Replay tests: feed a radar session recorded on the bench (bytes exactly as the radar sent them) into the
// engine and check what it publishes. The point is the real line endings, spacing and prompt, not the engine
// logic, which test_engine.cpp covers.
//
// Transcript format, one entry per line ('#' starts a comment):
//   <ms> > "<bytes>"   the component sent this; the replay waits for the engine to send the same bytes and
//                      re-bases the following timestamps on the moment it did
//   <ms> < "<bytes>"   the radar sent this; fed to the engine at <ms>
//   <ms> ! <action>    what the user did: "set <option> <value>" or "factory_reset"
// Quoted bytes understand \r, \n, \" and \\.

#include <fstream>

#include "fake_radar.h"
#include "mini_test.h"

namespace {

struct Entry {
  uint32_t ms{0};
  char dir{0};
  std::string data;
  int line{0};
};

bool unquote(const std::string &s, std::string *out) {
  size_t a = s.find('"');
  size_t b = s.rfind('"');
  if (a == std::string::npos || b <= a)
    return false;
  out->clear();
  for (size_t i = a + 1; i < b; i++) {
    char c = s[i];
    if (c != '\\') {
      out->push_back(c);
      continue;
    }
    if (++i >= b)
      return false;
    switch (s[i]) {
      case 'r':
        out->push_back('\r');
        break;
      case 'n':
        out->push_back('\n');
        break;
      case '"':
      case '\\':
        out->push_back(s[i]);
        break;
      default:
        return false;
    }
  }
  return true;
}

std::vector<Entry> load_transcript(const std::string &name) {
  std::string path(__FILE__);
  path = path.substr(0, path.find_last_of('/') + 1) + "replay/" + name;
  std::ifstream in(path);
  CHECK(in.good());
  std::vector<Entry> entries;
  std::string text;
  int n = 0;
  while (std::getline(in, text)) {
    n++;
    size_t start = text.find_first_not_of(' ');
    if (start == std::string::npos || text[start] == '#')
      continue;
    std::istringstream is(text);
    Entry e;
    e.line = n;
    std::string dir;
    is >> e.ms >> dir;
    bool ok = !is.fail() && dir.size() == 1 && (dir[0] == '>' || dir[0] == '<' || dir[0] == '!');
    if (ok) {
      e.dir = dir[0];
      std::string rest;
      std::getline(is, rest);
      if (e.dir == '!') {
        e.data = rest.substr(rest.find_first_not_of(' ') == std::string::npos ? 0 : rest.find_first_not_of(' '));
        ok = !e.data.empty();
      } else {
        ok = unquote(rest, &e.data) && !e.data.empty();
      }
    }
    if (!ok) {
      mini_test::fail(__FILE__, __LINE__, name + ":" + std::to_string(n) + ": bad entry: " + text);
      continue;
    }
    entries.push_back(e);
  }
  return entries;
}

std::string escaped(const std::string &s) {
  std::string o;
  for (char c : s) {
    if (c == '\r') {
      o += "\\r";
    } else if (c == '\n') {
      o += "\\n";
    } else {
      o += c;
    }
  }
  return o;
}

struct ReplayHost : public RecordingHost {
  std::string tx;
  bool saw_detected = false;
  std::vector<TargetFrame> full_frames;  // frames with at least one target
  void write(const char *data, size_t len) override { this->tx.append(data, len); }
  void on_occupancy(Presence p) override {
    RecordingHost::on_occupancy(p);
    this->saw_detected |= p == Presence::PRESENCE_DETECTED;
  }
  void on_targets(const TargetFrame &f) override {
    RecordingHost::on_targets(f);
    if (f.count > 0)
      this->full_frames.push_back(f);
  }
};

struct Replay {
  ReplayHost host;
  Engine *engine{nullptr};
  uint32_t t{0};

  explicit Replay(Model model) {
    EngineConfig cfg;
    cfg.model = model;
    this->host.clock = &this->t;
    this->engine = new Engine(cfg, &this->host);
    this->engine->set_log_level(LogLevel::LOG_LEVEL_VERBOSE);
    this->engine->begin(0);
  }
  ~Replay() { delete this->engine; }

  void action(const Entry &e) {
    std::istringstream is(e.data);
    std::string verb;
    is >> verb;
    if (verb == "factory_reset") {
      this->engine->request_factory_reset();
      return;
    }
    std::string option;
    float value;
    is >> option >> value;
    if (verb == "set" && !is.fail()) {
      for (uint8_t i = 0; i < PARAM_COUNT; i++) {
        const ParamSpec &spec = param_spec(static_cast<ParamId>(i));
        if (option == spec.name) {
          this->engine->set_desired(spec.id, value, this->t);
          return;
        }
      }
    }
    mini_test::fail(__FILE__, __LINE__, "line " + std::to_string(e.line) + ": unknown action: " + e.data);
  }

  // Returns false on a mismatch between what the engine sent and what the transcript says was sent.
  bool run(const std::vector<Entry> &entries, uint32_t max_ms) {
    int64_t offset = 0;
    size_t i = 0;
    while (i < entries.size() && this->t < max_ms) {
      this->t += 5;
      while (i < entries.size() && entries[i].dir != '>' && static_cast<int64_t>(entries[i].ms) + offset <= this->t) {
        if (entries[i].dir == '!') {
          this->action(entries[i]);
        } else {
          for (char c : entries[i].data)
            this->engine->feed(static_cast<uint8_t>(c), this->t);
        }
        i++;
      }
      this->engine->loop(this->t);
      size_t nl = this->host.tx.find('\n');
      if (nl == std::string::npos)
        continue;
      // the engine sent a command: anything the radar printed before it in the transcript is fed first
      while (i < entries.size() && entries[i].dir == '<') {
        for (char c : entries[i].data)
          this->engine->feed(static_cast<uint8_t>(c), this->t);
        i++;
      }
      const std::string sent = this->host.tx.substr(0, nl + 1);
      this->host.tx.erase(0, nl + 1);
      if (i >= entries.size() || entries[i].dir != '>' || entries[i].data != sent) {
        mini_test::fail(__FILE__, __LINE__,
                        "engine sent \"" + escaped(sent) + "\" at " + std::to_string(this->t) + " ms, transcript" +
                            (i < entries.size() ? " line " + std::to_string(entries[i].line) + " has \"" +
                                                      escaped(entries[i].data) + "\""
                                                : " has ended"));
        return false;
      }
      if (this->host.verbose)
        std::printf("    TX at %u ms, transcript %u ms: %s\n", this->t, entries[i].ms, escaped(sent).c_str());
      offset = static_cast<int64_t>(this->t) - entries[i].ms;
      i++;
    }
    if (i < entries.size()) {
      mini_test::fail(__FILE__, __LINE__,
                      "replay stopped at transcript line " + std::to_string(entries[i].line) + " (" +
                          escaped(entries[i].data) + ")");
      return false;
    }
    return true;
  }
};

}  // namespace

TEST(replay_sen0395_bench) {
  const std::vector<Entry> entries = load_transcript("sen0395_bench.txt");
  CHECK(!entries.empty());
  Replay r(Model::MODEL_SEN0395);
  CHECK(r.run(entries, 120000));
  r.host.tx.clear();

  CHECK_STR(r.host.sw, "JYSJ_02.08.07.010706");
  CHECK_STR(r.host.hw, "JYSJ_403_A01");
  CHECK(r.host.status == Status::STATUS_RUNNING);
  CHECK(r.host.link_ok);
  CHECK(r.host.saw_detected);
  CHECK(r.host.occupancy);
  CHECK(!r.engine->busy());
  CHECK(r.engine->overflow_count() == 0);

  // the one target burst
  CHECK(r.host.full_frames.size() == 1);
  if (!r.host.full_frames.empty()) {
    const TargetFrame &f = r.host.full_frames.front();
    CHECK(f.count == 8);
    CHECK_NEAR(f.distance[0], 1.062, 1e-4);
    CHECK_NEAR(f.snr[0], 24.305, 1e-3);
    CHECK_NEAR(f.distance[7], 5.316, 1e-4);
    CHECK_NEAR(f.snr[7], 3.817, 1e-3);
  }

  // the settings transaction and the factory reset
  CHECK(r.host.logged("radar saved its configuration"));
  CHECK(r.host.logged("radar restored its factory settings"));
  CHECK(r.host.logged("factory reset: max_range = 9.45 (was 6.45)"));
  CHECK(r.host.logged("factory reset: uart_target_report = 0 (was 1)"));
  CHECK(r.engine->save_count() == 2);  // the transaction and the factory reset

  // factory settings as read back at the end
  CHECK_NEAR(r.host.val(ParamId::PARAM_ID_RANGE_MIN), 0, 1e-4);
  CHECK_NEAR(r.host.val(ParamId::PARAM_ID_RANGE_MAX), 9.45, 1e-4);
  CHECK_NEAR(r.host.val(ParamId::PARAM_ID_SENSITIVITY), 7, 1e-4);
  CHECK_NEAR(r.host.val(ParamId::PARAM_ID_LATENCY_ON), 0.025, 1e-4);
  CHECK_NEAR(r.host.val(ParamId::PARAM_ID_LATENCY_OFF), 5, 1e-4);
  CHECK(r.host.val(ParamId::PARAM_ID_LED) == 1);
  CHECK(r.host.val(ParamId::PARAM_ID_UART_PRESENCE_EN) == 1);
  CHECK(r.host.val(ParamId::PARAM_ID_UART_TARGET_EN) == 0);
  CHECK_NEAR(r.host.val(ParamId::PARAM_ID_UART_REPORT_PERIOD), 1, 1e-4);
  CHECK_STR(r.host.last_error, "");
  CHECK(!r.host.logged("unexpected report"));
  CHECK(!r.host.logged("does not match"));
}
