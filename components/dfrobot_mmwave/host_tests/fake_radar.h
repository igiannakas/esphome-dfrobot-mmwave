#pragma once
// Scripted fake radar speaking the leapMMW / C4001 CLI, plus a recording EngineHost. The C4001 side
// follows what a SEN0609 did on the bench (clamps, replies, two apps), the SEN0395 side what a SEN0395
// (JYSJ_02.08.07.010706) did.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <functional>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include "../mmwave_engine.h"

using namespace esphome::dfrobot_mmwave::protocol;

// Whether the fake radar prints a space after its prompt; main() runs the suite with both.
inline bool g_prompt_space = true;

struct FakeRadar {
  Model model;
  const Dialect &d;
  explicit FakeRadar(Model m) : model(m), d(get_dialect(m)) { this->factory(); }

  // behaviour switches
  bool responsive = true;         // answers commands at all
  bool get_while_running = true;  // get* works while running
  bool echo = true;
  bool legacy = false;                // SEN0395 legacy firmware: new command set not recognized
  bool led_supported = true;          // getLedMode / setLedMode
  bool uart_output_supported = true;  // C4001: getUartOutput / setUartOutput (SEN0395 always)
  bool emit_sentences = true;
  bool deaf_tx = false;              // processes commands and changes state but sends nothing (TX wire cut)
  bool clamp_trigger_range = false;  // optional quirk: trig <= max - min (the real firmware does not do this)
  float inhibit_report_skew = 0.0f;  // quirk: getInhibit reports the stored value plus this (extra decimals)
  bool dirty_on_failed_set = false;  // quirk used to exercise "can't startSensor"
  bool reject_sets = false;          // every set* answers Error
  bool silent_unknown = false;       // unknown commands get no reply at all (not even the prompt)
  std::string reject_set_prefix;     // set commands starting with this answer Error
  // exact command line -> reply lines to send instead of the normal answer, one entry used per command
  std::map<std::string, std::deque<std::vector<std::string>>> scripted;
  // C4001 bench behaviour (SEN0609): saveConfig answers a bare Error when nothing changed, a set that
  // is clamped to the stored value changes nothing, and "save cfg complete" is never printed.
  bool c4001_save_error_if_unchanged = true;
  bool c4001_nothing_to_save_text = false;  // print "no parameter has changed" before the Error
  // SEN0395 bench behaviour: resetCfg prints "factory reset complete" before Done and stores the defaults
  // itself, so the following saveConfig answers "no parameter has changed" + Done.
  bool sen0395_reset_text = true;
  bool prompt_space = g_prompt_space;  // "DFRobot:/> " vs "DFRobot:/>"
  uint32_t reply_delay_ms = 15;
  uint32_t sentence_period_ms = 1000;

  // radar state
  bool running = true;
  bool unsaved = false;
  bool presence = false;
  int run_app = 0;
  int saves = 0;
  float app_range_min[2] = {0, 0}, app_range_max[2] = {0, 0};  // C4001: each app keeps its own range
  float range_min = 0, range_max = 0, trig = 0, sens = 0, hold = 0, trig_sens = 0, lat_on = 0, lat_off = 0, inhibit = 0,
        micro = 0, thr = 0, led = 0;
  float uart1_en = 0, uart_mode = 0, uart_period = 0, uart2_en = 0;
  uint8_t targets = 0;  // SEN0395 target count emitted per cycle
  uint32_t silent_until = 0;
  uint32_t app_boot_ms = 450;  // bench: first $DFDMD about 490 ms after setRunApp, commands ignored meanwhile
  std::vector<std::pair<uint32_t, std::string>> heard;  // every line, including ones ignored while silent

  std::vector<std::string> received;
  std::function<void(const std::string &)> on_cmd;

  // byte scheduling
  std::deque<std::pair<uint32_t, char>> out;
  std::string rx;
  uint32_t now = 0;
  uint32_t last_sentence = 0;

  void factory() {
    if (this->model == Model::MODEL_SEN0395) {  // measured on the bench after resetCfg
      range_min = 0;
      range_max = 9.45f;
      sens = 7;
      lat_on = 0.025f;
      lat_off = 5;
      led = 0;  // on (getLedMode 1 -> "1 0")
      uart1_en = 1;
      uart_mode = 1;
      uart_period = 1;
      uart2_en = 0;
    } else {
      range_min = 0.6f;
      range_max = 6;
      app_range_min[0] = 0.6f;
      app_range_max[0] = 6;
      app_range_min[1] = 0;
      app_range_max[1] = 26;
      trig = 6;
      hold = 7;
      trig_sens = 5;
      lat_on = 0.05f;
      lat_off = 15;
      inhibit = 1;
      micro = 0;
      thr = 5;
      led = 0;  // blink (bench: getLedMode 1 -> "1 0")
      uart1_en = 1;
      uart_mode = 0;
      uart_period = 1;
    }
  }

  void send_line(const std::string &line) { this->send_raw(line + "\r\n"); }
  void send_raw(const std::string &s) {
    if (this->deaf_tx)
      return;
    uint32_t t = this->now + this->reply_delay_ms;
    if (!this->out.empty() && this->out.back().first > t)
      t = this->out.back().first;
    for (char c : s)
      this->out.emplace_back(t, c);
  }
  void prompt() {
    std::string p(this->d.prompt);
    while (!p.empty() && p.back() == ' ')
      p.pop_back();
    this->send_raw(this->prompt_space ? p + " " : p);
  }

  static std::string num(float v) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%.3f", v);
    std::string s(buf);
    while (!s.empty() && s.back() == '0')
      s.pop_back();
    if (!s.empty() && s.back() == '.')
      s.pop_back();
    return s;
  }

  void write(const char *data, size_t len) {
    for (size_t i = 0; i < len; i++) {
      char c = data[i];
      if (c == '\r')
        continue;
      if (c == '\n') {
        std::string line = this->rx;
        this->rx.clear();
        this->command(line);
      } else {
        this->rx.push_back(c);
      }
    }
  }

  bool not_recognized(const std::string &name) {
    if (this->silent_unknown) {
      this->swallow_prompt = true;
      return true;
    }
    this->send_line(name + " is not recognized as a CLI command");
    return true;
  }
  bool swallow_prompt = false;

  void command(const std::string &line) {
    if (!line.empty())
      this->heard.emplace_back(this->now, line);
    if (!this->responsive || this->now < this->silent_until)
      return;
    if (line.empty()) {
      this->prompt();
      return;
    }
    this->received.push_back(line);
    if (this->on_cmd)
      this->on_cmd(line);
    if (this->echo && !this->silent_unknown)
      this->send_line(line);
    auto script = this->scripted.find(line);
    if (script != this->scripted.end() && !script->second.empty()) {
      for (const auto &reply : script->second.front())
        this->send_line(reply);
      script->second.pop_front();
      this->prompt();
      return;
    }
    if ((this->reject_sets && line.rfind("set", 0) == 0) ||
        (!this->reject_set_prefix.empty() && line.rfind(this->reject_set_prefix, 0) == 0)) {
      this->set_error();
      this->prompt();
      return;
    }
    std::istringstream is(line);
    std::string name;
    is >> name;
    std::vector<float> a;
    float f;
    while (is >> f)
      a.push_back(f);
    const std::vector<float> before = this->settings();
    this->handle(name, a);
    if (name.rfind("set", 0) == 0 && this->settings() != before)
      this->unsaved = true;  // only a real change needs saving
    if (name != "resetSystem" && !this->swallow_prompt)
      this->prompt();
    this->swallow_prompt = false;
  }

  bool stopped_or_fail() {
    if (!this->running)
      return true;
    if (this->model == Model::MODEL_SEN0395) {
      this->send_line("sensor is not stopped");
    } else {
      this->send_line("Error");
    }
    return false;
  }

  void response(std::vector<float> v) {
    std::string s = "Response";
    for (float x : v)
      s += " " + num(x);
    this->send_line(s);
    this->send_line("Done");
  }

  bool get_allowed() {
    if (this->running && !this->get_while_running) {
      this->send_line("Error");
      return false;
    }
    return true;
  }

  void set_done() { this->send_line("Done"); }

  void app_store() {
    this->app_range_min[this->run_app] = this->range_min;
    this->app_range_max[this->run_app] = this->range_max;
  }
  void use_app(int app) {  // power up in the given C4001 app
    this->run_app = app;
    this->app_sync();
  }
  void app_sync() {
    if (this->model == Model::MODEL_SEN0395)
      return;
    this->range_min = this->app_range_min[this->run_app];
    this->range_max = this->app_range_max[this->run_app];
  }

  std::vector<float> settings() const {
    return {this->range_min,
            this->range_max,
            this->trig,
            this->sens,
            this->hold,
            this->trig_sens,
            this->lat_on,
            this->lat_off,
            this->inhibit,
            this->micro,
            this->thr,
            this->led,
            this->uart1_en,
            this->uart_mode,
            this->uart_period,
            this->uart2_en,
            static_cast<float>(this->run_app)};
  }

  void set_error() {
    if (this->dirty_on_failed_set)
      this->unsaved = true;
    this->send_line("Error");
  }

  void handle(const std::string &n, const std::vector<float> &a) {
    const bool c4001 = this->model != Model::MODEL_SEN0395;
    if (this->legacy && (n.rfind("get", 0) == 0 || n.rfind("set", 0) == 0 || n == "saveConfig")) {
      this->not_recognized(n);
      return;
    }
    const bool speed_app = c4001 && this->run_app == 1;
    if (n == "getSWV") {
      if (!c4001) {
        this->send_line("SoftwareVersion:JYSJ_02.07.05.001128");
      } else {
        this->send_line(speed_app ? "SoftwareVersion:JYSJ_01.00.08.240220" : "SoftwareVersion:JYSJ_00.00.04.040220");
      }
      this->send_line("Done");
    } else if (n == "getHWV") {
      this->send_line(c4001 ? "HardwareVersion:JYSJ_428_A01_H" : "HardwareVersion:JYSJ_409_A01");
      this->send_line("Done");
    } else if (speed_app && (n == "getTrigRange" || n == "getSensitivity" || n == "getLatency" || n == "getInhibit" ||
                             n == "getUartOutput" || n == "setUartOutput" || n == "setTrigRange" || n == "setLatency" ||
                             n == "setInhibit")) {
      this->send_line("Error");  // presence-app settings
    } else if (c4001 && !speed_app && (n == "getMicroMotion" || n == "getThrFactor")) {
      this->send_line("Error");  // speed-app settings
    } else if (n == "getRunApp" && c4001) {
      this->send_line("Error");
    } else if (n == "sensorStop") {
      if (!this->running) {
        this->send_line("sensor stopped already");
        this->send_line("Done");
      } else {
        this->running = false;
        this->send_line("Done");
      }
    } else if (n == "sensorStart") {
      if (this->unsaved && !c4001) {
        this->send_line("new parameter isn't save, can't startSensor");
      } else if (this->running) {
        this->send_line("sensor started already");
        if (c4001)
          this->send_line("Done");
      } else {
        this->running = true;
        if (a.empty() || a[0] != 1)
          this->presence = false;
        this->send_line("Done");
      }
    } else if (n == "saveConfig") {
      if (c4001 && this->running) {
        this->send_line("sensor is not stopped");
        this->send_line("Error");
      } else if (!this->unsaved && !c4001) {
        this->send_line("no parameter has changed");
        this->send_line("Done");
      } else if (!this->unsaved && this->c4001_save_error_if_unchanged) {
        if (this->c4001_nothing_to_save_text)
          this->send_line("no parameter has changed");
        this->send_line("Error");
      } else {
        this->unsaved = false;
        this->saves++;
        if (!c4001)
          this->send_line("save cfg complete");
        this->send_line("Done");
      }
    } else if (n == "resetCfg") {
      const std::vector<float> before = this->settings();
      this->factory();
      this->app_sync();
      if (c4001) {
        if (this->settings() != before)
          this->unsaved = true;
      } else {
        this->unsaved = false;
        if (this->sen0395_reset_text)
          this->send_line("factory reset complete");
      }
      this->send_line("Done");
    } else if (n == "resetSystem") {
      this->running = true;
      this->silent_until = this->now + 1000;
    } else if (n == "getRange") {
      if (this->get_allowed())
        this->response({this->range_min, this->range_max});
    } else if (n == "setRange" && a.size() == 2) {
      if (!this->stopped_or_fail())
        return;
      float mn = a[0], mx = a[1];
      if (!c4001) {
        mn = std::floor(mn / 0.15f + 1e-3f) * 0.15f;
        mx = std::floor(mx / 0.15f + 1e-3f) * 0.15f;
      }
      float hi = this->model == Model::MODEL_SEN0610 ? 12.0f : (c4001 ? (speed_app ? 26.0f : 25.0f) : 9.45f);
      if (c4001) {
        mn = std::max(mn, speed_app ? 0.0f : 0.3f);  // clamps seen on the bench
        mx = std::min(std::max(mx, this->model == Model::MODEL_SEN0610 ? 1.2f : 2.4f), hi);
      }
      if (mn >= mx || mx > hi + 1e-3f) {
        this->set_error();
        return;
      }
      this->range_min = mn;
      this->range_max = mx;
      this->app_store();
      this->set_done();
    } else if (n == "getTrigRange" && c4001) {
      if (this->get_allowed())
        this->response({this->trig});
    } else if (n == "setTrigRange" && c4001 && a.size() == 1) {
      if (!this->stopped_or_fail())
        return;
      if (this->run_app != 0) {
        this->set_error();
        return;
      }
      float t = std::max(a[0], this->model == Model::MODEL_SEN0610 ? 1.2f : 2.4f);
      if (this->clamp_trigger_range && t > this->range_max - this->range_min)
        t = this->range_max - this->range_min;
      this->trig = t;
      this->set_done();
    } else if (n == "getSensitivity") {
      if (!this->get_allowed())
        return;
      if (c4001) {
        this->response({this->hold, this->trig_sens});
      } else {
        this->response({this->sens});
      }
    } else if (n == "setSensitivity") {
      if (!this->stopped_or_fail())
        return;
      if (c4001 && a.size() == 2) {
        if (a[0] != 255)
          this->hold = a[0];
        if (a[1] != 255)
          this->trig_sens = a[1];
        this->set_done();
      } else if (!c4001 && a.size() == 1) {
        if (a[0] > 9) {
          this->set_error();
          return;
        }
        this->sens = a[0];
        this->set_done();
      } else {
        this->set_error();
      }
    } else if (n == "getLatency") {
      if (this->get_allowed())
        this->response({this->lat_on, this->lat_off});
    } else if (n == "setLatency" && a.size() == 2) {
      if (!this->stopped_or_fail())
        return;
      this->lat_on = c4001 ? std::min(a[0], 2.0f) : a[0];
      this->lat_off = c4001 ? std::max(a[1], 2.0f) : a[1];
      this->set_done();
    } else if (n == "getInhibit" && c4001) {
      if (this->get_allowed())
        this->response({this->inhibit + this->inhibit_report_skew});
    } else if (n == "setInhibit" && c4001 && a.size() == 1) {
      if (!this->stopped_or_fail())
        return;
      this->inhibit = std::min(std::max(a[0], 0.3f), 60.0f);  // clamps seen on the bench
      this->set_done();
    } else if (n == "getMicroMotion" && c4001) {
      if (this->get_allowed())
        this->response({this->micro});
    } else if (n == "setMicroMotion" && c4001 && a.size() == 1) {
      if (!this->stopped_or_fail())
        return;
      if (this->run_app != 1) {
        this->set_error();
        return;
      }
      this->micro = a[0];
      this->set_done();
    } else if (n == "getThrFactor" && c4001) {
      if (this->get_allowed())
        this->response({this->thr});
    } else if (n == "setThrFactor" && c4001 && a.size() == 1) {
      if (!this->stopped_or_fail())
        return;
      this->thr = a[0];
      this->set_done();
    } else if (n == "setRunApp" && c4001 && a.size() == 1) {
      // restarts into the other app at once, persists without saveConfig, prints nothing
      this->app_store();
      this->run_app = a[0] >= 0.5f ? 1 : 0;
      this->app_sync();
      this->running = true;
      this->presence = false;
      this->last_sentence = 0;
      this->silent_until = this->now + this->app_boot_ms;
    } else if (n == "getLedMode" && this->led_supported && a.size() == 1) {
      if (this->get_allowed())
        this->response({1, this->led});
    } else if (n == "setLedMode" && this->led_supported && a.size() == 2) {
      if (!this->stopped_or_fail())
        return;
      this->led = a[1];
      this->set_done();
    } else if (n == "getUartOutput" && (!c4001 || this->uart_output_supported) && a.size() == 1) {
      if (!this->get_allowed())
        return;
      if (a[0] == 1) {
        this->response({1, this->uart1_en, this->uart_mode, this->uart_period});
      } else {
        this->response({2, this->uart2_en, this->uart_mode, this->uart_period});
      }
    } else if (n == "setUartOutput" && (!c4001 || this->uart_output_supported) && a.size() >= 2) {
      if (!this->stopped_or_fail())
        return;
      if (a[0] == 1) {
        this->uart1_en = a[1];
      } else {
        this->uart2_en = a[1];
      }
      if (a.size() == 4) {
        this->uart_mode = a[2];
        this->uart_period = (c4001 && a[3] < 0.2f) ? 0.2f : a[3];  // bench: 0.025 requested, 0.2 kept
      }
      this->set_done();
    } else if (n == "getEcho") {
      if (this->get_allowed())
        this->response({this->echo ? 1.0f : 0.0f});
    } else {
      this->not_recognized(n);
    }
  }

  bool last_presence_line = false;
  void presence_line() {
    this->last_presence_line = this->presence;
    const char *tag = this->model == Model::MODEL_SEN0395 ? "$JYBSS," : "$DFHPD,";
    this->send_line(std::string(tag) + (this->presence ? "1" : "0") + ", , , *");
  }

  void tick(uint32_t t) {
    this->now = t;
    if (!this->emit_sentences || !this->running || t < this->silent_until)
      return;
    const bool presence_app = this->model == Model::MODEL_SEN0395 || this->run_app == 0;
    // passive output (period > 1500 s) prints nothing; on-change mode also prints at once on a change
    const bool presence_reports = presence_app && this->uart1_en >= 0.5f && this->uart_period <= 1500.0f;
    if (presence_reports && this->uart_mode >= 0.5f && this->presence != this->last_presence_line) {
      this->presence_line();
      return;
    }
    uint32_t period = this->sentence_period_ms;
    if (t - this->last_sentence < period)
      return;
    this->last_sentence = t;
    if (this->model == Model::MODEL_SEN0395) {
      if (presence_reports)
        this->presence_line();
      if (this->uart2_en >= 0.5f) {
        for (uint8_t i = 1; i <= this->targets; i++) {
          this->send_line("$JYRPO," + std::to_string(this->targets) + "," + std::to_string(i) + "," + num(0.5f * i) +
                          ", ," + num(1.0f + i) + ", , *");
        }
      }
    } else if (this->run_app == 0) {
      if (presence_reports)  // bench: uart_presence_report off means no $DFHPD at all
        this->presence_line();
    } else if (this->presence) {
      this->send_line("$DFDMD,1,1,1.817,0.129,15304, , *");
    } else {
      this->send_line("$DFDMD,0, , , , , *");
    }
  }

  int count(const std::string &prefix) const {
    int n = 0;
    for (const auto &l : this->received) {
      if (l.rfind(prefix, 0) == 0)
        n++;
    }
    return n;
  }
};

struct RecordingHost : public EngineHost {
  FakeRadar *radar = nullptr;
  bool verbose = std::getenv("MMWAVE_VERBOSE") != nullptr;
  uint32_t *clock = nullptr;

  struct Pub {
    float value = NAN;
    bool valid = false;
    int count = 0;
    std::vector<float> history;
  };
  std::map<ParamId, Pub> params;
  int occupancy_count = 0;
  Presence occupancy_state = Presence::PRESENCE_UNKNOWN;
  Presence uart_state = Presence::PRESENCE_UNKNOWN;
  bool occupancy = false;  // PRESENCE_DETECTED
  bool uart_occupancy = false;
  bool pin_occupancy = false;
  bool pin_published = false;
  bool running = false;
  bool link_ok = false;
  bool link_ok_published = false;
  Status status = Status::STATUS_BOOT_WAIT;
  std::vector<Status> statuses;
  std::string last_error;
  std::string sw, hw;
  std::vector<TargetFrame> frames;
  std::vector<std::string> logs;

  void write(const char *data, size_t len) override { this->radar->write(data, len); }
  void on_param(ParamId id, float value, bool valid) override {
    auto &p = this->params[id];
    p.value = value;
    p.valid = valid;
    p.count++;
    p.history.push_back(valid ? value : NAN);
  }
  void on_occupancy(Presence p) override {
    this->occupancy_state = p;
    this->occupancy = p == Presence::PRESENCE_DETECTED;
    this->occupancy_count++;
  }
  void on_uart_occupancy(Presence p) override {
    this->uart_state = p;
    this->uart_occupancy = p == Presence::PRESENCE_DETECTED;
  }
  void on_pin_occupancy(bool v) override {
    this->pin_occupancy = v;
    this->pin_published = true;
  }
  void on_running(bool v) override { this->running = v; }
  void on_link_ok(bool v) override {
    this->link_ok = v;
    this->link_ok_published = true;
  }
  void on_status(Status s) override {
    this->status = s;
    this->statuses.push_back(s);
  }
  bool last_error_published = false;
  void on_last_error(const char *m) override {
    this->last_error = m;
    this->last_error_published = true;
  }
  void on_version(bool h, const char *t) override { (h ? this->hw : this->sw) = t; }
  void on_targets(const TargetFrame &f) override { this->frames.push_back(f); }
  void log(LogLevel level, const char *m) override {
    this->logs.emplace_back(m);
    if (this->verbose)
      std::printf("    [%6u] %s: %s\n", this->clock ? *this->clock : 0, level <= LogLevel::LOG_LEVEL_WARN ? "W" : "D",
                  m);
  }
  bool has(ParamId id) const { return this->params.count(id) != 0 && this->params.at(id).valid; }
  float val(ParamId id) const { return this->params.count(id) ? this->params.at(id).value : NAN; }
  bool logged(const std::string &needle) const {
    for (const auto &l : this->logs) {
      if (l.find(needle) != std::string::npos)
        return true;
    }
    return false;
  }
};

struct Sim {
  FakeRadar radar;
  RecordingHost host;
  EngineConfig cfg;
  Engine *engine = nullptr;
  uint32_t t = 0;
  bool pin = false;

  explicit Sim(Model m, std::function<void(EngineConfig &, FakeRadar &)> setup = nullptr,
               std::function<void(Engine &)> before_begin = nullptr)
      : radar(m) {
    this->cfg.model = m;
    if (setup)
      setup(this->cfg, this->radar);
    this->host.radar = &this->radar;
    this->host.clock = &this->t;
    this->engine = new Engine(this->cfg, &this->host);
    this->engine->set_log_level(LogLevel::LOG_LEVEL_VERBOSE);
    if (before_begin)
      before_begin(*this->engine);
    this->engine->begin(0);
  }
  ~Sim() { delete this->engine; }

  void step() {
    this->t += 5;
    this->radar.tick(this->t);
    while (!this->radar.out.empty() && this->radar.out.front().first <= this->t) {
      this->engine->feed(static_cast<uint8_t>(this->radar.out.front().second), this->t);
      this->radar.out.pop_front();
    }
    if (this->cfg.has_pin)
      this->engine->feed_pin(this->pin, this->t);
    this->engine->loop(this->t);
  }
  void run(uint32_t ms) {
    uint32_t end = this->t + ms;
    while (this->t < end)
      this->step();
  }
  bool run_until(const std::function<bool()> &cond, uint32_t max_ms) {
    uint32_t end = this->t + max_ms;
    while (this->t < end) {
      this->step();
      if (cond())
        return true;
    }
    return false;
  }
  bool booted(uint32_t max_ms = 30000) {
    return this->run_until(
        [this] {
          return !this->engine->busy() &&
                 (this->host.status == Status::STATUS_RUNNING ||
                  this->host.status == Status::STATUS_UNSUPPORTED_FIRMWARE) &&
                 this->host.link_ok_published;
        },
        max_ms);
  }
  bool idle(uint32_t max_ms = 60000) {
    return this->run_until([this] { return !this->engine->busy() && !this->engine->config_pending(); }, max_ms);
  }
};
