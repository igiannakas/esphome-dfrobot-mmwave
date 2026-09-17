// Unit tests: LineReader, classify(), formatter, parameter tables.

#include "../mmwave_formatter.h"
#include "../mmwave_line_reader.h"
#include "../mmwave_params.h"
#include "../mmwave_publish_filter.h"
#include "../mmwave_responses.h"
#include "mini_test.h"

using namespace esphome::dfrobot_mmwave::protocol;

namespace {

struct Collected {
  std::vector<std::string> lines;
  std::vector<std::string> prompts;
  int overflows = 0;
};

Collected feed_all(const std::string &bytes) {
  LineReader r;
  Collected c;
  for (unsigned char b : bytes) {
    switch (r.feed(b)) {
      case LineReader::Event::EVENT_LINE:
        c.lines.emplace_back(r.line(), r.length());
        break;
      case LineReader::Event::EVENT_PROMPT:
        c.prompts.emplace_back(r.prompt());
        break;
      case LineReader::Event::EVENT_OVERFLOW:
        c.overflows++;
        break;
      default:
        break;
    }
  }
  return c;
}

Reply cls(const char *line, const char *last = nullptr) { return classify(line, std::strlen(line), last); }

std::string cmd(const char *name, int prefix, std::vector<float> args, uint8_t int_mask) {
  char buf[64];
  size_t n = fmt_command(buf, sizeof(buf), name, prefix, args.data(), static_cast<uint8_t>(args.size()), int_mask);
  return std::string(buf, n);
}

std::string ff(float v) {
  char buf[32];
  size_t n = fmt_float(v, buf, sizeof(buf));
  return std::string(buf, n);
}

}  // namespace

TEST(line_reader_crlf_lf_cr) {
  auto c = feed_all("Done\r\nError\n\rResponse 1 2\r\n\r\n\n");
  CHECK(c.lines.size() == 3);
  CHECK_STR(c.lines[0], "Done");
  CHECK_STR(c.lines[1], "Error");
  CHECK_STR(c.lines[2], "Response 1 2");
}

TEST(line_reader_prompt_without_newline) {
  auto c = feed_all("leapMMW:/> DFRobot:/> ");
  CHECK(c.prompts.size() == 2);
  CHECK_STR(c.prompts[0], "leapMMW:/>");
  CHECK_STR(c.prompts[1], "DFRobot:/>");
  CHECK(c.lines.empty());
}

// Classifies every line of a byte stream the way the engine does.
struct Kinds {
  int prompts = 0, sentences = 0, echo = 0, response = 0, done = 0, unknown = 0;
};

static Kinds kinds_of(const std::string &bytes, const char *last_cmd) {
  LineReader r;
  Kinds k;
  for (unsigned char b : bytes) {
    auto ev = r.feed(b);
    if (ev == LineReader::Event::EVENT_PROMPT) {
      k.prompts++;
    } else if (ev == LineReader::Event::EVENT_LINE) {
      switch (classify(r.line(), r.length(), last_cmd).kind) {
        case ReplyKind::REPLY_KIND_SENTENCE:
          k.sentences++;
          break;
        case ReplyKind::REPLY_KIND_ECHO:
          k.echo++;
          break;
        case ReplyKind::REPLY_KIND_RESPONSE:
          k.response++;
          break;
        case ReplyKind::REPLY_KIND_DONE:
          k.done++;
          break;
        case ReplyKind::REPLY_KIND_UNKNOWN:
          k.unknown++;
          break;
        default:
          break;
      }
    }
  }
  return k;
}

TEST(line_reader_prompt_trailing_space_optional) {
  Kinds a = kinds_of("leapMMW:/> $JYBSS,1, , , *\r\n", nullptr);
  CHECK(a.prompts == 1 && a.sentences == 1 && a.unknown == 0);
  Kinds b = kinds_of("leapMMW:/>$JYBSS,1, , , *\r\n", nullptr);
  CHECK(b.prompts == 1 && b.sentences == 1 && b.unknown == 0);
  Kinds c = kinds_of("leapMMW:/>getRange\r\nResponse 0 6\r\n", "getRange");
  CHECK(c.prompts == 1 && c.echo == 1 && c.response == 1 && c.unknown == 0);
  Kinds d = kinds_of("DFRobot:/>Done\r\n", nullptr);
  CHECK(d.prompts == 1 && d.done == 1);
  Kinds e = kinds_of("DFRobot:/> getRange\r\n", "getRange");
  CHECK(e.prompts == 1 && e.echo == 1);
  // only one space is swallowed, and only right after the prompt
  auto f = feed_all("DFRobot:/>\r\nDone\r\nDFRobot:/>  x\r\n");
  CHECK(f.prompts.size() == 2);
  CHECK(f.lines.size() == 2);
  CHECK_STR(f.lines[0], "Done");
  CHECK_STR(f.lines[1], " x");
  // a lone ":/>" with nothing before it is not a prompt
  auto g = feed_all(":/>Done\r\n");
  CHECK(g.prompts.empty());
}

TEST(line_reader_prompt_then_sentence_same_line) {
  auto c = feed_all("DFRobot:/> $DFHPD,1, , , *\r\n");
  CHECK(c.prompts.size() == 1);
  CHECK(c.lines.size() == 1);
  CHECK_STR(c.lines[0], "$DFHPD,1, , , *");
}

TEST(line_reader_overflow_resyncs) {
  std::string garbage(200, 'x');
  auto c = feed_all(garbage + "\r\nDone\r\n" + std::string(150, 'y') + "DFRobot:/> Error\r\n");
  CHECK(c.overflows == 2);
  CHECK(c.lines.size() == 2);
  CHECK_STR(c.lines[0], "Done");
  CHECK_STR(c.lines[1], "Error");
  CHECK(c.prompts.size() == 1);
}

TEST(line_reader_high_bytes_and_trailing_spaces) {
  auto c = feed_all(std::string("Do\xff\x80ne   \r\n"));
  CHECK(c.lines.size() == 1);
  CHECK_STR(c.lines[0], "Do??ne");
}

TEST(line_reader_interleaved_sentence_in_exchange) {
  auto c = feed_all("getRange\r\n$JYBSS,1, , , *\r\nResponse 0 6\r\nDone\r\nleapMMW:/> ");
  CHECK(c.lines.size() == 4);
  CHECK_STR(c.lines[1], "$JYBSS,1, , , *");
  CHECK(c.prompts.size() == 1);
}

TEST(classify_reply_vocabulary) {
  CHECK(cls("Done").kind == ReplyKind::REPLY_KIND_DONE);
  CHECK(cls("Error").kind == ReplyKind::REPLY_KIND_ERROR);
  CHECK(cls("save cfg complete").kind == ReplyKind::REPLY_KIND_SAVE_COMPLETE);
  CHECK(cls("factory reset complete").kind == ReplyKind::REPLY_KIND_FACTORY_RESET_DONE);
  CHECK(cls("sensor stopped already").kind == ReplyKind::REPLY_KIND_ALREADY_STOPPED);
  CHECK(cls("sensor started already").kind == ReplyKind::REPLY_KIND_ALREADY_STARTED);
  CHECK(cls("sensor is not stopped").kind == ReplyKind::REPLY_KIND_NOT_STOPPED);
  CHECK(cls("no parameter has changed").kind == ReplyKind::REPLY_KIND_NOTHING_TO_SAVE);
  CHECK(cls("new parameter isn't save, can't startSensor").kind == ReplyKind::REPLY_KIND_UNSAVED_CANT_START);
  CHECK(cls("fooBar is not recognized as a CLI command").kind == ReplyKind::REPLY_KIND_UNRECOGNIZED);
  CHECK(cls("reset enter bootloader!").kind == ReplyKind::REPLY_KIND_UNKNOWN);
  CHECK(cls("some noise").kind == ReplyKind::REPLY_KIND_UNKNOWN);
  Reply hw = cls("HardwareVersion:JYSJ_409_A01");
  CHECK(hw.kind == ReplyKind::REPLY_KIND_VERSION_HW);
  CHECK_STR(std::string("HardwareVersion:JYSJ_409_A01").substr(hw.text_off, hw.text_len), "JYSJ_409_A01");
  Reply sw = cls("SoftwareVersion:JYSJ_02.07.05.001128");
  CHECK(sw.kind == ReplyKind::REPLY_KIND_VERSION_SW);
  CHECK(sw.text_len == std::strlen("JYSJ_02.07.05.001128"));
}

TEST(classify_response_arity) {
  Reply r0 = cls("Response");
  CHECK(r0.kind == ReplyKind::REPLY_KIND_RESPONSE && r0.nvals == 0);
  Reply r1 = cls("Response 7");
  CHECK(r1.nvals == 1 && r1.vals[0] == 7.0f);
  Reply r2 = cls("Response 0.6 6");
  CHECK(r2.nvals == 2);
  CHECK_NEAR(r2.vals[0], 0.6, 1e-6);
  Reply r4 = cls("Response 1 1 0 1.5");
  CHECK(r4.nvals == 4 && !r4.bad_value);
  CHECK_NEAR(r4.vals[3], 1.5, 1e-6);
  Reply r5 = cls("Response 1 2 3 4 5");
  CHECK(r5.nvals == 4);
  Reply bad = cls("Response 1 abc 3");
  CHECK(bad.nvals == 2 && bad.bad_value);
  Reply nan = cls("Response nan inf");
  CHECK(nan.nvals == 0 && nan.bad_value);
}

TEST(classify_sentences) {
  const char *hpd = "$DFHPD,1, , , *";
  Reply a = cls(hpd);
  CHECK(a.kind == ReplyKind::REPLY_KIND_SENTENCE && a.sentence == SentenceId::SENTENCE_ID_DFHPD);
  float v = -1;
  CHECK(field_number(hpd, a, 0, &v) && v == 1.0f);
  CHECK(!field_number(hpd, a, 1, &v));

  const char *dmd = "$DFDMD,1,1,1.817,0.129,15304, , *";
  Reply d = cls(dmd);
  CHECK(d.sentence == SentenceId::SENTENCE_ID_DFDMD && d.nfields == 7);
  CHECK(field_number(dmd, d, 2, &v));
  CHECK_NEAR(v, 1.817, 1e-5);
  CHECK(field_number(dmd, d, 4, &v) && v == 15304.0f);

  const char *idle = "$DFDMD,0, , , , , *";
  Reply i = cls(idle);
  CHECK(i.sentence == SentenceId::SENTENCE_ID_DFDMD && i.nfields == 6);
  CHECK(field_number(idle, i, 0, &v) && v == 0.0f);
  CHECK(!field_number(idle, i, 2, &v));
  CHECK(!field_number(idle, i, 9, &v));

  CHECK(cls("$DFDMD,1,1,1.8").kind == ReplyKind::REPLY_KIND_UNKNOWN);  // truncated, no '*'
  CHECK(cls("$*").kind == ReplyKind::REPLY_KIND_UNKNOWN);

  const char *rpo = "$JYRPO,8,8,4.281, ,0.687, , *";
  Reply p = cls(rpo);
  CHECK(p.sentence == SentenceId::SENTENCE_ID_JYRPO);
  CHECK(field_number(rpo, p, 1, &v) && v == 8.0f);
  CHECK(field_number(rpo, p, 4, &v));
  CHECK_NEAR(v, 0.687, 1e-5);
  CHECK(cls("$ABCDE,1*").sentence == SentenceId::SENTENCE_ID_OTHER);
}

TEST(classify_echo) {
  CHECK(cls("getRange", "getRange").kind == ReplyKind::REPLY_KIND_ECHO);
  CHECK(cls("setRange 0.6 6", "setRange 0.6 6").kind == ReplyKind::REPLY_KIND_ECHO);
  CHECK(cls("Done", "getRange").kind == ReplyKind::REPLY_KIND_DONE);
  CHECK(cls("getRange", "").kind == ReplyKind::REPLY_KIND_UNKNOWN);
}

TEST(formatter_floats) {
  CHECK_STR(ff(6.0f), "6");
  CHECK_STR(ff(0.025f), "0.025");
  CHECK_STR(ff(2.8f), "2.8");
  CHECK_STR(ff(9.45f), "9.45");
  CHECK_STR(ff(1500.0f), "1500");
  CHECK_STR(ff(0.0f), "0");
  CHECK_STR(ff(-0.0001f), "0");
  CHECK_STR(ff(0.6f), "0.6");
}

TEST(formatter_commands_match_datasheet) {
  CHECK_STR(cmd("setSensitivity", -1, {255, 3}, 0b11), "setSensitivity 255 3");
  CHECK_STR(cmd("setLatency", -1, {0.5f, 15}, 0), "setLatency 0.5 15");
  CHECK_STR(cmd("setLatency", -1, {0.025f, 15}, 0), "setLatency 0.025 15");
  CHECK_STR(cmd("setLedMode", 1, {0}, 0b1), "setLedMode 1 0");
  CHECK_STR(cmd("setUartOutput", 1, {1, 1, 5}, 0b011), "setUartOutput 1 1 1 5");
  CHECK_STR(cmd("setRange", -1, {0.6f, 6}, 0), "setRange 0.6 6");
  CHECK_STR(cmd("setThrFactor", -1, {5.4f}, 0b1), "setThrFactor 5");
  CHECK_STR(cmd("getLedMode", 1, {}, 0), "getLedMode 1");
  CHECK_STR(cmd("sensorStart", -1, {1}, 0b1), "sensorStart 1");
  CHECK_STR(cmd("setInhibit", -1, {0.5f}, 0), "setInhibit 0.5");
  std::string longname(70, 'a');
  CHECK(cmd(longname.c_str(), -1, {}, 0).empty());
  char small[8];
  float a = 1;
  CHECK(fmt_command(small, sizeof(small), "setRange", -1, &a, 1, 0) == 0);
}

TEST(quantize_sen0395_grid) {
  CHECK_NEAR(quantize_down(9.36f, 0.15f), 9.3, 1e-4);
  CHECK_NEAR(quantize_down(0.45f, 0.15f), 0.45, 1e-4);
  CHECK_NEAR(quantize_down(6.0f, 0.15f), 6.0, 1e-4);
  CHECK_NEAR(quantize_down(0.5f, 0.15f), 0.45, 1e-4);
  CHECK_NEAR(quantize_down(9.45f, 0.15f), 9.45, 1e-4);
}

TEST(param_tables) {
  CHECK(param_support(Model::MODEL_SEN0395, ParamId::PARAM_ID_TRIG_RANGE) == Support::SUPPORT_NO);
  CHECK(param_support(Model::MODEL_SEN0609, ParamId::PARAM_ID_TRIG_RANGE) == Support::SUPPORT_YES);
  CHECK(param_support(Model::MODEL_SEN0610, ParamId::PARAM_ID_LED) == Support::SUPPORT_PROBE);
  CHECK(param_support(Model::MODEL_SEN0395, ParamId::PARAM_ID_LED) == Support::SUPPORT_YES);
  CHECK(param_support(Model::MODEL_SEN0609, ParamId::PARAM_ID_UART_TARGET_EN) == Support::SUPPORT_NO);
  CHECK(group_support(Model::MODEL_SEN0609, GroupId::GROUP_ID_UART_OUT1) == Support::SUPPORT_PROBE);
  CHECK(group_modes(Model::MODEL_SEN0609, GroupId::GROUP_ID_MICRO_MOTION) == MODE_MASK_SPEED);
  CHECK(group_modes(Model::MODEL_SEN0395, GroupId::GROUP_ID_LATENCY) == MODE_MASK_BOTH);
  CHECK_NEAR(param_limits(Model::MODEL_SEN0610, ParamId::PARAM_ID_RANGE_MAX).hi, 12.0, 1e-6);
  CHECK_NEAR(param_limits(Model::MODEL_SEN0395, ParamId::PARAM_ID_RANGE_MAX).step, 0.15, 1e-6);
  for (uint8_t i = 0; i < PARAM_COUNT; i++)
    CHECK(param_spec(static_cast<ParamId>(i)).id == static_cast<ParamId>(i));
  for (uint8_t i = 0; i < GROUP_COUNT; i++)
    CHECK(group_spec(static_cast<GroupId>(i)).id == static_cast<GroupId>(i));
}

TEST(nan_repeat_filter_passes_every_finite_reading) {
  // ESPHome's throttle_with_priority: NaN always passes and restarts the window, a finite value inside the
  // window is dropped. The component used to drop repeated values before the filter, so a still target
  // (2 m, 2 m, ...) that followed a NaN never came through.
  NanRepeatFilter f;
  uint32_t last_input = 0;
  bool have_input = false;
  float shown = NAN;
  auto throttle = [&](float v, uint32_t now) {
    if (!have_input || now - last_input >= 500 || std::isnan(v)) {
      have_input = true;
      last_input = now;
      shown = v;
    }
  };
  uint32_t published_nan = 0;
  for (uint32_t t = 0; t <= 300; t += 100) {  // no target for a while
    if (f.next(NAN)) {
      published_nan++;
      throttle(NAN, t);
    }
  }
  CHECK(published_nan == 1);
  for (uint32_t t = 400; t <= 2000; t += 100) {  // a still target
    if (f.next(2.0f))
      throttle(2.0f, t);
  }
  CHECK_NEAR(shown, 2.0, 1e-6);
  CHECK(f.next(2.0f));
  CHECK(f.next(NAN));
  CHECK(!f.next(NAN));
}
