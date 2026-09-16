// Engine tests against the scripted fake radar: boot, read-back, transactions, coalescing, failures,
// sentences, presence arbitration.

#include "fake_radar.h"
#include "mini_test.h"

using P = ParamId;

TEST(boot_c4001_reads_everything_without_writing) {
  Sim s(Model::MODEL_SEN0609);
  CHECK(s.booted());
  CHECK(s.host.status == Status::STATUS_RUNNING);
  CHECK(s.host.link_ok);
  CHECK_STR(s.host.sw, "JYSJ_00.00.04.040220");
  CHECK_STR(s.host.hw, "JYSJ_428_A01_H");
  CHECK_NEAR(s.host.val(P::PARAM_ID_RANGE_MIN), 0.6, 1e-4);
  CHECK_NEAR(s.host.val(P::PARAM_ID_RANGE_MAX), 6, 1e-4);
  CHECK_NEAR(s.host.val(P::PARAM_ID_TRIG_RANGE), 6, 1e-4);
  CHECK_NEAR(s.host.val(P::PARAM_ID_HOLD_SENS), 7, 1e-4);
  CHECK_NEAR(s.host.val(P::PARAM_ID_TRIG_SENS), 5, 1e-4);
  CHECK_NEAR(s.host.val(P::PARAM_ID_LATENCY_ON), 0.05, 1e-4);
  CHECK_NEAR(s.host.val(P::PARAM_ID_LATENCY_OFF), 15, 1e-4);
  CHECK_NEAR(s.host.val(P::PARAM_ID_INHIBIT), 1, 1e-4);
  CHECK(s.host.has(P::PARAM_ID_LED) && s.host.val(P::PARAM_ID_LED) == 1);  // probe answered; blink = on
  CHECK(s.engine->param_available(P::PARAM_ID_UART_PRESENCE_EN));          // probe answered
  CHECK(s.host.val(P::PARAM_ID_UART_PRESENCE_EN) == 1);
  CHECK(s.host.has(P::PARAM_ID_WORK_MODE) && s.host.val(P::PARAM_ID_WORK_MODE) == 0);
  CHECK(!s.engine->get_requires_stop());
  CHECK(s.radar.count("sensorStop") == 0);
  CHECK(s.radar.count("saveConfig") == 0);
  CHECK(s.radar.count("set") == 0);
  CHECK(s.host.last_error.empty());
  // nothing published twice at boot
  CHECK(s.host.params[P::PARAM_ID_RANGE_MAX].count == 1);
}

TEST(boot_sen0395_values_and_targets_decay) {
  Sim s(Model::MODEL_SEN0395);
  CHECK(s.booted());
  CHECK_NEAR(s.host.val(P::PARAM_ID_SENSITIVITY), 7, 1e-4);
  CHECK_NEAR(s.host.val(P::PARAM_ID_LATENCY_ON), 0.025, 1e-4);
  CHECK(s.host.val(P::PARAM_ID_UART_PRESENCE_EN) == 1);
  CHECK(s.host.val(P::PARAM_ID_UART_TARGET_EN) == 0);
  CHECK_NEAR(s.host.val(P::PARAM_ID_UART_REPORT_PERIOD), 1, 1e-4);
  CHECK(!s.host.has(P::PARAM_ID_WORK_MODE));
  CHECK(!s.host.frames.empty() && s.host.frames.back().count == 0);  // decayed to 0 after target_timeout
  CHECK(std::isnan(s.host.frames.back().distance[0]));
}

TEST(get_requires_stop_detected_and_radar_left_running) {
  Sim s(Model::MODEL_SEN0609, [](EngineConfig &, FakeRadar &r) { r.get_while_running = false; });
  CHECK(s.booted());
  CHECK(s.engine->get_requires_stop());
  CHECK_NEAR(s.host.val(P::PARAM_ID_RANGE_MAX), 6, 1e-4);
  CHECK(s.host.has(P::PARAM_ID_INHIBIT));
  CHECK(s.radar.running);
  CHECK(s.radar.count("sensorStart 1") == 1);
  CHECK(s.radar.count("saveConfig") == 0);
  // no periodic reads when a stop is needed
  s.run(15 * 60 * 1000);
  CHECK(s.radar.count("sensorStop") == 1);
}

TEST(debounce_gives_one_flash_write) {
  Sim s(Model::MODEL_SEN0609);
  CHECK(s.booted());
  s.engine->set_desired(P::PARAM_ID_RANGE_MAX, 5.0f, s.t);
  s.run(200);
  s.engine->set_desired(P::PARAM_ID_RANGE_MAX, 5.5f, s.t);
  s.run(200);
  s.engine->set_desired(P::PARAM_ID_RANGE_MAX, 6.5f, s.t);
  CHECK(s.engine->config_pending());
  CHECK(s.idle());
  CHECK(s.radar.saves == 1);
  CHECK(s.engine->transaction_count() == 1);
  CHECK(s.radar.count("setRange 0.6 6.5") == 1);
  CHECK_NEAR(s.host.val(P::PARAM_ID_RANGE_MAX), 6.5, 1e-4);
  CHECK(s.radar.running);
  CHECK(s.radar.count("sensorStart 1") == 1);
  CHECK(s.host.last_error.empty());
  CHECK(!s.engine->config_pending());
}

TEST(coalescing_acceptance_case) {
  // set A=1 -> txn starts -> set A=2 during TX_SET -> set B=3 during TX_READBACK
  Sim s(Model::MODEL_SEN0609);
  CHECK(s.booted());
  bool a2_sent = false, b_sent = false;
  s.radar.on_cmd = [&](const std::string &line) {
    if (line.rfind("setRange", 0) == 0 && !a2_sent) {
      a2_sent = true;
      s.engine->set_desired(P::PARAM_ID_RANGE_MAX, 4.0f, s.t);
    }
    if (line.rfind("getRange", 0) == 0 && a2_sent && !b_sent && s.engine->transaction_count() == 1) {
      b_sent = true;
      s.engine->set_desired(P::PARAM_ID_LATENCY_OFF, 30.0f, s.t);
    }
  };
  s.engine->set_desired(P::PARAM_ID_RANGE_MAX, 3.0f, s.t);
  CHECK(s.idle());
  CHECK(a2_sent && b_sent);
  CHECK(s.engine->transaction_count() == 2);
  CHECK(s.radar.saves == 2);
  auto &hist = s.host.params[P::PARAM_ID_RANGE_MAX].history;
  CHECK(hist.size() >= 3);                  // boot, round one, round two
  CHECK_NEAR(hist[1], 3.0, 1e-4);           // round one read-back
  CHECK_NEAR(hist.back(), 4.0, 1e-4);       // round two
  CHECK(s.radar.count("setLatency") == 1);  // B written in round two only
  size_t lat_idx = 0, second_stop = 0, stops = 0;
  for (size_t i = 0; i < s.radar.received.size(); i++) {
    if (s.radar.received[i] == "sensorStop" && ++stops == 2)
      second_stop = i;
    if (s.radar.received[i].rfind("setLatency", 0) == 0)
      lat_idx = i;
  }
  CHECK(second_stop > 0 && lat_idx > second_stop);
  CHECK_NEAR(s.radar.lat_off, 30, 1e-4);
  CHECK(s.radar.running);
}

TEST(boot_c4001_without_uart_output_commands) {
  Sim s(Model::MODEL_SEN0609, [](EngineConfig &, FakeRadar &r) { r.uart_output_supported = false; });
  CHECK(s.booted());
  CHECK(!s.engine->param_available(P::PARAM_ID_UART_PRESENCE_EN));  // probe: not recognized
  CHECK(!s.host.has(P::PARAM_ID_UART_PRESENCE_EN));
  CHECK(s.host.has(P::PARAM_ID_RANGE_MAX));
}

TEST(mismatch_publishes_radar_value_and_sets_error) {
  Sim s(Model::MODEL_SEN0609, [](EngineConfig &, FakeRadar &r) { r.clamp_trigger_range = true; });
  CHECK(s.booted());
  s.engine->set_desired(P::PARAM_ID_TRIG_RANGE, 6.0f, s.t);  // radar clamps to max - min = 5.4
  CHECK(s.idle());
  CHECK_NEAR(s.host.val(P::PARAM_ID_TRIG_RANGE), 5.4, 1e-3);
  CHECK_CONTAINS(s.host.last_error, "trigger_range: requested 6, radar reports 5.4");
}

TEST(set_error_reverts_to_radar_value) {
  Sim s(Model::MODEL_SEN0610);
  CHECK(s.booted());
  s.engine->set_desired(P::PARAM_ID_RANGE_MIN, 5.0f, s.t);
  s.engine->set_desired(P::PARAM_ID_RANGE_MAX, 5.0f, s.t);  // min == max: blocked before sending
  CHECK(s.idle());
  CHECK(s.radar.count("setRange") == 0);
  CHECK(s.radar.count("sensorStop") == 0);
  CHECK_NEAR(s.host.val(P::PARAM_ID_RANGE_MAX), 6, 1e-4);
  CHECK_CONTAINS(s.host.last_error, "must be below");
  CHECK_CONTAINS(s.host.last_error, "reverted min_range and max_range");
  // out-of-limit request rejected in set_desired (SEN0610 max 12 m)
  CHECK(!s.engine->set_desired(P::PARAM_ID_RANGE_MAX, 20.0f, s.t));
  CHECK_CONTAINS(s.host.last_error, "outside");
}

TEST(failed_transaction_reverts_and_recovers) {
  Sim s(Model::MODEL_SEN0609);
  CHECK(s.booted());
  s.radar.responsive = false;
  s.engine->set_desired(P::PARAM_ID_RANGE_MAX, 7.0f, s.t);
  CHECK(s.run_until([&] { return s.host.status == Status::STATUS_LINK_LOST; }, 20000));
  CHECK(!s.host.link_ok);
  CHECK_NEAR(s.host.val(P::PARAM_ID_RANGE_MAX), 6, 1e-4);  // reverted
  CHECK_CONTAINS(s.host.last_error, "sensorStop timed out");
  CHECK(!s.engine->config_pending());
  s.radar.responsive = true;
  CHECK(s.run_until([&] { return s.host.status == Status::STATUS_RUNNING && s.host.link_ok; }, 30000));
  CHECK_NEAR(s.host.val(P::PARAM_ID_RANGE_MAX), 6, 1e-4);
  CHECK(s.radar.saves == 0);
  CHECK(s.radar.running);
}

TEST(timeout_mid_transaction_still_restarts_radar) {
  Sim s(Model::MODEL_SEN0609);
  CHECK(s.booted());
  s.radar.on_cmd = [&](const std::string &line) {
    if (line.rfind("setLatency", 0) == 0)
      s.radar.responsive = false;  // stops answering after this command (this one is not answered either)
  };
  s.engine->set_desired(P::PARAM_ID_LATENCY_OFF, 20.0f, s.t);
  CHECK(s.run_until([&] { return s.host.status == Status::STATUS_LINK_LOST; }, 20000));
  CHECK_NEAR(s.host.val(P::PARAM_ID_LATENCY_OFF), 15, 1e-4);
  s.radar.responsive = true;
  s.radar.on_cmd = nullptr;
  CHECK(s.run_until([&] { return s.host.status == Status::STATUS_RUNNING; }, 30000));
  CHECK(s.radar.running);
}

TEST(sen0395_range_quantised_no_false_mismatch) {
  Sim s(Model::MODEL_SEN0395);
  CHECK(s.booted());
  s.engine->set_desired(P::PARAM_ID_RANGE_MIN, 0.5f, s.t);
  CHECK(s.idle(60000));
  CHECK(s.radar.count("setRange 0.45 9.45") == 1);
  CHECK_NEAR(s.host.val(P::PARAM_ID_RANGE_MIN), 0.45, 1e-4);
  CHECK(s.host.last_error.empty());
  CHECK(s.radar.count("sensorStart") == 1);  // plain start on SEN0395
  CHECK(s.radar.count("sensorStart 1") == 0);
}

TEST(sen0395_start_refused_until_saved) {
  // a rejected set leaves this (quirky) radar dirty: saveConfig is skipped because nothing succeeded,
  // sensorStart is refused, the engine saves and starts again
  Sim s(Model::MODEL_SEN0395, [](EngineConfig &, FakeRadar &r) { r.dirty_on_failed_set = true; });
  CHECK(s.booted());
  s.radar.reject_sets = true;
  s.engine->set_desired(P::PARAM_ID_SENSITIVITY, 3.0f, s.t);
  CHECK(s.idle(60000));
  CHECK(s.radar.running);
  CHECK(s.radar.saves == 1);
  CHECK(s.host.logged("saving first"));
  CHECK_NEAR(s.host.val(P::PARAM_ID_SENSITIVITY), 7, 1e-4);
  CHECK_CONTAINS(s.host.last_error, "sensitivity: requested 3, radar reports 7");
}

TEST(legacy_sen0395_firmware_unsupported_but_presence_works) {
  Sim s(Model::MODEL_SEN0395, [](EngineConfig &, FakeRadar &r) { r.legacy = true; });
  CHECK(s.booted());
  CHECK(s.host.status == Status::STATUS_UNSUPPORTED_FIRMWARE);
  s.radar.presence = true;
  s.run(3000);
  CHECK(s.host.occupancy);
  CHECK(!s.engine->set_desired(P::PARAM_ID_SENSITIVITY, 3.0f, s.t));
  s.run(5000);
  CHECK(s.radar.count("set") == 0);
  CHECK(s.radar.running);
}

TEST(work_mode_change_confirmed_from_sentences) {
  Sim s(Model::MODEL_SEN0609, [](EngineConfig &, FakeRadar &r) { r.sentence_period_ms = 200; });
  CHECK(s.booted());
  CHECK(s.host.val(P::PARAM_ID_WORK_MODE) == 0);
  CHECK(!s.engine->set_desired(P::PARAM_ID_MICRO_MOTION, 1.0f, s.t));  // presence mode: rejected
  CHECK_NEAR(s.host.val(P::PARAM_ID_RANGE_MAX), 6, 1e-4);
  size_t first = s.radar.received.size();
  s.engine->set_desired(P::PARAM_ID_WORK_MODE, 1.0f, s.t);
  CHECK(s.idle());
  CHECK(s.radar.run_app == 1);
  CHECK(s.radar.running);
  // its own operation: stop, switch, then only reads (no saveConfig, no sensorStart)
  CHECK(s.radar.received.size() >= first + 3);
  CHECK_STR(s.radar.received[first], "sensorStop");
  CHECK_STR(s.radar.received[first + 1], "setRunApp 1");
  for (size_t i = first + 2; i < s.radar.received.size(); i++)
    CHECK(s.radar.received[i].rfind("get", 0) == 0);
  CHECK(s.radar.count("saveConfig") == 0);
  CHECK(s.radar.count("sensorStart") == 0);
  CHECK(s.radar.count("getUartOutput") == 1);  // boot only: presence-app group skipped in speed mode
  CHECK(s.radar.saves == 0);
  CHECK(s.host.val(P::PARAM_ID_WORK_MODE) == 1);
  CHECK_STR(s.host.last_error, "");
  CHECK(!s.host.logged("not supported by this firmware"));
  CHECK(!s.host.logged("answered Error"));
  CHECK(s.engine->work_mode() == WorkMode::WORK_MODE_SPEED);
  CHECK_STR(s.host.sw, "JYSJ_01.00.08.240220");             // each app has its own version
  CHECK_NEAR(s.host.val(P::PARAM_ID_RANGE_MAX), 26, 1e-4);  // and its own range
  CHECK(s.host.has(P::PARAM_ID_MICRO_MOTION));
  s.radar.presence = true;
  s.run(1000);
  CHECK(!s.host.frames.empty());
  CHECK(s.host.frames.back().count == 1);
  CHECK_NEAR(s.host.frames.back().distance[0], 1.817, 1e-3);
  CHECK_NEAR(s.host.frames.back().speed, 0.129, 1e-3);
  CHECK_NEAR(s.host.frames.back().energy, 15304, 1e-1);
  CHECK(s.host.occupancy);
  s.radar.presence = false;
  s.run(1000);
  CHECK(s.host.frames.back().count == 0 && std::isnan(s.host.frames.back().distance[0]));
  CHECK(!s.host.occupancy);
  CHECK(s.engine->set_desired(P::PARAM_ID_MICRO_MOTION, 1.0f, s.t));
  CHECK(!s.engine->set_desired(P::PARAM_ID_TRIG_RANGE, 3.0f, s.t));
  CHECK(s.idle());
  CHECK(s.host.val(P::PARAM_ID_MICRO_MOTION) == 1);
}

TEST(speed_mode_no_target_is_nan) {
  Sim s(Model::MODEL_SEN0610, [](EngineConfig &, FakeRadar &r) {
    r.use_app(1);
    r.sentence_period_ms = 100;
  });
  CHECK(s.booted());
  s.run(500);
  CHECK(s.host.frames.back().available && s.host.frames.back().count == 0);
  CHECK(std::isnan(s.host.frames.back().distance[0]) && std::isnan(s.host.frames.back().speed));
  CHECK(s.host.val(P::PARAM_ID_WORK_MODE) == 1);
}

TEST(sen0395_target_cycles) {
  Sim s(Model::MODEL_SEN0395, [](EngineConfig &, FakeRadar &r) {
    r.uart2_en = 1;
    r.targets = 3;
  });
  CHECK(s.booted());
  s.run(1500);
  bool found = false;
  for (auto &f : s.host.frames) {
    if (f.count == 3) {
      found = true;
      CHECK_NEAR(f.distance[0], 0.5, 1e-3);
      CHECK_NEAR(f.distance[2], 1.5, 1e-3);
      CHECK_NEAR(f.snr[1], 3.0, 1e-3);
      CHECK(std::isnan(f.distance[3]));
    }
  }
  CHECK(found);
  s.radar.targets = 0;
  s.run(3500);
  CHECK(s.host.frames.back().count == 0);
}

TEST(sen0395_target_cycle_desync_discarded) {
  Sim s(Model::MODEL_SEN0395, [](EngineConfig &, FakeRadar &r) { r.emit_sentences = false; });
  s.run(100);
  size_t before = s.host.frames.size();
  auto feed = [&](const std::string &l) {
    for (char c : l + "\r\n")
      s.engine->feed(static_cast<uint8_t>(c), s.t);
  };
  feed("$JYRPO,2,2,1.0, ,1.0, , *");  // i=2 without i=1
  feed("$JYRPO,2,1,1.0, ,1.0, , *");
  feed("$JYRPO,3,2,1.0, ,1.0, , *");  // n changed mid-cycle
  feed("$JYRPO,9,1,1.0, ,1.0, , *");  // n > 8
  CHECK(s.host.frames.size() == before);
  feed("$JYRPO,2,1,1.0, ,1.0, , *");
  feed("$JYRPO,2,2,2.0, ,1.0, , *");
  CHECK(s.host.frames.size() == before + 1 && s.host.frames.back().count == 2);
}

TEST(occupancy_or_pin_and_uart) {
  Sim s(Model::MODEL_SEN0609, [](EngineConfig &c, FakeRadar &r) {
    c.has_pin = true;
    r.sentence_period_ms = 200;
  });
  CHECK(s.booted());
  CHECK(s.host.occupancy_state == Presence::PRESENCE_CLEAR);
  s.radar.presence = true;  // pin off, UART on
  s.run(500);
  CHECK(s.host.occupancy && s.host.uart_occupancy && !s.host.pin_occupancy);
  s.radar.presence = false;  // pin on, UART off
  s.pin = true;
  s.run(500);
  CHECK(s.host.occupancy && !s.host.uart_occupancy && s.host.pin_occupancy);
  s.pin = false;  // both off
  s.run(50);
  CHECK(s.host.occupancy_state == Presence::PRESENCE_CLEAR);
  s.pin = true;
  s.run(10);
  CHECK(s.host.occupancy);  // the pin is instant
}

TEST(refresh_detects_external_change) {
  Sim s(Model::MODEL_SEN0609);
  CHECK(s.booted());
  s.radar.inhibit = 9;
  s.run(15 * 60 * 1000);
  CHECK_NEAR(s.host.val(P::PARAM_ID_INHIBIT), 1, 1e-4);  // no periodic read
  CHECK(s.radar.count("getInhibit") == 1);
  s.engine->request_refresh();
  s.run(100);
  CHECK(s.idle());
  CHECK_NEAR(s.host.val(P::PARAM_ID_INHIBIT), 9, 1e-4);
  CHECK(s.host.last_error.empty());
  CHECK(s.host.logged("radar reports inhibit_time = 9"));
  CHECK(s.radar.count("sensorStop") == 0);
}

TEST(factory_reset_publishes_defaults) {
  Sim s(Model::MODEL_SEN0609);
  CHECK(s.booted());
  s.engine->set_desired(P::PARAM_ID_INHIBIT, 20.0f, s.t);
  CHECK(s.idle());
  CHECK_NEAR(s.host.val(P::PARAM_ID_INHIBIT), 20, 1e-4);
  s.engine->request_factory_reset();
  s.run(100);
  CHECK(s.idle());
  CHECK_NEAR(s.host.val(P::PARAM_ID_INHIBIT), 1, 1e-4);
  CHECK(s.radar.count("resetCfg") == 1);
  CHECK(s.radar.saves == 2);
  CHECK(s.radar.running);
}

TEST(link_loss_detected_and_recovered) {
  Sim s(Model::MODEL_SEN0609);
  CHECK(s.booted());
  s.radar.responsive = false;
  s.radar.emit_sentences = false;
  CHECK(s.run_until([&] { return s.host.status == Status::STATUS_LINK_LOST; }, 40000));
  CHECK(!s.host.link_ok);
  s.radar.responsive = true;
  s.radar.emit_sentences = true;
  CHECK(s.run_until([&] { return s.host.link_ok; }, 90000));
  CHECK(s.host.status == Status::STATUS_RUNNING);
}

TEST(running_switch_stop_does_not_trigger_link_loss) {
  Sim s(Model::MODEL_SEN0609);
  CHECK(s.booted());
  s.engine->request_running(false);
  CHECK(s.run_until([&] { return s.host.status == Status::STATUS_STOPPED; }, 5000));
  CHECK(!s.radar.running && !s.host.running);
  s.run(60000);
  CHECK(s.host.status == Status::STATUS_STOPPED);
  // a settings change while stopped keeps the radar stopped
  s.engine->set_desired(P::PARAM_ID_INHIBIT, 2.0f, s.t);
  CHECK(s.idle());
  CHECK(!s.radar.running);
  CHECK_NEAR(s.host.val(P::PARAM_ID_INHIBIT), 2, 1e-4);
  s.engine->request_running(true);
  CHECK(s.run_until([&] { return s.host.status == Status::STATUS_RUNNING; }, 5000));
  CHECK(s.radar.running);
}

TEST(restart_reprobes) {
  Sim s(Model::MODEL_SEN0609);
  CHECK(s.booted());
  s.engine->request_restart();
  CHECK(s.run_until([&] { return s.host.status == Status::STATUS_BOOT_WAIT; }, 3000));
  CHECK(s.booted());
  CHECK(s.radar.count("resetSystem 0") == 1);
  CHECK(s.radar.count("getSWV") == 2);
}

TEST(no_answer_at_boot_backs_off_and_flushes) {
  Sim s(Model::MODEL_SEN0609, [](EngineConfig &, FakeRadar &r) { r.responsive = false; });
  s.run(20000);
  CHECK(s.host.status == Status::STATUS_LINK_LOST);
  CHECK(s.host.logged("radar sends reports but does not answer commands"));
  s.radar.presence = true;
  s.run(2000);
  CHECK(s.host.occupancy);  // UART presence still works
  s.radar.responsive = true;
  CHECK(s.booted(90000));
}

TEST(uart_output_write_always_on_change) {
  for (Model m : {Model::MODEL_SEN0395, Model::MODEL_SEN0609}) {
    Sim s(m, [](EngineConfig &, FakeRadar &r) { r.uart_mode = 0; });  // periodic
    CHECK(s.booted());
    s.engine->set_desired(P::PARAM_ID_UART_REPORT_PERIOD, 5.0f, s.t);
    CHECK(s.idle(60000));
    CHECK(s.radar.count("setUartOutput 1 1 1 5") == 1);
    if (m == Model::MODEL_SEN0395)
      CHECK(s.radar.count("setUartOutput 2 0 1 5") == 1);
    CHECK(s.radar.uart_mode == 1);
    CHECK_NEAR(s.host.val(P::PARAM_ID_UART_REPORT_PERIOD), 5, 1e-4);
    CHECK(s.host.last_error.empty());
    // toggling the report also writes on-change mode with the known period
    s.engine->set_desired(P::PARAM_ID_UART_PRESENCE_EN, 0.0f, s.t);
    CHECK(s.idle(60000));
    CHECK(s.radar.count("setUartOutput 1 0 1 5") == 1);
  }
}

TEST(periodic_radar_is_logged_not_flagged) {
  Sim s(Model::MODEL_SEN0609, [](EngineConfig &, FakeRadar &r) {
    r.uart_mode = 0;
    r.sentence_period_ms = 1000;
  });
  CHECK(s.booted());
  CHECK(s.host.logged("UART report mode is periodic; change the report period once to switch it to on-change"));
  CHECK_STR(s.host.last_error, "");
  s.engine->request_refresh();
  s.run(100);
  CHECK(s.idle());
  int lines = 0;
  for (const auto &l : s.host.logs)
    lines += l.find("UART report mode is") != std::string::npos;
  CHECK(lines == 1);  // once per boot
  s.run(1500);
  s.radar.presence = true;
  CHECK(s.run_until([&] { return s.host.occupancy; }, 1500));  // with lag, up to one period
  CHECK(s.radar.count("setUartOutput") == 0);                  // nothing written at boot
}

TEST(on_change_radar_reports_at_once) {
  Sim s(Model::MODEL_SEN0609, [](EngineConfig &, FakeRadar &r) {
    r.uart_mode = 1;
    r.sentence_period_ms = 1000;
  });
  CHECK(s.booted());
  CHECK(!s.host.logged("UART report mode is"));
  s.run(1100);
  s.radar.presence = true;
  CHECK(s.run_until([&] { return s.host.occupancy; }, 100));
}

TEST(passive_radar_is_logged_and_uart_never_live) {
  Sim s(Model::MODEL_SEN0395, [](EngineConfig &c, FakeRadar &r) {
    c.has_pin = true;
    r.uart_period = 1600;
  });
  CHECK(s.booted());
  CHECK(s.host.logged("UART report mode is passive"));
  CHECK(!s.host.has(P::PARAM_ID_UART_REPORT_PERIOD));
  CHECK_STR(s.host.last_error, "");
  s.radar.presence = true;
  s.run(10000);
  CHECK(s.host.uart_state == Presence::PRESENCE_UNKNOWN);
  CHECK(s.host.occupancy_state == Presence::PRESENCE_CLEAR);  // the pin only
  CHECK(s.host.status == Status::STATUS_RUNNING);             // silence is expected, no link loss
  // toggling a report alone keeps the radar's mode (no period to write)
  s.engine->set_desired(P::PARAM_ID_UART_TARGET_EN, 1.0f, s.t);
  CHECK(s.idle(60000));
  CHECK(s.radar.count("setUartOutput 2 1") == 1);
  CHECK(s.radar.count("setUartOutput 2 1 1") == 0);
  // setting the period switches it to on-change
  s.engine->set_desired(P::PARAM_ID_UART_REPORT_PERIOD, 1.0f, s.t);
  CHECK(s.idle(60000));
  CHECK(s.radar.count("setUartOutput 1 1 1 1") == 1);
  CHECK_NEAR(s.host.val(P::PARAM_ID_UART_REPORT_PERIOD), 1, 1e-4);
  s.radar.presence = true;  // plain sensorStart on the SEN0395 cleared it
  s.run(100);
  CHECK(s.host.uart_occupancy && s.host.occupancy);  // on change: at once
}

TEST(echo_off_radar_still_works) {
  Sim s(Model::MODEL_SEN0395, [](EngineConfig &, FakeRadar &r) { r.echo = false; });
  CHECK(s.booted());
  s.engine->set_desired(P::PARAM_ID_LATENCY_OFF, 30.0f, s.t);
  CHECK(s.idle(60000));
  CHECK_NEAR(s.host.val(P::PARAM_ID_LATENCY_OFF), 30, 1e-4);
  CHECK(s.host.last_error.empty());
}

TEST(refresh_on_radar_needing_stop_restarts_it) {
  Sim s(Model::MODEL_SEN0610, [](EngineConfig &, FakeRadar &r) { r.get_while_running = false; });
  CHECK(s.booted());
  s.radar.lat_off = 42;
  s.engine->request_refresh();
  s.run(100);
  CHECK(s.idle());
  CHECK_NEAR(s.host.val(P::PARAM_ID_LATENCY_OFF), 42, 1e-4);
  CHECK(s.radar.running);
  CHECK(s.radar.count("sensorStop") == 2);
  CHECK(s.host.logged("brief stop"));
}

TEST(quiet_but_alive_radar_does_not_flap) {
  Sim s(Model::MODEL_SEN0609);
  CHECK(s.booted());
  size_t statuses_before = s.host.statuses.size();
  int swv_before = s.radar.count("getSWV");
  s.radar.emit_sentences = false;  // alive, answers commands, reports nothing
  bool lost = s.run_until([&] { return s.host.status == Status::STATUS_LINK_LOST; }, 4 * 60 * 1000);
  CHECK(!lost);
  for (size_t i = statuses_before; i < s.host.statuses.size(); i++)
    CHECK(s.host.statuses[i] != Status::STATUS_LINK_LOST);
  CHECK(s.host.link_ok);
  int pings = s.radar.count("getSWV") - swv_before;
  CHECK(pings >= 1 && pings <= (4 * 60) / 30);
  CHECK(s.radar.count("sensorStop") == 0);
  CHECK(s.radar.count("getRange") == 1);  // no boot re-read
  CHECK(s.host.logged("quiet but answers"));

  s.radar.responsive = false;
  uint32_t t0 = s.t;
  CHECK(s.run_until([&] { return s.host.status == Status::STATUS_LINK_LOST; },
                    Engine::LINK_TIMEOUT_MS + 2 * Engine::TIMEOUT_MS + 50));
  CHECK(!s.host.link_ok);
  CHECK(s.t - t0 <= Engine::LINK_TIMEOUT_MS + 2 * Engine::TIMEOUT_MS + 50);
}

TEST(quiet_radar_needing_stop_is_not_stopped_by_ping) {
  Sim s(Model::MODEL_SEN0609, [](EngineConfig &, FakeRadar &r) { r.get_while_running = false; });
  CHECK(s.booted());
  s.radar.emit_sentences = false;
  s.run(3 * 60 * 1000);
  CHECK(s.host.status == Status::STATUS_RUNNING);
  CHECK(s.radar.count("sensorStop") == 1);  // the boot read only
  CHECK(s.radar.running);
}

TEST(silent_probe_commands_are_not_retried) {
  // a C4001 that ignores unknown commands entirely (no Error, no "not recognized")
  Sim s(Model::MODEL_SEN0609, [](EngineConfig &, FakeRadar &r) {
    r.led_supported = false;
    r.uart_output_supported = false;
    r.silent_unknown = true;
  });
  CHECK(s.booted());
  CHECK(s.radar.count("getRunApp") == 1);
  CHECK(s.radar.count("getLedMode 1") == 1);
  CHECK(s.radar.count("getUartOutput 1") == 1);
  CHECK(s.radar.count("getEcho") == 0);  // no echo probe
  CHECK(!s.engine->param_available(ParamId::PARAM_ID_LED));
  CHECK(!s.engine->param_available(ParamId::PARAM_ID_UART_PRESENCE_EN));
  CHECK(s.host.has(ParamId::PARAM_ID_RANGE_MAX));
  CHECK(s.t < 3000 + 12000);  // boot delay + one timeout per probe, not three
}

TEST(last_error_clears_after_clean_transaction) {
  Sim s(Model::MODEL_SEN0609, [](EngineConfig &, FakeRadar &r) { r.clamp_trigger_range = true; });
  CHECK(s.booted());
  CHECK(s.host.last_error.empty());
  s.engine->set_desired(P::PARAM_ID_TRIG_RANGE, 6.0f, s.t);  // radar clamps to max - min = 5.4
  CHECK(s.idle());
  CHECK_CONTAINS(s.host.last_error, "trigger_range: requested 6, radar reports 5.4");
  s.engine->set_desired(P::PARAM_ID_INHIBIT, 3.0f, s.t);  // clean
  CHECK(s.idle());
  CHECK_NEAR(s.host.val(P::PARAM_ID_INHIBIT), 3, 1e-4);
  CHECK_STR(s.host.last_error, "");
  CHECK(s.engine->last_error()[0] == '\0');

  // a failed transaction leaves its message standing
  s.radar.on_cmd = [&](const std::string &line) {
    if (line.rfind("setLatency", 0) == 0)
      s.radar.responsive = false;
  };
  s.engine->set_desired(P::PARAM_ID_LATENCY_OFF, 20.0f, s.t);
  CHECK(s.run_until([&] { return s.host.status == Status::STATUS_LINK_LOST; }, 20000));
  CHECK_CONTAINS(s.host.last_error, "operation failed");
  s.run(3000);
  CHECK_CONTAINS(s.host.last_error, "operation failed");  // still there during back-off
}

TEST(last_error_published_empty_after_clean_boot) {
  Sim s(Model::MODEL_SEN0395);
  CHECK(s.booted());
  CHECK(s.host.last_error_published);
  CHECK_STR(s.host.last_error, "");
}

TEST(c4001_save_error_when_nothing_changed_is_not_an_error) {
  Sim s(Model::MODEL_SEN0609);
  CHECK(s.booted());
  s.engine->set_desired(P::PARAM_ID_INHIBIT, 1.0f, s.t);  // already stored
  CHECK(s.idle());
  CHECK(s.radar.count("saveConfig") == 1);
  CHECK(s.radar.saves == 0);  // radar answered Error
  CHECK_STR(s.host.last_error, "");
  CHECK_NEAR(s.host.val(P::PARAM_ID_INHIBIT), 1, 1e-4);
  CHECK(s.host.logged("nothing to save"));
  CHECK(s.radar.running);
}

TEST(c4001_clamped_uart_period_reports_mismatch_not_save_error) {
  Sim s(Model::MODEL_SEN0609, [](EngineConfig &, FakeRadar &r) {
    r.uart_output_supported = true;
    r.uart_mode = 1;
    r.uart_period = 0.2f;
  });
  CHECK(s.booted());
  CHECK(s.engine->param_available(P::PARAM_ID_UART_REPORT_PERIOD));
  CHECK_NEAR(s.host.val(P::PARAM_ID_UART_REPORT_PERIOD), 0.2, 1e-4);
  s.engine->set_desired(P::PARAM_ID_UART_REPORT_PERIOD, 0.2f, s.t);  // same as stored
  CHECK(s.idle());
  CHECK(s.radar.count("setUartOutput 1 1 1 0.2") == 1);
  CHECK(s.radar.saves == 0);
  CHECK_STR(s.host.last_error, "");
}

TEST(c4001_save_error_after_failed_set_is_reported) {
  Sim s(Model::MODEL_SEN0609);
  CHECK(s.booted());
  s.radar.reject_set_prefix = "setLatency";
  s.engine->set_desired(P::PARAM_ID_INHIBIT, 1.0f, s.t);       // accepted, unchanged
  s.engine->set_desired(P::PARAM_ID_LATENCY_OFF, 30.0f, s.t);  // rejected
  CHECK(s.idle());
  CHECK(s.radar.count("saveConfig") == 1);
  CHECK(s.radar.saves == 0);
  CHECK_CONTAINS(s.host.last_error, "saveConfig answered Error");
  CHECK_NEAR(s.host.val(P::PARAM_ID_LATENCY_OFF), 15, 1e-4);
}

TEST(c4001_uart_report_period_floor) {
  CHECK_NEAR(param_limits(Model::MODEL_SEN0609, P::PARAM_ID_UART_REPORT_PERIOD).lo, 0.2, 1e-6);
  CHECK_NEAR(param_limits(Model::MODEL_SEN0610, P::PARAM_ID_UART_REPORT_PERIOD).lo, 0.2, 1e-6);
  CHECK_NEAR(param_limits(Model::MODEL_SEN0395, P::PARAM_ID_UART_REPORT_PERIOD).lo, 0.025, 1e-6);
  Sim s(Model::MODEL_SEN0609, [](EngineConfig &, FakeRadar &r) {
    r.uart_output_supported = true;
    r.uart_period = 1.0f;
  });
  CHECK(s.booted());
  int before = s.host.params[P::PARAM_ID_UART_REPORT_PERIOD].count;
  CHECK(!s.engine->set_desired(P::PARAM_ID_UART_REPORT_PERIOD, 0.1f, s.t));
  CHECK(s.host.params[P::PARAM_ID_UART_REPORT_PERIOD].count == before + 1);  // reverted
  CHECK_NEAR(s.host.val(P::PARAM_ID_UART_REPORT_PERIOD), 1.0, 1e-4);
  CHECK_CONTAINS(s.host.last_error, "outside");
  s.run(3000);
  CHECK(s.radar.count("setUartOutput") == 0);
  CHECK(s.engine->set_desired(P::PARAM_ID_UART_REPORT_PERIOD, 0.2f, s.t));
  CHECK(s.idle());
  CHECK_NEAR(s.host.val(P::PARAM_ID_UART_REPORT_PERIOD), 0.2, 1e-4);
}

TEST(link_loss_drops_uart_source) {
  Sim s(Model::MODEL_SEN0609, [](EngineConfig &c, FakeRadar &r) {
    c.has_pin = true;
    r.presence = true;
  });
  s.pin = true;
  CHECK(s.booted());
  s.run(2000);
  CHECK(s.host.occupancy && s.host.uart_occupancy && s.host.pin_occupancy);
  // radar power cycle: no replies, no reports, OUT pin low
  s.radar.responsive = false;
  s.radar.emit_sentences = false;
  s.pin = false;
  s.run(50);
  CHECK(s.host.occupancy);  // UART still live
  s.run(60000);
  CHECK(s.host.status == Status::STATUS_LINK_LOST);
  CHECK(s.host.uart_state == Presence::PRESENCE_UNKNOWN);
  CHECK(s.host.occupancy_state == Presence::PRESENCE_CLEAR);  // the pin keeps occupancy working
}

TEST(stop_delivered_without_reply_is_undone_after_recovery) {
  Sim s(Model::MODEL_SEN0609);
  CHECK(s.booted());
  s.radar.deaf_tx = true;  // radar still hears us, we hear nothing
  s.engine->set_desired(P::PARAM_ID_INHIBIT, 3.0f, s.t);
  CHECK(s.run_until([&] { return s.host.status == Status::STATUS_LINK_LOST; }, 40000));
  CHECK(s.radar.count("sensorStop") == 3);
  CHECK(s.radar.count("setInhibit") == 0);  // dropped, not retried later
  CHECK(!s.radar.running);
  int starts = s.radar.count("sensorStart");
  s.radar.deaf_tx = false;
  CHECK(s.run_until([&] { return s.host.link_ok && !s.engine->busy(); }, 90000));
  CHECK(s.radar.count("sensorStart 1") == starts + 1);
  CHECK(s.radar.running);
  CHECK(s.host.status == Status::STATUS_RUNNING);
  CHECK_NEAR(s.host.val(P::PARAM_ID_INHIBIT), 1, 1e-4);
  s.radar.presence = true;
  s.run(2500);
  CHECK(s.host.uart_occupancy);  // reports resumed
  s.run(30000);
  CHECK(s.radar.count("setInhibit") == 0);
}

TEST(user_stopped_radar_stays_stopped_after_recovery) {
  Sim s(Model::MODEL_SEN0609);
  CHECK(s.booted());
  s.engine->request_running(false);
  CHECK(s.run_until([&] { return s.host.status == Status::STATUS_STOPPED; }, 5000));
  s.radar.deaf_tx = true;
  s.engine->set_desired(P::PARAM_ID_INHIBIT, 3.0f, s.t);
  CHECK(s.run_until([&] { return s.host.status == Status::STATUS_LINK_LOST; }, 40000));
  int starts = s.radar.count("sensorStart");
  s.radar.deaf_tx = false;
  CHECK(s.run_until([&] { return s.host.link_ok && !s.engine->busy(); }, 90000));
  s.run(10000);
  CHECK(s.radar.count("sensorStart") == starts);
  CHECK(!s.radar.running);
}

TEST(silent_radar_after_read_is_started_once) {
  // a radar left stopped by an earlier session: it answers, but sends no reports
  Sim s(Model::MODEL_SEN0609, [](EngineConfig &, FakeRadar &r) { r.running = false; });
  CHECK(s.booted());
  CHECK(s.run_until([&] { return s.radar.running; }, 10000));
  CHECK(s.radar.count("sensorStart 1") == 1);
  s.run(5000);
  CHECK(s.host.running);
  CHECK(s.radar.count("sensorStart") == 1);
}

TEST(work_mode_switch_runs_before_other_changes) {
  Sim s(Model::MODEL_SEN0609, [](EngineConfig &, FakeRadar &r) { r.sentence_period_ms = 200; });
  CHECK(s.booted());
  s.engine->set_desired(P::PARAM_ID_WORK_MODE, 1.0f, s.t);
  s.engine->set_desired(P::PARAM_ID_RANGE_MAX, 10.0f, s.t);
  CHECK(s.idle());
  CHECK(s.radar.run_app == 1);
  CHECK_NEAR(s.radar.app_range_max[1], 10, 1e-4);  // written to the new app
  CHECK_NEAR(s.radar.app_range_max[0], 6, 1e-4);
  CHECK_NEAR(s.host.val(P::PARAM_ID_RANGE_MAX), 10, 1e-4);
  size_t run_app = 0, set_range = 0;
  for (size_t i = 0; i < s.radar.received.size(); i++) {
    if (s.radar.received[i].rfind("setRunApp", 0) == 0)
      run_app = i;
    if (s.radar.received[i].rfind("setRange", 0) == 0)
      set_range = i;
  }
  CHECK(run_app > 0 && set_range > run_app);
  CHECK_STR(s.host.last_error, "");
  // and back: the presence app still has its own range
  s.engine->set_desired(P::PARAM_ID_WORK_MODE, 0.0f, s.t);
  CHECK(s.idle());
  CHECK(s.radar.run_app == 0);
  CHECK_NEAR(s.host.val(P::PARAM_ID_RANGE_MAX), 6, 1e-4);
  CHECK_STR(s.host.sw, "JYSJ_00.00.04.040220");
}

TEST(save_config_not_stopped_is_an_error) {
  Sim s(Model::MODEL_SEN0609);
  CHECK(s.booted());
  s.radar.on_cmd = [&](const std::string &line) {
    if (line == "saveConfig")
      s.radar.running = true;  // radar restarted behind our back
  };
  s.engine->set_desired(P::PARAM_ID_INHIBIT, 3.0f, s.t);
  CHECK(s.idle());
  CHECK_CONTAINS(s.host.last_error, "sensor is not stopped");
  CHECK(!s.host.logged("nothing to save"));
}

TEST(uart_output_group_is_presence_only_on_c4001) {
  // speed app at power-up and no reports during the boot read: the read assumes presence and the
  // uart output probe fails
  Sim s(Model::MODEL_SEN0609, [](EngineConfig &, FakeRadar &r) {
    r.use_app(1);
    r.emit_sentences = false;
  });
  CHECK(s.booted());
  CHECK(s.radar.count("getUartOutput 1") == 1);
  CHECK(!s.engine->param_available(P::PARAM_ID_UART_REPORT_PERIOD));
  s.radar.emit_sentences = true;
  s.radar.sentence_period_ms = 100;
  s.run(2000);
  CHECK(s.idle());
  CHECK(s.engine->work_mode() == WorkMode::WORK_MODE_SPEED);
  CHECK(s.host.has(P::PARAM_ID_MICRO_MOTION));
  CHECK(s.radar.count("getUartOutput 1") == 1);  // not asked again in speed mode
  s.run(30000);
  CHECK(s.radar.count("getUartOutput 1") == 1);
  CHECK(!s.host.logged("getUartOutput answered"));

  // setting one in speed mode is refused at the entity
  int before = s.host.params[P::PARAM_ID_UART_REPORT_PERIOD].count;
  CHECK(!s.engine->set_desired(P::PARAM_ID_UART_REPORT_PERIOD, 2.0f, s.t));
  CHECK_STR(s.host.last_error, "uart output settings are presence-mode only");
  CHECK(s.host.params[P::PARAM_ID_UART_REPORT_PERIOD].count == before + 1);
  s.run(3000);
  CHECK(s.radar.count("setUartOutput") == 0);

  // back to presence: the identify-and-read probes the group again
  s.engine->set_desired(P::PARAM_ID_WORK_MODE, 0.0f, s.t);
  CHECK(s.idle());
  CHECK(s.radar.run_app == 0);
  CHECK(s.radar.count("getUartOutput 1") == 2);
  CHECK(s.engine->param_available(P::PARAM_ID_UART_REPORT_PERIOD));
  CHECK_NEAR(s.host.val(P::PARAM_ID_UART_REPORT_PERIOD), 1, 1e-4);
  CHECK(s.host.val(P::PARAM_ID_UART_PRESENCE_EN) == 1);
  CHECK(s.engine->set_desired(P::PARAM_ID_UART_REPORT_PERIOD, 2.0f, s.t));
  CHECK(s.idle());
  CHECK_NEAR(s.radar.uart_period, 2, 1e-4);
}

TEST(occupancy_speed_mode) {
  Sim s(Model::MODEL_SEN0609, [](EngineConfig &c, FakeRadar &r) {
    c.has_pin = true;
    r.use_app(1);
    r.sentence_period_ms = 100;
  });
  CHECK(s.booted());
  CHECK(s.engine->work_mode() == WorkMode::WORK_MODE_SPEED);
  CHECK(s.host.occupancy_state == Presence::PRESENCE_CLEAR);
  s.radar.presence = true;  // $DFDMD with one target, pin low
  s.run(300);
  CHECK(s.host.occupancy && s.host.uart_occupancy);
  // reports stop: no target after target_timeout, then the source goes stale
  s.radar.emit_sentences = false;
  s.run(s.cfg.target_timeout_ms + 200);
  CHECK(s.host.uart_state == Presence::PRESENCE_CLEAR);
  CHECK(s.host.occupancy_state == Presence::PRESENCE_CLEAR);
  s.pin = true;  // the pin counts in speed mode too
  s.run(10);
  CHECK(s.host.occupancy);
  s.run(s.engine->uart_presence_timeout_ms());
  CHECK(s.host.uart_state == Presence::PRESENCE_UNKNOWN);
  CHECK(s.host.occupancy);
  s.pin = false;
  s.run(10);
  CHECK(s.host.occupancy_state == Presence::PRESENCE_CLEAR);
}

TEST(range_limits_always_leave_a_window) {
  // bench (SEN0395): the max_range slider offered 0 with min_range at its factory 0, which can only be
  // refused ("min_range 0.00 must be below max_range 0.00"). The limits keep one step between them.
  for (Model m : {Model::MODEL_SEN0395, Model::MODEL_SEN0609, Model::MODEL_SEN0610}) {
    Limits mn = param_limits(m, P::PARAM_ID_RANGE_MIN);
    Limits mx = param_limits(m, P::PARAM_ID_RANGE_MAX);
    CHECK(mx.lo >= mn.lo + mn.step - 1e-6f);
    CHECK(mn.hi <= mx.hi - mx.step + 1e-6f);
  }
  Sim s(Model::MODEL_SEN0395);
  CHECK(s.booted());
  size_t sent = s.radar.received.size();
  CHECK(!s.engine->set_desired(P::PARAM_ID_RANGE_MAX, 0.0f, s.t));  // refused by the limit, nothing sent
  CHECK(!s.engine->set_desired(P::PARAM_ID_RANGE_MIN, 9.45f, s.t));
  CHECK(s.radar.received.size() == sent);
  CHECK_CONTAINS(s.host.last_error, "outside");
  CHECK_NEAR(s.host.val(P::PARAM_ID_RANGE_MAX), 9.45, 1e-4);
  CHECK_NEAR(s.host.val(P::PARAM_ID_RANGE_MIN), 0, 1e-4);
  s.engine->set_desired(P::PARAM_ID_RANGE_MAX, 0.15f, s.t);  // the smallest window the sliders allow
  CHECK(s.idle(60000));
  CHECK(s.radar.count("setRange 0 0.15") == 1);
  CHECK_NEAR(s.host.val(P::PARAM_ID_RANGE_MAX), 0.15, 1e-4);
}

TEST(c4001_max_range_floor) {
  CHECK_NEAR(param_limits(Model::MODEL_SEN0609, P::PARAM_ID_RANGE_MAX).lo, 2.4, 1e-6);
  CHECK_NEAR(param_limits(Model::MODEL_SEN0610, P::PARAM_ID_RANGE_MAX).lo, 2.4, 1e-6);
  CHECK_NEAR(param_limits(Model::MODEL_SEN0609, P::PARAM_ID_TRIG_RANGE).lo, 2.4, 1e-6);
  CHECK_NEAR(param_limits(Model::MODEL_SEN0609, P::PARAM_ID_INHIBIT).hi, 60, 1e-6);
  CHECK_NEAR(param_limits(Model::MODEL_SEN0609, P::PARAM_ID_LATENCY_ON).hi, 2, 1e-6);
  CHECK_NEAR(param_limits(Model::MODEL_SEN0609, P::PARAM_ID_LATENCY_OFF).lo, 2, 1e-6);
  CHECK_NEAR(param_limits(Model::MODEL_SEN0610, P::PARAM_ID_TRIG_RANGE).hi, 12, 1e-6);
  CHECK_NEAR(param_limits(Model::MODEL_SEN0395, P::PARAM_ID_RANGE_MAX).lo, 0.15, 1e-6);
  Sim s(Model::MODEL_SEN0609);
  CHECK(s.booted());
  int before = s.host.params[P::PARAM_ID_RANGE_MAX].count;
  CHECK(!s.engine->set_desired(P::PARAM_ID_RANGE_MAX, 1.2f, s.t));
  CHECK(s.host.params[P::PARAM_ID_RANGE_MAX].count == before + 1);
  CHECK_NEAR(s.host.val(P::PARAM_ID_RANGE_MAX), 6, 1e-4);
  CHECK_CONTAINS(s.host.last_error, "outside");
  s.run(3000);
  CHECK(s.radar.count("setRange") == 0);
  CHECK(s.radar.count("sensorStop") == 0);
}

TEST(c4001_min_range_and_inhibit_floors) {
  // bench: "inhibit_time: requested 0.1, radar reports 0.3", "min_range: requested 0, radar reports 0.3"
  for (Model m : {Model::MODEL_SEN0609, Model::MODEL_SEN0610}) {
    CHECK_NEAR(param_limits(m, P::PARAM_ID_INHIBIT).lo, 0.3, 1e-6);
    CHECK_NEAR(param_limits(m, P::PARAM_ID_RANGE_MIN).lo, 0.3, 1e-6);
  }
  CHECK_NEAR(param_limits(Model::MODEL_SEN0609, P::PARAM_ID_RANGE_MIN).hi, 25.9, 1e-6);
  CHECK_NEAR(param_limits(Model::MODEL_SEN0610, P::PARAM_ID_RANGE_MIN).hi, 11.9, 1e-6);
  CHECK_NEAR(param_limits(Model::MODEL_SEN0395, P::PARAM_ID_RANGE_MIN).lo, 0, 1e-6);
  Sim s(Model::MODEL_SEN0609);
  CHECK(s.booted());
  int before = s.host.params[P::PARAM_ID_INHIBIT].count;
  CHECK(!s.engine->set_desired(P::PARAM_ID_INHIBIT, 0.1f, s.t));
  CHECK(s.host.params[P::PARAM_ID_INHIBIT].count == before + 1);  // reverted
  CHECK_NEAR(s.host.val(P::PARAM_ID_INHIBIT), 1, 1e-4);
  CHECK_CONTAINS(s.host.last_error, "inhibit_time: 0.100 is outside 0.300..60.000");
  CHECK(!s.engine->set_desired(P::PARAM_ID_RANGE_MIN, 0.0f, s.t));
  CHECK_CONTAINS(s.host.last_error, "min_range: 0.000 is outside 0.300..25.900");
  s.run(3000);
  CHECK(s.radar.count("setInhibit") == 0);
  CHECK(s.radar.count("setRange") == 0);
  CHECK(s.radar.count("sensorStop") == 0);
  CHECK(s.engine->set_desired(P::PARAM_ID_INHIBIT, 0.3f, s.t));
  CHECK(s.idle());
  CHECK_NEAR(s.host.val(P::PARAM_ID_INHIBIT), 0.3, 1e-4);
  CHECK_STR(s.host.last_error, "");
}

TEST(trigger_range_warning_removed) {
  // factory trigger 6 m with a 0.6-6 m window used to warn on every change; the firmware has no such rule
  Sim s(Model::MODEL_SEN0609);
  CHECK(s.booted());
  s.engine->set_desired(P::PARAM_ID_TRIG_RANGE, 2.4f, s.t);
  CHECK(s.idle());
  s.engine->set_desired(P::PARAM_ID_RANGE_MIN, 3.5f, s.t);
  CHECK(s.idle());
  s.engine->set_desired(P::PARAM_ID_TRIG_RANGE, 5.5f, s.t);  // > max - min (2.5), <= max
  CHECK(s.idle());
  CHECK(!s.host.logged("exceeds"));
  CHECK(!s.host.logged("may limit"));
  CHECK_NEAR(s.host.val(P::PARAM_ID_TRIG_RANGE), 5.5, 1e-4);
  CHECK_STR(s.host.last_error, "");
}

TEST(max_range_below_trigger_pulls_trigger_down) {
  Sim s(Model::MODEL_SEN0609);
  CHECK(s.booted());
  CHECK_NEAR(s.host.val(P::PARAM_ID_TRIG_RANGE), 6, 1e-4);
  int trig_before = s.host.params[P::PARAM_ID_TRIG_RANGE].count;
  s.engine->set_desired(P::PARAM_ID_RANGE_MAX, 4.5f, s.t);
  CHECK(s.idle());
  CHECK(s.engine->transaction_count() == 1);
  CHECK(s.radar.saves == 1);
  CHECK(s.radar.count("setRange 0.6 4.5") == 1);
  CHECK(s.radar.count("setTrigRange 4.5") == 1);
  CHECK(s.radar.count("getTrigRange") == 2);  // boot read and this read-back
  CHECK_NEAR(s.radar.trig, 4.5, 1e-4);
  CHECK_NEAR(s.host.val(P::PARAM_ID_RANGE_MAX), 4.5, 1e-4);
  CHECK_NEAR(s.host.val(P::PARAM_ID_TRIG_RANGE), 4.5, 1e-4);
  auto &hist = s.host.params[P::PARAM_ID_TRIG_RANGE].history;
  CHECK(s.host.params[P::PARAM_ID_TRIG_RANGE].count == trig_before + 2);  // optimistic, then read-back
  CHECK(hist.size() >= 2 && std::fabs(hist[hist.size() - 2] - 4.5f) < 1e-4f);
  CHECK(s.host.logged("max_range 4.5 pulls trigger_range down to 4.5"));
  CHECK_STR(s.host.last_error, "");
  CHECK(s.radar.running);
}

TEST(trigger_above_max_pushes_max_up) {
  Sim s(Model::MODEL_SEN0609);
  CHECK(s.booted());
  s.engine->set_desired(P::PARAM_ID_TRIG_RANGE, 9.0f, s.t);
  CHECK(s.idle());
  CHECK(s.engine->transaction_count() == 1);
  CHECK(s.radar.saves == 1);
  CHECK(s.radar.count("setRange 0.6 9") == 1);
  CHECK(s.radar.count("setTrigRange 9") == 1);
  size_t range_idx = 0, trig_idx = 0;
  for (size_t i = 0; i < s.radar.received.size(); i++) {
    if (s.radar.received[i] == "setRange 0.6 9")
      range_idx = i;
    if (s.radar.received[i] == "setTrigRange 9")
      trig_idx = i;
  }
  CHECK(range_idx > 0 && trig_idx > range_idx);  // the window is widened first
  CHECK_NEAR(s.host.val(P::PARAM_ID_RANGE_MAX), 9, 1e-4);
  CHECK_NEAR(s.host.val(P::PARAM_ID_TRIG_RANGE), 9, 1e-4);
  CHECK_NEAR(s.host.val(P::PARAM_ID_RANGE_MIN), 0.6, 1e-4);
  CHECK(s.host.logged("trigger_range 9 pushes max_range up to 9"));
  CHECK_STR(s.host.last_error, "");
}

TEST(coupling_respects_limits) {
  {
    Sim s(Model::MODEL_SEN0610);
    CHECK(s.booted());
    CHECK(!s.engine->set_desired(P::PARAM_ID_TRIG_RANGE, 30.0f, s.t));
    CHECK_CONTAINS(s.host.last_error, "outside");
    s.run(3000);
    CHECK(s.radar.count("setTrigRange") == 0);
    CHECK(s.radar.count("setRange") == 0);
    CHECK(s.radar.count("sensorStop") == 0);
    CHECK_NEAR(s.host.val(P::PARAM_ID_RANGE_MAX), 6, 1e-4);
    // the SEN0610 ceiling is 12 for both, so the pushed max never needs clamping
    s.engine->set_desired(P::PARAM_ID_TRIG_RANGE, 12.0f, s.t);
    CHECK(s.idle());
    CHECK(s.radar.count("setRange 0.6 12") == 1);
    CHECK_NEAR(s.host.val(P::PARAM_ID_RANGE_MAX), 12, 1e-4);
    CHECK_NEAR(s.host.val(P::PARAM_ID_TRIG_RANGE), 12, 1e-4);
    CHECK_STR(s.host.last_error, "");
  }
  {
    Sim s(Model::MODEL_SEN0609);
    CHECK(s.booted());
    s.engine->set_desired(P::PARAM_ID_RANGE_MAX, 2.4f, s.t);
    CHECK(s.idle());
    CHECK(s.radar.count("setRange 0.6 2.4") == 1);
    CHECK(s.radar.count("setTrigRange 2.4") == 1);  // the trigger floor
    CHECK_NEAR(s.host.val(P::PARAM_ID_RANGE_MAX), 2.4, 1e-4);
    CHECK_NEAR(s.host.val(P::PARAM_ID_TRIG_RANGE), 2.4, 1e-4);
    CHECK_STR(s.host.last_error, "");
  }
}

TEST(coupling_only_when_needed) {
  Sim s(Model::MODEL_SEN0609);
  CHECK(s.booted());
  s.engine->set_desired(P::PARAM_ID_RANGE_MAX, 8.0f, s.t);
  CHECK(s.idle());
  CHECK(s.radar.count("setRange 0.6 8") == 1);
  CHECK(s.radar.count("setTrigRange") == 0);
  s.engine->set_desired(P::PARAM_ID_TRIG_RANGE, 8.0f, s.t);  // equal is consistent
  CHECK(s.idle());
  CHECK(s.radar.count("setRange") == 1);
  CHECK(s.radar.count("setTrigRange 8") == 1);
  s.engine->set_desired(P::PARAM_ID_RANGE_MIN, 1.0f, s.t);  // min_range plays no part
  CHECK(s.idle());
  CHECK(s.radar.count("setTrigRange") == 1);
  CHECK(!s.host.logged("pulls") && !s.host.logged("pushes"));
  CHECK_STR(s.host.last_error, "");
}

TEST(coupling_never_overrides_a_request) {
  // both changed in one batch: each is sent as asked and the read-back shows what the radar kept
  Sim s(Model::MODEL_SEN0609);
  CHECK(s.booted());
  s.engine->set_desired(P::PARAM_ID_RANGE_MAX, 4.0f, s.t);
  s.engine->set_desired(P::PARAM_ID_TRIG_RANGE, 5.0f, s.t);
  CHECK(s.idle());
  CHECK(s.engine->transaction_count() == 1);
  CHECK(s.radar.count("setRange 0.6 4") == 1);
  CHECK(s.radar.count("setTrigRange 5") == 1);
  CHECK(!s.host.logged("pulls") && !s.host.logged("pushes"));
}

TEST(refused_range_does_not_move_trigger) {
  Sim s(Model::MODEL_SEN0609);
  CHECK(s.booted());
  s.engine->set_desired(P::PARAM_ID_RANGE_MIN, 3.0f, s.t);
  CHECK(s.idle());
  int trig_before = s.host.params[P::PARAM_ID_TRIG_RANGE].count;
  s.engine->set_desired(P::PARAM_ID_RANGE_MAX, 2.5f, s.t);  // below min_range: a mistake, refused
  CHECK(s.idle());
  CHECK(s.radar.count("setRange") == 1);
  CHECK(s.radar.count("setTrigRange") == 0);
  CHECK(s.host.params[P::PARAM_ID_TRIG_RANGE].count == trig_before);
  CHECK_NEAR(s.host.val(P::PARAM_ID_TRIG_RANGE), 6, 1e-4);
  CHECK_NEAR(s.host.val(P::PARAM_ID_RANGE_MAX), 6, 1e-4);
  CHECK_CONTAINS(s.host.last_error, "must be below");
  CHECK_CONTAINS(s.host.last_error, "reverted max_range");
}

TEST(no_coupling_in_speed_mode) {
  Sim s(Model::MODEL_SEN0609, [](EngineConfig &, FakeRadar &r) { r.sentence_period_ms = 200; });
  CHECK(s.booted());
  s.engine->set_desired(P::PARAM_ID_WORK_MODE, 1.0f, s.t);
  CHECK(s.idle());
  CHECK(s.engine->work_mode() == WorkMode::WORK_MODE_SPEED);
  s.engine->set_desired(P::PARAM_ID_RANGE_MAX, 4.0f, s.t);  // speed app range 0-26, presence trigger 6
  CHECK(s.idle());
  CHECK(s.radar.count("setRange 0 4") == 1);
  CHECK(s.radar.count("setTrigRange") == 0);
  CHECK(!s.host.logged("pulls"));
  CHECK_STR(s.host.last_error, "");
}

TEST(c4001_factory_reset_nothing_changed) {
  // bench: resetCfg on a radar at factory values, then "no parameter has changed" + Error to saveConfig
  Sim s(Model::MODEL_SEN0609, [](EngineConfig &, FakeRadar &r) { r.c4001_nothing_to_save_text = true; });
  CHECK(s.booted());
  s.engine->request_factory_reset();
  s.run(100);
  CHECK(s.idle());
  CHECK(s.radar.count("resetCfg") == 1);
  CHECK(s.radar.count("saveConfig") == 1);
  CHECK(s.radar.saves == 0);
  CHECK_STR(s.host.last_error, "");
  CHECK(s.radar.running);
  CHECK_NEAR(s.host.val(P::PARAM_ID_RANGE_MIN), 0.6, 1e-4);
  CHECK_NEAR(s.host.val(P::PARAM_ID_LATENCY_ON), 0.05, 1e-4);
}

TEST(sen0395_factory_reset) {
  // bench: resetCfg -> "factory reset complete" + Done, saveConfig -> "no parameter has changed" + Done
  for (bool text : {true, false}) {
    Sim s(Model::MODEL_SEN0395, [text](EngineConfig &, FakeRadar &r) { r.sen0395_reset_text = text; });
    CHECK(s.booted());
    s.engine->set_desired(P::PARAM_ID_RANGE_MAX, 6.45f, s.t);
    s.engine->set_desired(P::PARAM_ID_LATENCY_OFF, 15.0f, s.t);
    CHECK(s.idle());
    CHECK(s.radar.saves == 1);
    s.engine->request_factory_reset();
    s.run(100);
    CHECK(s.idle());
    CHECK(s.radar.count("resetCfg") == 1);
    CHECK(s.radar.count("saveConfig") == 2);
    CHECK(s.radar.saves == 1);
    CHECK(s.host.logged("radar restored its factory settings") == text);
    CHECK(s.host.logged("max_range = 9.45 (was 6.45)"));
    CHECK_NEAR(s.host.val(P::PARAM_ID_LATENCY_OFF), 5, 1e-4);
    CHECK_STR(s.host.last_error, "");
    CHECK(s.radar.running);
  }
}

TEST(occupancy_uart_goes_stale) {
  for (bool pin : {true, false}) {
    Model m = pin ? Model::MODEL_SEN0609 : Model::MODEL_SEN0610;
    Sim s(m, [pin](EngineConfig &c, FakeRadar &r) {
      c.has_pin = pin;
      r.presence = true;
    });
    CHECK(s.booted());
    s.run(1500);
    CHECK(s.host.occupancy && s.host.uart_occupancy);
    CHECK(s.engine->uart_presence_timeout_ms() == 4000);  // 3 x 1 s report period + 1 s
    s.radar.emit_sentences = false;
    s.run(3500);
    CHECK(s.host.occupancy);
    s.run(1500);
    CHECK(s.host.uart_state == Presence::PRESENCE_UNKNOWN);
    CHECK(s.host.occupancy_state == (pin ? Presence::PRESENCE_CLEAR : Presence::PRESENCE_UNKNOWN));
    s.radar.emit_sentences = true;
    s.run(1200);
    CHECK(s.host.occupancy && s.host.uart_occupancy);
  }
}

TEST(occupancy_uart_timeout_follows_report_period) {
  Sim s(Model::MODEL_SEN0609, [](EngineConfig &, FakeRadar &r) { r.uart_period = 0.5f; });
  CHECK(s.booted());
  CHECK(s.engine->uart_presence_timeout_ms() == 2500);
  s.engine->set_desired(P::PARAM_ID_UART_REPORT_PERIOD, 2.0f, s.t);
  CHECK(s.idle());
  CHECK(s.engine->uart_presence_timeout_ms() == 7000);
}

TEST(occupancy_pin_only_without_uart_report) {
  Sim s(Model::MODEL_SEN0609, [](EngineConfig &c, FakeRadar &r) {
    c.has_pin = true;
    r.uart1_en = 0;
  });
  CHECK(s.booted());
  CHECK(s.host.val(P::PARAM_ID_UART_PRESENCE_EN) == 0);
  CHECK(s.host.occupancy_state == Presence::PRESENCE_CLEAR);
  s.pin = true;
  s.run(10);
  CHECK(s.host.occupancy);
  s.run(60000);
  CHECK(s.host.occupancy);
  CHECK(s.host.uart_state == Presence::PRESENCE_UNKNOWN);
  CHECK(!s.host.logged("UART presence unavailable"));
  s.pin = false;
  s.run(10);
  CHECK(s.host.occupancy_state == Presence::PRESENCE_CLEAR);
}

TEST(occupancy_uart_only_when_no_pin) {
  Sim s(Model::MODEL_SEN0610);
  s.run(500);
  CHECK(s.host.occupancy_count == 0);  // nothing live yet: unknown, not published
  CHECK(s.booted());
  CHECK(s.host.occupancy_state == Presence::PRESENCE_CLEAR);
  CHECK(!s.host.pin_published);
  s.radar.presence = true;
  s.run(1100);
  CHECK(s.host.occupancy);
  s.radar.presence = false;
  s.run(1100);
  CHECK(s.host.occupancy_state == Presence::PRESENCE_CLEAR);
}

TEST(c4001_other_mode_entities_unavailable) {
  Sim s(Model::MODEL_SEN0609, [](EngineConfig &, FakeRadar &r) { r.sentence_period_ms = 200; });
  CHECK(s.booted());
  // presence mode: no target data, speed settings unavailable
  CHECK(!s.host.frames.empty() && !s.host.frames.back().available);
  CHECK(!s.host.has(P::PARAM_ID_THR_FACTOR));
  CHECK(s.host.has(P::PARAM_ID_UART_REPORT_PERIOD));
  s.engine->set_desired(P::PARAM_ID_WORK_MODE, 1.0f, s.t);
  CHECK(s.idle());
  CHECK(s.host.has(P::PARAM_ID_THR_FACTOR));
  CHECK(s.host.has(P::PARAM_ID_MICRO_MOTION));
  CHECK(!s.host.has(P::PARAM_ID_UART_REPORT_PERIOD));
  CHECK(!s.host.has(P::PARAM_ID_UART_PRESENCE_EN));  // switch
  CHECK(s.host.has(P::PARAM_ID_LED) && s.host.has(P::PARAM_ID_WORK_MODE));
  CHECK(!s.host.has(P::PARAM_ID_INHIBIT));
  CHECK(s.host.has(P::PARAM_ID_RANGE_MAX));
  CHECK(s.host.frames.back().available);
  s.engine->set_desired(P::PARAM_ID_WORK_MODE, 0.0f, s.t);
  CHECK(s.idle());
  CHECK(!s.host.has(P::PARAM_ID_THR_FACTOR));
  CHECK(!s.host.has(P::PARAM_ID_MICRO_MOTION));
  CHECK(s.host.has(P::PARAM_ID_INHIBIT));
  CHECK(s.host.has(P::PARAM_ID_UART_PRESENCE_EN));
  CHECK(!s.host.frames.back().available);
}

TEST(led_switch_on_is_blink) {
  Sim s(Model::MODEL_SEN0395);
  CHECK(s.booted());
  CHECK(s.host.val(P::PARAM_ID_LED) == 1);  // wire 0 = blink
  s.engine->set_desired(P::PARAM_ID_LED, 0.0f, s.t);
  CHECK(s.idle(60000));
  CHECK(s.radar.count("setLedMode 1 1") == 1);  // wire 1 = off
  CHECK(s.radar.led == 1);
  CHECK(s.host.val(P::PARAM_ID_LED) == 0);
  CHECK(s.host.last_error.empty());
  s.engine->set_desired(P::PARAM_ID_LED, 1.0f, s.t);
  CHECK(s.idle(60000));
  CHECK(s.radar.count("setLedMode 1 0") == 1);
  CHECK(s.host.val(P::PARAM_ID_LED) == 1);
}

TEST(own_transaction_does_not_drop_uart_presence) {
  // slow radar that has to stop for every get: the settings change pauses reports for several seconds
  Sim s(Model::MODEL_SEN0610, [](EngineConfig &, FakeRadar &r) {
    r.get_while_running = false;
    r.reply_delay_ms = 600;
    r.presence = true;
  });
  CHECK(s.booted());
  s.run(1500);
  CHECK(s.host.occupancy);
  int published = s.host.occupancy_count;
  uint32_t t0 = s.t;
  s.engine->set_desired(P::PARAM_ID_LATENCY_OFF, 30.0f, s.t);
  s.engine->set_desired(P::PARAM_ID_INHIBIT, 3.0f, s.t);
  s.engine->set_desired(P::PARAM_ID_HOLD_SENS, 4.0f, s.t);
  s.engine->set_desired(P::PARAM_ID_TRIG_RANGE, 5.0f, s.t);
  CHECK(s.idle());
  CHECK(s.t - t0 > s.engine->uart_presence_timeout_ms());
  s.run(1500);
  CHECK(s.host.occupancy_count == published);  // never went unknown
  CHECK(s.host.occupancy);
}

TEST(mode_switch_back_to_presence_with_uart_report_off) {
  Sim s(Model::MODEL_SEN0609, [](EngineConfig &c, FakeRadar &r) {
    c.has_pin = true;
    r.uart1_en = 0;  // pin-only user: no $DFHPD in the presence app
    r.sentence_period_ms = 100;
  });
  CHECK(s.booted());
  s.engine->set_desired(P::PARAM_ID_WORK_MODE, 1.0f, s.t);
  CHECK(s.idle());
  CHECK(s.engine->work_mode() == WorkMode::WORK_MODE_SPEED);
  s.engine->set_desired(P::PARAM_ID_WORK_MODE, 0.0f, s.t);
  CHECK(s.idle());
  CHECK(s.radar.run_app == 0);
  CHECK(s.engine->work_mode() == WorkMode::WORK_MODE_PRESENCE);
  CHECK_STR(s.host.last_error, "");
  CHECK(s.host.has(P::PARAM_ID_HOLD_SENS));
  CHECK(s.host.val(P::PARAM_ID_WORK_MODE) == 0);
  CHECK(s.host.logged("presence app"));
  CHECK(s.radar.count("getMicroMotion") == 1);  // only the read in speed mode
}

TEST(mode_learned_mid_read_is_followed_by_a_read_of_that_mode) {
  Sim s(Model::MODEL_SEN0609, [](EngineConfig &, FakeRadar &r) {
    r.use_app(1);
    r.emit_sentences = false;
    r.sentence_period_ms = 100;
  });
  s.radar.on_cmd = [&](const std::string &line) {
    if (line == "getTrigRange")
      s.radar.emit_sentences = true;  // first $DFDMD lands inside the boot read
  };
  CHECK(s.booted());
  s.run(5000);
  CHECK(s.idle());
  CHECK(s.engine->work_mode() == WorkMode::WORK_MODE_SPEED);
  CHECK(s.host.has(P::PARAM_ID_MICRO_MOTION));
  CHECK(s.radar.count("getMicroMotion") == 1);
  s.run(30000);
  CHECK(s.radar.count("getMicroMotion") == 1);  // no repeated reads
}

TEST(mode_learned_before_the_gets_needs_no_extra_read) {
  Sim s(Model::MODEL_SEN0609, [](EngineConfig &, FakeRadar &r) {
    r.use_app(1);
    r.emit_sentences = false;
    r.sentence_period_ms = 100;
  });
  s.radar.on_cmd = [&](const std::string &line) {
    if (line == "getHWV")
      s.radar.emit_sentences = true;  // mode known before the boot read picks its groups
  };
  CHECK(s.booted());
  s.run(5000);
  CHECK(s.idle());
  CHECK(s.host.has(P::PARAM_ID_MICRO_MOTION));
  CHECK(s.radar.count("getMicroMotion") == 1);
  CHECK(s.radar.count("getRange") == 1);
}

TEST(work_mode_published_for_a_silent_presence_radar) {
  Sim s(Model::MODEL_SEN0609, [](EngineConfig &c, FakeRadar &r) {
    c.has_pin = true;
    r.uart1_en = 0;  // pin-only: the presence app prints nothing
  });
  CHECK(s.booted());
  s.run(5000);
  CHECK(s.host.has(P::PARAM_ID_WORK_MODE));
  CHECK(s.host.val(P::PARAM_ID_WORK_MODE) == 0);
  CHECK(s.engine->work_mode() == WorkMode::WORK_MODE_PRESENCE);
}

TEST(work_mode_select_corrected_after_lost_switch_confirmation) {
  Sim s(Model::MODEL_SEN0609, [](EngineConfig &c, FakeRadar &r) {
    c.has_pin = true;
    r.uart1_en = 0;
    r.sentence_period_ms = 100;
  });
  CHECK(s.booted());
  s.engine->set_desired(P::PARAM_ID_WORK_MODE, 1.0f, s.t);
  CHECK(s.idle());
  s.radar.on_cmd = [&](const std::string &line) {
    if (line.rfind("setRunApp", 0) == 0)
      s.radar.responsive = false;  // link dies right after the switch is delivered
  };
  s.engine->set_desired(P::PARAM_ID_WORK_MODE, 0.0f, s.t);
  s.run(12000);
  s.radar.responsive = true;
  CHECK(s.run_until([&] { return s.host.link_ok && !s.engine->busy(); }, 120000));
  CHECK(s.radar.run_app == 0);
  CHECK(s.engine->work_mode() == WorkMode::WORK_MODE_PRESENCE);
  CHECK(s.host.val(P::PARAM_ID_WORK_MODE) == 0);
  CHECK(s.host.has(P::PARAM_ID_HOLD_SENS));
}

TEST(speed_radar_is_not_mistaken_for_presence) {
  // the speed app streams $DFDMD, so the silence rule must never fire on it
  Sim s(Model::MODEL_SEN0609, [](EngineConfig &c, FakeRadar &r) {
    c.has_pin = true;
    r.uart1_en = 0;
    r.use_app(1);
    r.sentence_period_ms = 100;
  });
  CHECK(s.booted());
  s.run(5000);
  CHECK(s.engine->work_mode() == WorkMode::WORK_MODE_SPEED);
  CHECK(s.host.val(P::PARAM_ID_WORK_MODE) == 1);
  s.engine->request_restart();
  s.run(100);
  CHECK(s.booted());
  s.run(5000);
  CHECK(s.engine->work_mode() == WorkMode::WORK_MODE_SPEED);
  CHECK(s.host.val(P::PARAM_ID_WORK_MODE) == 1);
}

TEST(mode_switch_waits_for_the_new_app) {
  for (bool report : {true, false}) {
    Sim s(Model::MODEL_SEN0609, [report](EngineConfig &c, FakeRadar &r) {
      c.has_pin = true;
      r.uart1_en = report ? 1 : 0;
      r.sentence_period_ms = 100;
    });
    CHECK(s.booted());
    for (float target : {1.0f, 0.0f}) {
      size_t first = s.radar.heard.size();
      s.engine->set_desired(P::PARAM_ID_WORK_MODE, target, s.t);
      CHECK(s.idle());
      uint32_t run_app_ms = 0;
      int swv = 0;
      uint32_t swv_ms = 0;
      for (size_t i = first; i < s.radar.heard.size(); i++) {
        const auto &h = s.radar.heard[i];
        if (h.second.rfind("setRunApp", 0) == 0)
          run_app_ms = h.first;
        if (h.second == "getSWV" && swv++ == 0)
          swv_ms = h.first;
      }
      CHECK(run_app_ms > 0);
      CHECK(swv == 1);  // sent once, after the new app is up: no retry
      if (target > 0.5f || report) {
        CHECK(swv_ms >= run_app_ms + s.radar.app_boot_ms);  // after the first report of the new app
        CHECK(swv_ms < run_app_ms + Engine::MODE_SWITCH_WAIT_MS);
      } else {
        CHECK(swv_ms >= run_app_ms + Engine::MODE_SWITCH_WAIT_MS);  // silent presence app: full wait
      }
      CHECK(s.engine->work_mode() == (target > 0.5f ? WorkMode::WORK_MODE_SPEED : WorkMode::WORK_MODE_PRESENCE));
      CHECK_STR(s.host.last_error, "");
    }
  }
}

TEST(uart_source_drop_is_logged_with_what_is_left) {
  for (bool pin : {true, false}) {
    Model m = pin ? Model::MODEL_SEN0609 : Model::MODEL_SEN0610;
    Sim s(m, [pin](EngineConfig &c, FakeRadar &) { c.has_pin = pin; });
    CHECK(s.booted());
    s.radar.emit_sentences = false;
    s.run(6000);
    CHECK(s.host.logged(pin ? "UART presence source dropped (pin only)"
                            : "UART presence source dropped (no presence source left)"));
  }
}

int main(int argc, char **argv) {
  auto &reg = mini_test::Registry::get();
  int failed_tests = 0;
  size_t runs = 0;
  // the whole suite runs twice: radar prompt printed with and without a trailing space
  for (bool space : {true, false}) {
    g_prompt_space = space;
    std::printf("=== radar prompt %s trailing space ===\n", space ? "with" : "without");
    for (auto &t : reg.tests) {
      if (argc > 1 && std::strstr(t.first, argv[1]) == nullptr)
        continue;
      int before = reg.failures;
      std::printf("[ RUN  ] %s\n", t.first);
      t.second();
      runs++;
      if (reg.failures != before) {
        failed_tests++;
        std::printf("[ FAIL ] %s (prompt %s space)\n", t.first, space ? "with" : "without");
      }
    }
  }
  std::printf("\n%zu tests x 2 prompt variants = %zu runs, %d checks, %d failed runs, %d failed checks\n",
              reg.tests.size(), runs, reg.checks, failed_tests, reg.failures);
  return reg.failures == 0 ? 0 : 1;
}
