# `dfrobot_mmwave` — hardware test plan (XIAO ESP32-C6 + SEN0395 / SEN0609 / SEN0610)

Everything you need is in `examples/bench/` in the esphome checkout:

| File | Purpose |
|---|---|
| `bench-common.yaml` | board, Wi-Fi, API, OTA, web UI, verbose radar logging, heap/loop sensors, UART on D6/D7 |
| `radar-test-sen0395.yaml` | SEN0395 at 115200, IO2 on D3, all entities incl. 8 target slots |
| `radar-test-sen0609.yaml` | SEN0609 at 9600, OUT on D3, all entities incl. speed mode |
| `radar-test-sen0610.yaml` | SEN0610 at 9600, no pin, all entities |
| `secrets.yaml` | fill in `wifi_ssid` / `wifi_password` (placeholders now; keep it out of git) |
| `logs/` | put the captured logs here |

All three configs were validated with `esphome config` against the fork's ESPHome (2026.09.0). They build straight from the checkout (`external_components: type: local`), so no push is needed to start.

Time budget: about 45 min per radar for stages 2–7, plus a 24 h soak on one of them.

**Status:** all three radars have been validated in real use and across all of the below test scenarios.

The SEN0609 & SEN0610 run found one thing: `max_range` and `trigger_range` stop at **2.4 m**, not the 1.2 m declared in the data sheet. These two models refuse to accept ranges below 2.4m.

| Radar | Stages 1–7 | Stage 8 (soak) |
|---|---|---|
| SEN0609 | done | done |
| SEN0395 | done | done |
| SEN0610 | done | done |

---

## Stage 0 — before touching hardware (10 min)

1. Confirm the branch and tests: `components/dfrobot_mmwave/host_tests/run.sh` — expect all green.
2. Edit `examples/bench/secrets.yaml` with your Wi-Fi. Add that path to `.gitignore`.
3. Validate: `esphome config "examples/bench/bench-sen0609.yaml"` (and the other two). Should end without `Failed config`.
4. Build once so the toolchain is cached before you sit at the bench: `esphome compile "examples/bench/bench-sen0609.yaml"`.

---

## Stage 1 — wiring

The harness is identical for all three; only the labels on the radar side change. XIAO ESP32-C6 pin map: **D6 = GPIO16 (ESP TX)**, **D7 = GPIO17 (ESP RX)**, **D3 = GPIO21 (presence input)**.

| XIAO pin | SEN0395 | SEN0609 | SEN0610 |
|---|---|---|---|
| **5V** (USB VBUS) | **V** (needs 3.6–5.5 V) | — | — |
| **3V3** | — | **VIN** (rated 3.3/5 V; 3V3 keeps its TX/OUT at 3.3 V) | **+** |
| **GND** | G | GND | − |
| **D6 / GPIO16** (ESP TX) | RX | RX | **C/R** |
| **D7 / GPIO17** (ESP RX) | TX | TX | **D/T** |
| **D3 / GPIO21** | **IO2** | **OUT** | not connected (no OUT pin) |

Notes:
* TX↔RX cross. If you see the radar's reports but every command times out, swap D6/D7 — the component logs exactly that hint ("radar sends reports but does not answer commands").
* Never feed 5 V into a C6 GPIO. The SEN0395 must be powered from 5 V but its manual states IO2 drives 3.3 V and its UART is 3.3 V logic; the C4001s are powered from 3V3 here so that question doesn't arise.
* SEN0610: if the Gravity board has an I2C/UART selector switch, set it to UART before power-up. (If there is no switch, it auto-detects; the boot log will tell you within 5 s.)
* Keep the radar ≥ 30 cm from the XIAO's antenna and point it at the room, not at your desk.

---

## Stage 2 — first contact (per radar, 10 min)

This stage replaces most of the old probe YAML: at VERBOSE the component logs every byte-level exchange, so the transcript you need falls out of a normal boot.

1. Wire the radar, plug the XIAO in over USB, flash:
   `esphome run "examples/bench/bench-sen0609.yaml"` (pick the file for the radar in hand).
2. Capture the log to a file from the start — this is the transcript that later becomes a replay test:
   `esphome logs "examples/bench/bench-sen0609.yaml" | tee "examples/bench/logs/sen0609-boot-$(date +%Y%m%d-%H%M).log"`
3. Open `http://radar-test-sen0609.local` (or add the device in HA) to watch entities.

What to look for in the first ~20 s of log, tag `[dfrobot_mmwave]`:

| Check | Expected line / entity | Record in results |
|---|---|---|
| Boot wait respected | `status: boot_wait` → `status: probing` about 3 s after the radar has power | — |
| Radar answers | `radar software version: JYSJ_…` then `radar hardware version: …`; both text sensors populated | firmware string (differs per unit) |
| Prompt text | in `RX:`/`echo:` lines: is it `DFRobot:/> ` with a space or `DFRobot:/>` without? (Any `radar prompt '…' does not match` warning means the `model:` is wrong.) | space: yes/no — **[VERIFY #1]** |
| Echo | `echo: getSWV` lines present → echo is on | on/off |
| Get while running | either `radar answers get commands while running` **or** `getRange not answered while running; reading with the radar stopped` | which one — **[VERIFY #2]** |
| Capability probes (C4001 only) | for `led_mode`, `uart_presence_report`, `uart_report_mode`, `uart_report_period`: either a value appears in HA or the log says `<name>: not supported by this firmware` | supported / not — **[VERIFY #3]** |
| All settings read | every number/select/switch shows a value (not "unknown") within ~15 s; `Radar status` = `running`; `Radar last error` empty; `Config pending` off | list any that stayed unknown |
| Reports flowing | SEN0395: `RX: $JYBSS,…` once per second. C4001: `RX: $DFHPD,…` — **count how many arrive per minute with the room empty and still** | C4001 cadence — **[VERIFY #4]** |
| Nothing written | grep the log for `TX: saveConfig` — must be **zero** occurrences during boot | — |
| Terminator | commands are always sent with CR LF; confirm `Done`/`Response` replies arrive without extra `Error` lines after each command | — |

If the radar is silent (no `RX:` at all): check power and the TX/RX cross; then try `baud_rate: "115200"` on a C4001 / `"9600"` on a SEN0395 in case a unit was reconfigured at some point (the component refuses to build with a non-default baud — temporarily change `DEFAULT_BAUD` in `__init__.py` only for this diagnosis, then restore).

Do this stage for all three radars before moving on; it is the cheapest way to find a wiring or firmware surprise.

---

## Stage 3 — presence (per radar, 10 min)

1. Leave the room (or stand > max range away) for 30 s. Expect: `Occupancy` off, `Occupancy (UART)` off, `Occupancy (OUT pin)` off (not on SEN0610).
2. Walk in. Start a stopwatch as you cross the door. Expect `Occupancy` on within `on_latency` (0.05 s C4001 / 0.025 s SEN0395 by default — i.e. immediately) plus the radar's own detection time (< 1 s).
3. Sit still for 60 s. Expect `Occupancy` to stay on (presence, not motion).
4. Leave. Expect `Occupancy` off after `off_latency` (factory 15 s on the C4001, 5 s on the SEN0395) ± 2 s. Write down the measured time.
5. Repeat 2–4 three times. `Occupancy` must go on as soon as *either* source does and off only when both are off (change note 3); note in the log how far apart the OUT-pin flip and the first `$DFHPD` line of the new state are.
6. SEN0395 only: with `Occupancy` on, watch the pin vs UART: the UART report is once per second, the pin is instant — the pin should always change first.

---

## Stage 4 — settings round-trips (per radar, 15 min)

Keep the log open and grep it afterwards: `grep -c "TX: saveConfig" logs/<file>` must equal the number of changes you made (one per change, none extra).

For each row: change the value in HA, then watch `Config pending` go on → off, the log show `transaction #N`, `radar saved its configuration`, `<name> confirmed`, and the entity show the confirmed value.

| Radar | Change | Expected |
|---|---|---|
| C4001 | `Max range` → 4 | confirmed 4 |
| SEN0395 | `Max range` → 4.05 (the slider steps in 0.15 m) | confirmed 4.05; then switch the entity to box mode in HA and type 4 → the radar rounds down, entity shows **3.9**, `last_error` empty (3.9 is what was sent) |
| all | `Off latency` → 20 | confirmed 20; then repeat stage 3 step 4 and measure ≈ 20 s |
| SEN0395 | `Sensitivity` → 5 | confirmed |
| C4001 | `Hold sensitivity` → 6, then `Trigger sensitivity` → 4 (two separate changes, > 1 s apart) | two transactions; log shows `setSensitivity 6 255` then `setSensitivity 255 4` |
| C4001 | `Trigger range` → 30 | **rejected before sending**: `last_error` = `trigger_range: 30.000 is outside 2.400..25.000`, entity reverts |
| C4001 | `Trigger range` → 20 with range 0.6–4 | `Max range` slider jumps to 20 at once; log `trigger_range 20 pushes max_range up to 20`, **one** transaction with `setRange 0.6 20` then `setTrigRange 20`; both confirmed 20, `last_error` empty |
| C4001 | then `Max range` → 4 | `Trigger range` slider jumps to 4; log `max_range 4 pulls trigger_range down to 4`; both confirmed 4 |
| C4001 | `Max range` → 8 (trigger 4) | only `setRange` in the transaction, trigger stays 4 |
| C4001 | `Inhibit time` → 0.1 (box mode) | **rejected before sending**: `last_error` = `inhibit_time: 0.100 is outside 0.300..60.000`, entity reverts |
| C4001 | `Inhibit time` → 2 | confirmed |
| all | drag the `Max range` slider continuously for ~3 s | **one** `TX: saveConfig` in the log, final value confirmed |
| all | change `Max range`, and while the log is inside the transaction (between `TX: sensorStop` and `TX: sensorStart`) change `Off latency` | a second transaction follows the first; both end confirmed |
| all | press **Re-read settings** | one `get` per group in the log; no `saveConfig`; values unchanged |
| all | power-cycle **the radar only** (pull its VCC for 3 s, ESP stays up) | `Radar link` off then on within ~10 s (`radar link lost` → `probing`), status back to `running`, all values re-read and **equal to what you set** (proves `saveConfig` persisted) |
| all | reboot the **ESP** (Restart ESP button) | after boot, values equal to what you set; `grep -c "TX: saveConfig"` did not increase |
| C4001 | `LED` select (if supported) → `off` / `blink` | green LED behaviour changes; **record whether the command is accepted** |
| all | set everything back to factory defaults by hand (range 0–9.45 / 0.6–6, sensitivity 7 / 7+5, latency 0.025+5 / 0.05+15, inhibit 1) | confirmed |

---

## Stage 5 — modes and targets

**C4001 (SEN0609 and SEN0610), 10 min**

1. `Work mode` → `speed_and_distance`. Expect: `transaction`, `TX: setRunApp 1`, `saveConfig`, `sensorStart 1`, then `radar is in speed_and_distance mode` within 5 s, `work_mode confirmed`, `RX: $DFDMD,…` lines.
2. Walk towards and away from the radar: `Targets` 0/1, `Target 1 distance` (m), `Target 1 speed` (m/s), `Target 1 energy` update; `Occupancy` follows `Targets > 0`.
3. **Count `$DFDMD` lines per second** in the log (rate unknown — **[VERIFY #5]**). If it is > 5 Hz, add a `throttle_with_priority: 500ms` filter to the three target sensors in your production YAML.
4. SEN0609 only: watch `Occupancy (OUT pin)` while walking in speed mode. Does the OUT pin still toggle? **[VERIFY #6]** — if yes, the `presence_source` speed-mode override can be relaxed later.
5. `Micro motion` on, `Threshold factor` → 3: both confirmed. Try `Off latency` now: expect `last_error` = `off_latency is only available in presence mode`.
6. `Work mode` → `presence`. Expect `$DFHPD` lines again and the presence-mode settings readable.

**SEN0395, 10 min**

1. `UART target report` on → confirmed; `RX: $JYRPO,…` lines appear while someone is in range.
2. One person walking: `Targets` 1, `Target 1 distance`/`Target 1 SNR` update at 1 Hz; leave the range: `Targets` → 0 within `target_timeout` (2 s) and the slots go to unknown.
3. Two people at different distances: `Targets` 2; note whether `Target 1 distance` < `Target 2 distance` consistently (ordering — **[VERIFY #7]**).
4. `UART report period` → 0.5, `UART report mode` → `on_change`: confirmed; `$JYBSS` now arrives immediately on change plus every 0.5 s. Set back to `periodic` / 1.
5. `UART presence report` off: the log warns `disabling the UART presence report: occupancy will no longer update` — occupancy must still work from the pin (`Occupancy (UART)` freezes, `Occupancy` follows the pin). Turn it back on.

---

## Stage 6 — robustness (per radar, 10 min)

| Action | Expected |
|---|---|
| Pull the radar **TX** wire (D7) for 60 s | after ~30 s: `TX: getSWV` ping, no reply, `radar link lost (no answer to getSWV)`, `Radar link` off, `Radar status` = `link_lost`. `Occupancy` **keeps working from the pin** (SEN0395/SEN0609). Reconnect: back to `running` within one backoff (5–60 s), no ESP reboot |
| Pull the radar **RX** wire (D6) for 60 s | reports keep arriving, commands time out; the next ping/refresh fails → `link_lost`; reconnect → recovers |
| Change a setting while TX is disconnected | entity shows the request, then `last_error` = `operation failed: …`, entity **reverts to the last confirmed value**, `Config pending` off |
| Radar enabled (`running` switch) off | `TX: sensorStop`, `Radar running` off, reports stop, status `stopped`; **no** `link_lost` even after 5 min (the link check is suspended while you stopped it). Switch on → `sensorStart` (`1` on C4001), reports resume |
| **Restart radar** button | `TX: resetSystem 0`, status `boot_wait` → `probing` → `running`, `getSWV` seen twice in the log |
| Quiet room for 5 min (C4001 in presence mode) | if `$DFHPD` is on-change only, you will see one `TX: getSWV` ping every 30 s answered with `radar is quiet but answers commands` and **no** `link_lost` |
| **Factory reset radar** button (do this last) | `sensorStop` → `resetCfg` → `saveConfig` → `sensorStart` → all settings re-read and back to defaults, with `factory reset: <name> = X (was Y)` lines; then set your preferred values again |

---

## Stage 7 — soak (24 h, one radar; SEN0395 preferred because it is the chattiest)

Leave the device running with the test config in a normally used room. Afterwards check in HA history: `Heap free` flat (no downward trend over 24 h), `Loop time` steady, `Radar link` never off, `Radar last error` empty, `Config pending` never stuck on, and in the log `grep -c "TX: saveConfig"` = 0 and `grep -c "link lost"` = 0. Occupancy history should look like the room's real use, with no gaps.

---

## Stage 8 — close out

1. Turn each radar's `logs/*-boot-*.log` transcript into a file in `components/dfrobot_mmwave/host_tests/replay/` (format at the top of `test_replay.cpp`; `sen0395_bench.txt` is the first) and add a replay test for it that checks what the engine published.
2. Fill in the results table below and hand it back; every **[VERIFY #n]** answer either confirms the current default or points at one line to change in `mmwave_dialect.h` / `mmwave_params.cpp` / the `presence_source` speed-mode override.
3. Update the README section "Things still to confirm on hardware" with the outcomes, then tag a release and pin it in your production YAML (`source: github://igiannakas/esphome-dfrobot-mmwave@v1.0.0`).

### Results template

| # | Question | SEN0395 | SEN0609 | SEN0610 |
|---|---|---|---|---|
| 1 | Prompt has trailing space? | | | |
| 2 | `get*` answered while running? | | | |
| 3 | LED / UART-output commands supported? | n/a (documented) | | |
| 4 | Presence report cadence in an empty room | 1 Hz | | |
| 5 | `$DFDMD` rate in speed mode | n/a | | |
| 6 | OUT pin toggles in speed mode? | n/a | | n/a |
| 7 | `$JYRPO` targets ordered by distance? | | n/a | n/a |
| 8 | Bare commands (no CR LF) accepted? | | | |
| — | Firmware version string | | | |
| — | Measured off-latency at 15 s / 20 s | | | |
| — | `saveConfig` count vs changes made | | | |
| — | Trigger/max range coupling: both confirmed, no `last_error` | n/a | | |
| — | Anything in `last_error` you did not cause | | | |
