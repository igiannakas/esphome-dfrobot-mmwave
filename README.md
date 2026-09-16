# DFRobot mmWave presence radars for ESPHome

An ESPHome component for DFRobot's three 24 GHz presence radars: the **SEN0395** (the original "leapMMW" module, 9 m), the **SEN0609** (C4001, 25 m) and the **SEN0610** (C4001, 12 m). Wire one up to a ESP32 and Home Assistant gets an occupancy sensor plus every setting the radar has, as proper entities you can change from the UI.

The three radars run closely related firmware, which is why one component covers all of them. The module takes care of the underlying variations in radar capabilities.

## What it does

The radar detects whether someone is in the room and exposes it into a Home Assistant `occupancy` sensor. This module also lets you tune the radar parameters as per the full sensor data sheet.

- **Occupancy** from the radar's OUT pin, from its serial reports, or both. If you wire both, the component uses whichever is alive, so a loose wire on one side does not take the sensor down.
- **Every setting as an entity.** Detection range, trigger range, sensitivities, on/off delays, the LED, the reporting settings, and on the C4001 the work mode. Change a slider in Home Assistant and the radar is updated and saved. Settings live in the radar's own memory, so they survive reflashing the ESP.
- **The radar is the source of truth.** After every change the component reads the value back and shows what the radar actually kept. If you ask for something the firmware won't do (a trigger range below 2.4 m, say) the slider snaps back to the real value and the `last_error` sensor tells you why.
- **Fault tolerance.** A radar that stops answering, a wire that comes loose, a power cycle on the radar side: the component notices, keeps occupancy working from whatever source is still alive, and picks up where it left off when the link comes back.
- **Both C4001 modes.** The SEN0609 and SEN0610 run in presence mode or in speed-and-distance mode, where the radar tracks one target and reports its distance and speed. You can switch between them from Home Assistant or the device's web page.

## Enhancements over the built-in component

ESPHome ships a `dfrobot_sen0395` component. While that module is functional, it only supports the SEN0395, it writes settings from YAML at boot rather than exposing them, and it doesn't work with the C4001 radars at all. This component started as a way to get the SEN0609 & SEN0610 working and grew from there. Key differences are outlined below.

| | built-in `dfrobot_sen0395` | this component |
|---|---|---|
| **Radars** | SEN0395 | SEN0395, SEN0609, SEN0610 |
| **Settings** | applied from YAML via actions, not visible in HA | Home Assistant entities, read back from the radar |
| **Occupancy source** | serial reports | OUT pin and/or serial reports |
| **Link problems** | not handled | detected, recovered, reported in `status` and `last_error` |
| **C4001 speed-and-distance mode** | — | yes, with distance, speed and energy sensors |
| **Setting limits** | datasheet values | measured on hardware where the datasheet is wrong |

## Wiring

All three radars talk over a 3.3 V serial link. The two with an OUT pin also give you a plain high/low presence signal, which is worth wiring: it's instant, and it keeps working if the serial side has a problem.

| | SEN0395 | SEN0609 | SEN0610 |
|---|---|---|---|
| Power | `V` — needs 3.6–5.5 V (use the ESP's 5 V pin) | `VIN` — 3.3 V or 5 V | `+` — 3.3 V or 5 V |
| ESP TX → radar | `RX` | `RX` | `C/R` |
| ESP RX ← radar | `TX` | `TX` | `D/T` |
| Presence pin → ESP | `IO2` | `OUT` | none |
| Serial speed | 115200 | 9600 | 9600 |

## Getting started

Add the component, a UART, and the radar. This is the smallest useful config for a SEN0609:

```yaml
external_components:
  - source: github://igiannakas/esphome-dfrobot-mmwave
    components: [dfrobot_mmwave]

uart:
  id: radar_uart
  tx_pin: GPIO18
  rx_pin: GPIO20
  baud_rate: 9600          # 115200 for the SEN0395

dfrobot_mmwave:
  id: radar
  uart_id: radar_uart
  model: SEN0609           # SEN0395, SEN0609 or SEN0610
  presence_pin: GPIO19     # leave out on the SEN0610, or if OUT isn't wired

binary_sensor:
  - platform: dfrobot_mmwave
    occupancy:
      name: Occupancy
```

That gives you an occupancy sensor and nothing else. The settings only become entities when you add them, so add what you want to see in Home Assistant. Every entity has a sensible default name, so an empty entry (`max_range: `) is enough if you don't need to rename the entity. A complete config for the SEN0609 looks like this:

```yaml
binary_sensor:
  - platform: dfrobot_mmwave
    occupancy:
      name: Occupancy
    link_ok:
      name: Radar link

number:
  - platform: dfrobot_mmwave
    min_range:
      name: Min range
    max_range:
      name: Max range
    trigger_range:
      name: Trigger range
    hold_sensitivity:
      name: Hold sensitivity
    trigger_sensitivity:
      name: Trigger sensitivity
    on_latency:
      name: On latency
    off_latency:
      name: Off latency
    inhibit_time:
      name: Inhibit time

switch:
  - platform: dfrobot_mmwave
    running:
      name: Radar enabled
    led:
      name: LED

button:
  - platform: dfrobot_mmwave
    refresh:
      name: Reread radar settings
    restart:
      name: Restart radar
    factory_reset:
      name: Factory reset radar

text_sensor:
  - platform: dfrobot_mmwave
    status:
      name: Radar status
    last_error:
      name: Radar last error
```

There are complete, commented production configs for a XIAO ESP32-C6 in `examples/sen0609-xiao-esp32c6.yaml` and a Wemos D1 mini32 in `examples/sen0609-wemos-d1-mini32.yaml`, and bench configs for all three radars in `examples/bench/`.

## How it behaves

**Nothing is written to the radar at boot.** The component reads every setting once the radar answers and publishes what is stored in the radar memory.

**Changes are grouped.** Drag a slider and the component waits until you've stopped (about a second), then sends everything that changed in one go. It stops the radar, writes the settings, saves, starts it again, reads the values back. One flash write per batch, not one per slider notch.

**Trigger range and max range are kept consistent for you.** Lowering one below the other, or raising one above the other, moves the other with it.

**What you see is what the radar has.** The entity shows your new value straight away, and is corrected a moment later if the radar kept something else. The `last_error` text sensor explains it — "trigger_range: requested 1.2, radar reports 2.4" — and clears itself the next time an operation goes through cleanly. Values the firmware can't accept at all are refused before anything is sent.

**Occupancy comes from whatever output is connected (UART/Out pin).** The OUT pin, when wired, is always current. The serial reports count for as long as they keep arriving: the radar sends a line on every change and a keep-alive every `uart_report_period`, and if three periods pass without one the serial source is dropped so a dead link can't hold occupancy on. If neither source is alive — the radar is stopped, or unplugged — occupancy shows as unavailable rather than pretending the room is empty. Changing a setting stops and restarts the radar; on the SEN0395 presence reads clear for a few seconds after the restart, so expect a short blip if the room is occupied at the time.

**Losing the radar is handled.** If commands go unanswered the component backs off and probes again, `status` reads `link_lost` and `link_ok` goes off. When the radar answers again everything is reread, and if a stop command got through before the link died the radar is started again. A change you made while the link was down is not retried; the entity goes back to the radar's value and `last_error` says what happened.

**The C4001 has two work modes: presence and speed-and-distance.** Switching `work_mode` restarts the radar in the other mode. Each mode keeps its own range and LED setting. The presence settings (sensitivities, latencies, inhibit time, reporting) are unavailable in speed-and-distance mode, and the speed-and-distance settings (`speed_micro_motion`, `speed_threshold_factor`, the target sensors) are unavailable in presence mode. In speed-and-distance mode the radar tracks a single target and reports its distance, speed and energy. There are no delays: the OUT pin and the reports follow the target instantly, and the target sensors update about ten times a second, so put a `throttle_with_priority` filter on them if you enable them.

**Speed-and-distance mode is a setup tool.** It's most useful from the device's web page while you install the sensor, which is why the example config keeps its entities out of Home Assistant. Switch `work_mode` to `speed_and_distance`, then:

1. With the room empty, check that no target is reported. A target in an empty room is interference.
2. Walk to the furthest point where you want to be detected and note the target distance.
3. Switch back to `presence` and set `max_range` to that distance. The two modes keep separate ranges, so it has to be set in presence mode.

## Options

### The radar itself

```yaml
dfrobot_mmwave:
  id: radar
  uart_id: radar_uart
  model: SEN0609
  presence_pin: GPIO19
  target_timeout: 2s
```

- **model** (required): `SEN0395`, `SEN0609` or `SEN0610`. This also fixes the serial speed — the config is rejected if the UART is set to anything else, which catches speed discrepancies (eg. "SEN0395 at 9600")
- **presence_pin** (optional): the OUT / IO2 pin. A bare pin number is enough; the component configures it as an input with a pull-down so an unplugged wire reads as "nobody here". Not allowed on the SEN0610, which has no such pin.
- **target_timeout** (default 2 s): how long without a target report before the target sensors are cleared. Only matters for the SEN0395's target list and the C4001's speed-and-distance mode.

### Entities

Add any of these under their respective platform. Everything is optional and takes the usual ESPHome entity options (`name`, `id`, `icon`, `filters` and so on). "Presence" and "speed and distance" in the mode column say which C4001 work mode the entity belongs to; the SEN0395 only has the one mode.

**binary_sensor**

| Key | Models | What it is |
|---|---|---|
| `occupancy` | all | someone is there, from the pin and/or the serial reports |
| `out_pin_occupancy` | SEN0395, SEN0609 | the OUT pin on its own (needs `presence_pin`) |
| `uart_occupancy` | all | the serial reports on their own; unavailable while they've stopped |
| `link_ok` | all | the component can talk to the radar |

**number** — the sliders. Ranges are what the firmware actually accepts, measured on the bench.

| Key | Mode | SEN0395 | SEN0609 | SEN0610 | Notes |
|---|---|---|---|---|---|
| `min_range` | both | 0–9.3 m | 0.3–25.9 m | 0.3–11.9 m | SEN0395 rounds to 0.15 m steps; must stay below `max_range` |
| `max_range` | both | 0.15–9.45 m | 2.4–26 m | 2.4–12 m | 26 m in speed-and-distance mode; presence mode goes to 25 |
| `trigger_range` | presence | — | 2.4–25 m | 2.4–12 m | distance within which new presence is detected; never above `max_range` |
| `sensitivity` | both | 0–9 | — | — | |
| `hold_sensitivity` | presence | — | 0–9 | 0–9 | how easily presence, once detected, is held |
| `trigger_sensitivity` | presence | — | 0–9 | 0–9 | how easily new presence is detected (DFRobot suggest 2–6) |
| `on_latency` | presence | 0–100 s | 0–2 s | 0–2 s | how long presence must be detected before "occupied" |
| `off_latency` | presence | 0.5–1500 s | 2–1500 s | 2–1500 s | how long after the last detection before "clear" |
| `inhibit_time` | presence | — | 0.3–60 s | 0.3–60 s | after presence clears, how long before a new presence can be detected |
| `uart_report_period` | presence | 0.025–1500 s | 0.2–1500 s | 0.2–1500 s | keep-alive interval of the serial reports |
| `speed_threshold_factor` | speed and distance | — | 0–65535 | 0–65535 | |

The wide-range ones (`off_latency`, `uart_report_period`, `speed_threshold_factor`) show as a text
box rather than a slider. Add `mode: slider` to an entity if you prefer.

**select**

| Key | Models | Options |
|---|---|---|
| `work_mode` | SEN0609, SEN0610 | `presence`, `speed_and_distance` |

**switch**

| Key | Models | What it does |
|---|---|---|
| `running` | all | starts and stops the radar; not remembered by the radar across power cycles |
| `led` | all | the radar's LED: on = a blink every second while running, which is how it ships |
| `uart_presence_report` | all | the serial presence reports; off means only the pin can provide occupancy |
| `uart_target_report` | SEN0395 | the per-target distance/SNR reports |
| `speed_micro_motion` | SEN0609, SEN0610 (speed-and-distance mode) | on: also detects micro motion such as breathing; off: larger movements only |

**button**

| Key | What it does |
|---|---|
| `refresh` | drops any unapplied change and rereads everything from the radar |
| `restart` | reboots the radar |
| `factory_reset` | resets the radar to its factory settings and rereads them |

**sensor**

| Key | Models | What it is |
|---|---|---|
| `target_count` | all | targets seen (SEN0395, up to 8); on a C4001 0 or 1, in speed-and-distance mode |
| `target_1_distance` … `target_8_distance` | SEN0395 (1–8), SEN0609, SEN0610 (target 1, speed-and-distance mode) | metres |
| `target_1_snr` … `target_8_snr` | SEN0395 | |
| `target_1_speed` | SEN0609, SEN0610 (speed-and-distance mode) | m/s, positive moving away, negative approaching |
| `target_1_energy` | SEN0609, SEN0610 (speed-and-distance mode) | the radar's own signal strength figure, no unit |

**text_sensor**

| Key | What it is |
|---|---|
| `status` | `boot_wait`, `probing`, `reading`, `applying`, `running`, `stopped`, `link_lost` or `unsupported_firmware` |
| `last_error` | the most recent problem, in words; empty once things are fine again |
| `software_version`, `hardware_version` | as reported by the radar (the C4001 has a different version string per mode) |

### Actions

For automations, the same things the buttons do, plus a way to set any setting from a lambda:

```yaml
on_...:
  - dfrobot_mmwave.refresh: radar
  - dfrobot_mmwave.restart: radar
  - dfrobot_mmwave.factory_reset: radar
  - dfrobot_mmwave.set_parameter:
      id: radar
      parameter: max_range
      value: 4.5
```

`parameter` takes any of the entity keys above; selects and switches take the option index or 0/1.

## Factory settings, for reference

Both columns were read back from the radar after a factory reset.

| | SEN0395 | SEN0609 |
|---|---|---|
| Detection range | 0 – 9.45 m | 0.6 – 6 m |
| Trigger range | — | 6 m |
| Sensitivity | 7 | hold 7, trigger 5 |
| On / off latency | 0.025 / 5 s | 0.05 / 15 s |
| Inhibit time | — | 1 s |
| LED | on | on |
| UART presence report | on, every 1 s | not recorded |
| UART target report | off | — |

The SEN0610 is expected to match the SEN0609 within its 12 m range.

## Still to do

The SEN0609 and the SEN0395 have been through the hardware test plan (`docs/hardware-test-plan.md`) apart from the 24-hour soak; the SEN0610 not yet. In rough order of usefulness:

1. **Validate the SEN0610 on hardware.** Its 12 m limit and factory defaults are from the datasheet and may be wrong in the same ways the SEN0609's were. The SEN0395's old `detRangeCfg` firmware is detected and left read-only, but has not been seen on the bench.
2. **A 24-hour soak on each radar**, watching heap, uptime and the log for anything unexpected.
4. **Check whether a factory reset on a C4001 also resets the speed-and-distance mode's settings.** It restores the presence mode's; the other mode wasn't checked.
5. **More replay tests.** There is one SEN0395 session in `host_tests/replay/`, rebuilt from the bench notes; raw serial captures of each radar would make it stronger and add the C4001s.

## Tests

```bash
components/dfrobot_mmwave/host_tests/run.sh            # protocol tests: simulated radar and a replayed session
for f in tests/test-*.yaml; do esphome config "$f"; done # every entity of every model validates
components/dfrobot_mmwave/host_tests/validate_negative.sh # bad configs are rejected with the right message
```

CI runs all three on every push. The host tests need only `g++` and `python3`.

## Repository layout

| Path | Contents |
|---|---|
| `components/dfrobot_mmwave/` | the component |
| `components/dfrobot_mmwave/host_tests/` | the simulated-radar tests, the replayed radar sessions and the negative configs |
| `examples/` | the production configs and the bench configs |
| `tests/` | config-validation tests |
| `docs/hardware-test-plan.md` | the hardware test plan and its results |

## AI use

AI tools were used to assist in developing this component. The design decisions and the bench testing on real radars were done by the author, and the code is checked by the protocol and config tests that run on every push.

## License

GPL-3.0 — see `LICENSE`.
