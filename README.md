# DFRobot mmWave presence radars for ESPHome

An ESPHome component for DFRobot's three 24 GHz presence radars: the **SEN0395** (the original "leapMMW" module, 9 m), the **SEN0609** (C4001, 25 m) and the **SEN0610** (C4001, 12 m). Wire one up to an ESP32 and Home Assistant gets an occupancy sensor plus every setting the radar has, as proper entities you can change from the UI.

The three radars run closely related firmware, which is why one component covers all of them. The module takes care of the underlying variations in radar capabilities.

## What it does

The radar detects whether someone is in the room and exposes it into a Home Assistant `occupancy` sensor. This module also lets you tune the radar parameters as per the full sensor data sheet.

- **Occupancy** from the radar's OUT pin, from its serial reports, or both. If you wire both, the component uses whichever is alive, so a loose wire on one side does not take the sensor down.
- **Every setting as an entity.** Detection range, trigger range, sensitivities, on/off delays, the LED, the reporting settings, and on the C4001 the work mode. Change a slider in Home Assistant and the radar is updated and saved. Settings live in the radar's own memory, so they survive reflashing the ESP.
- **The radar is the source of truth.** After every change the component reads the value back and shows what the radar actually kept. If that differs from what you asked for, the `last_error` sensor tells you.
- **Fault tolerance.** Occupancy keeps working from whatever source is still alive (OUT pin, UART) and picks up where it left off when the module recovers, making the component tolerant to wire faults and interference.
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

All three radars talk over a 3.3 V serial link. The two with an OUT pin also give a high/low presence signal, which is worth wiring: it's instant, and it keeps working if the serial side has a problem.

| | SEN0395 | SEN0609 | SEN0610 |
|---|---|---|---|
| Power | `V` — needs 3.6–5.5 V (use the ESP's 5 V pin) | `VIN` — 3.3 V or 5 V | `+` — 3.3 V or 5 V |
| ESP TX → radar | `RX` | `RX` | `C/R` |
| ESP RX ← radar | `TX` | `TX` | `D/T` |
| Presence pin → ESP | `IO2` | `OUT` | none |
| Serial speed | 115200 | 9600 | 9600 |

## Component/Hub

```yaml
external_components:
  - source: github://igiannakas/esphome-dfrobot-mmwave
    components: [dfrobot_mmwave]

uart:
  id: radar_uart
  tx_pin: GPIO18
  rx_pin: GPIO20
  baud_rate: 9600

dfrobot_mmwave:
  id: radar
  uart_id: radar_uart
  model: SEN0609
  presence_pin: GPIO19
  target_timeout: 2s
```

### Configuration variables

- **model** (**Required**, string): `SEN0395`, `SEN0609` or `SEN0610`. The UART must run at the radar's factory speed (see [Wiring](#wiring)); any other baud rate is rejected.
- **presence_pin** (*Optional*, [Pin Schema](https://esphome.io/guides/configuration-types#pin-schema)): The radar's OUT pin (`IO2` on the SEN0395). Set up as an input with pull-down, so a disconnected wire reads as no presence. Not available on the SEN0610.
- **target_timeout** (*Optional*, [Time](https://esphome.io/guides/configuration-types#time)): How long without a target report before the target sensors are cleared. Applies to the SEN0395 and to the C4001 in speed-and-distance mode. Defaults to `2s`.
- **uart_id** (*Optional*, [ID](https://esphome.io/guides/configuration-types#id)): The ID of the [UART component](https://esphome.io/components/uart). Only needed with more than one UART.
- **id** (*Optional*, [ID](https://esphome.io/guides/configuration-types#id)): The ID of this component. Only needed with more than one radar.

## How it works

- **Changes are batched.** About a second after the last change the component stops the radar, writes every changed setting, saves, starts it again and reads the values back. One flash write per batch.
- **Values are read back.** Each entity shows the value the radar kept. If it differs from the request, `last_error` reads `<setting>: requested <value>, radar reports <value>`. It clears after the next clean operation.
- **Trigger range and max range stay consistent.** Moving one past the other moves the other with it.
- **Occupancy** is on when either live source reports presence. The OUT pin is always live. The radar sends a serial report on every change and repeats it every `uart_report_period`; after three periods without one, the serial source is dropped. With neither source live (radar stopped or disconnected) occupancy is unknown. On the SEN0395, presence reads clear for a few seconds after a setting change restarts the radar.
- **Link loss.** When commands go unanswered, `status` reads `link_lost`, `link_ok` turns off and the component keeps probing. When the radar answers again every setting is reread and, if a stop command got through before the link died, the radar is started again. A change made while the link was down is not retried.
- **Target tracking.** SEN0395: up to 8 targets (distance, SNR) alongside presence. C4001: one target (distance, speed) in speed-and-distance mode.

## Entities

Every entity below is optional, has a default name and accepts all options of its platform. Each platform also takes:

- **dfrobot_mmwave_id** (*Optional*, [ID](https://esphome.io/guides/configuration-types#id)): The ID of the radar. Only needed with more than one radar.

Numbers and sensors of the other C4001 work mode show as unknown; switches keep their last value.

### Binary Sensor

```yaml
binary_sensor:
  - platform: dfrobot_mmwave
    occupancy:
      name: Occupancy
    out_pin_occupancy:           # SEN0395, SEN0609
      name: Occupancy (OUT pin)
    uart_occupancy:
      name: Occupancy (UART)
    link_ok:
      name: Radar link
```

- **occupancy**: Presence from the OUT pin or the serial reports, whichever is live.
- **out_pin_occupancy**: The OUT pin on its own. Requires `presence_pin`. SEN0395 and SEN0609.
- **uart_occupancy**: The serial reports on their own. Unknown while they have stopped.
- **link_ok**: On while the component can talk to the radar.

All other options from [Binary Sensor](https://esphome.io/components/binary_sensor).

### Sensor

```yaml
sensor:
  - platform: dfrobot_mmwave
    target_count:
      name: Target count
    target_1_distance:
      name: Target 1 distance
      filters:
        - throttle_with_priority: 500ms
    target_1_speed:              # C4001
      name: Target 1 speed
      filters:
        - throttle_with_priority: 500ms
    target_1_energy:             # C4001
      name: Target 1 energy
      filters:
        - throttle_with_priority: 500ms
    target_1_snr:                # SEN0395
      name: Target 1 SNR
```

- **target_count**: Number of targets. SEN0395: up to 8. C4001: 0 or 1, speed-and-distance mode.
- **target_1_distance** … **target_8_distance**: Target distance, m. SEN0395: targets 1–8. C4001: target 1, speed-and-distance mode.
- **target_1_snr** … **target_8_snr**: Target SNR. SEN0395.
- **target_1_speed**: Target speed, m/s, positive moving away, negative approaching. C4001, speed-and-distance mode.
- **target_1_energy**: The radar's own signal strength figure, no unit. C4001, speed-and-distance mode.

In speed-and-distance mode the target sensors update about ten times a second. Use a `throttle_with_priority` filter rather than `throttle`, which can drop the single "no target" update and leave the last distance showing.

All other options from [Sensor](https://esphome.io/components/sensor).

### Number

```yaml
number:
  - platform: dfrobot_mmwave
    min_range:
      name: Min range
    max_range:
      name: Max range
    trigger_range:               # C4001
      name: Trigger range
    sensitivity:                 # SEN0395
      name: Sensitivity
    hold_sensitivity:            # C4001
      name: Hold sensitivity
    trigger_sensitivity:         # C4001
      name: Trigger sensitivity
    on_latency:
      name: On latency
    off_latency:
      name: Off latency
    inhibit_time:                # C4001
      name: Inhibit time
    uart_report_period:
      name: UART report period
    speed_threshold_factor:      # C4001
      name: Speed and distance threshold factor
```

Settings stored in the radar. Minimum, maximum and step are set per model from what the firmware accepts, measured on the bench. The mode column is the C4001 work mode the setting belongs to.

| Key | Mode | SEN0395 | SEN0609 | SEN0610 | What it sets |
|---|---|---|---|---|---|
| `min_range` | both | 0–9.3 m | 0.3–25.9 m | 0.3–11.9 m | detection start distance; must stay below `max_range`. SEN0395 rounds to 0.15 m steps |
| `max_range` | both | 0.15–9.45 m | 2.4–26 m | 1.2–12 m | detection end distance; 26 m in speed-and-distance mode, 25 m in presence mode |
| `trigger_range` | presence | — | 2.4–25 m | 1.2–12 m | distance within which new presence is detected; never above `max_range` |
| `sensitivity` | — | 0–9 | — | — | detection sensitivity |
| `hold_sensitivity` | presence | — | 0–9 | 0–9 | how easily presence, once detected, is held |
| `trigger_sensitivity` | presence | — | 0–9 | 0–9 | how easily new presence is detected (DFRobot suggest 2–6) |
| `on_latency` | presence | 0–100 s | 0–2 s | 0–2 s | how long presence must be detected before occupied |
| `off_latency` | presence | 0.5–1500 s | 2–1500 s | 2–1500 s | how long after the last detection before clear |
| `inhibit_time` | presence | — | 0.3–60 s | 0.3–60 s | after presence clears, how long before new presence can be detected |
| `uart_report_period` | presence | 0.025–1500 s | 0.2–1500 s | 0.2–1500 s | interval of the repeated serial presence report |
| `speed_threshold_factor` | speed and distance | — | 0–65535 | 0–65535 | |

`off_latency`, `uart_report_period` and `speed_threshold_factor` show as a text box; add `mode: slider` to change that.

All other options from [Number](https://esphome.io/components/number).

### Select

```yaml
select:
  - platform: dfrobot_mmwave
    work_mode:                   # C4001
      name: Work mode
```

- **work_mode**: `presence` or `speed_and_distance`. Switching restarts the radar in the other mode; each mode keeps its own range and LED setting. SEN0609 and SEN0610.

All other options from [Select](https://esphome.io/components/select).

### Switch

```yaml
switch:
  - platform: dfrobot_mmwave
    running:
      name: Radar enabled
    led:
      name: LED
    uart_presence_report:
      name: UART presence report
    uart_target_report:          # SEN0395
      name: UART target report
    speed_micro_motion:          # C4001
      name: Speed and distance micro motion
```

- **running**: Starts and stops the radar. Not remembered by the radar across power cycles.
- **led**: The radar's LED. On: blinks once a second while running (factory setting).
- **uart_presence_report**: The serial presence reports. Off: only the OUT pin provides occupancy.
- **uart_target_report**: The per-target distance/SNR reports. SEN0395.
- **speed_micro_motion**: On: also detects micro motion such as breathing. Off: larger movements only. C4001, speed-and-distance mode.

All other options from [Switch](https://esphome.io/components/switch).

### Button

```yaml
button:
  - platform: dfrobot_mmwave
    refresh:
      name: Reread radar settings
    restart:
      name: Restart radar
    factory_reset:
      name: Factory reset radar
```

- **refresh**: Drops any unapplied change and rereads every setting from the radar.
- **restart**: Reboots the radar.
- **factory_reset**: Restores the radar's factory settings and reads them back.

All other options from [Button](https://esphome.io/components/button).

### Text Sensor

```yaml
text_sensor:
  - platform: dfrobot_mmwave
    status:
      name: Radar status
    last_error:
      name: Radar last error
    software_version:
      name: Radar firmware
    hardware_version:
      name: Radar hardware
```

- **status**: `boot_wait`, `probing`, `reading`, `applying`, `running`, `stopped`, `link_lost` or `unsupported_firmware`.
- **last_error**: The most recent problem, in words. Empty once an operation goes through cleanly.
- **software_version**, **hardware_version**: As reported by the radar. The C4001 has a different version string per mode.

All other options from [Text Sensor](https://esphome.io/components/text_sensor).

## Actions

### `dfrobot_mmwave.refresh`, `dfrobot_mmwave.restart`, `dfrobot_mmwave.factory_reset`

The same as the buttons.

```yaml
on_...:
  - dfrobot_mmwave.refresh: radar
  - dfrobot_mmwave.restart: radar
  - dfrobot_mmwave.factory_reset: radar
```

- **id** (*Optional*, [ID](https://esphome.io/guides/configuration-types#id)): The radar. Only needed with more than one radar.

### `dfrobot_mmwave.set_parameter`

Sets one radar setting.

```yaml
on_...:
  - dfrobot_mmwave.set_parameter:
      id: radar
      parameter: max_range
      value: 4.5
```

- **id** (*Optional*, [ID](https://esphome.io/guides/configuration-types#id)): The radar. Only needed with more than one radar.
- **parameter** (**Required**, string): Any number, select or switch key above except `running`.
- **value** (**Required**, float, templatable): The new value. Selects take the option index (`presence` = 0, `speed_and_distance` = 1), switches 0 or 1. Fixed values are checked against the model's limits when the config is validated.

## Setting the range with speed-and-distance mode

Speed-and-distance mode is most useful from the device's web page while you install the sensor, which is why the example configs keep its entities out of Home Assistant. Switch `work_mode` to `speed_and_distance`, then:

1. With the room empty, check that no target is reported. A target in an empty room is interference.
2. Walk to the furthest point where you want to be detected and note the target distance.
3. Switch back to `presence` and set `max_range` to that distance. The two modes keep separate ranges, so it has to be set in presence mode.

## Example configs

- `examples/sen0609-xiao-esp32c6.yaml`: SEN0609 on a XIAO ESP32-C6.
- `examples/sen0609-wemos-d1-mini32.yaml`: SEN0609 on a Wemos D1 mini32.
- `examples/bench/`: bench configs for all three radars.

## Factory settings

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

The SEN0609 and the SEN0395 have been through the hardware test plan (`docs/hardware-test-plan.md`) apart from the 24-hour soak; the SEN0610 not yet.

1. **Validate the SEN0610 on hardware.** Its 12 m limit and factory defaults are from the datasheet and may be wrong in the same ways the SEN0609's were. The SEN0395's old `detRangeCfg` firmware is detected and left read-only, but has not been seen on the bench.
2. **A 24-hour soak on each radar**, watching heap, uptime and the log for anything unexpected.
3. **Check whether a factory reset on a C4001 also resets the speed-and-distance mode's settings.** It restores the presence mode's; the other mode wasn't checked.
4. **More replay tests.** There is one SEN0395 session in `host_tests/replay/`, rebuilt from the bench notes; raw serial captures of each radar would make it stronger and add the C4001s.

## Tests

```bash
components/dfrobot_mmwave/host_tests/run.sh               # protocol tests
for f in tests/test-*.yaml; do esphome config "$f"; done  # every entity of every model validates
components/dfrobot_mmwave/host_tests/validate_negative.sh # bad configs are rejected
```

CI runs all three on every push.

## AI use

AI agents were used to assist with designing the code architecture and deliver boilerplate code that supports the implementation of this module. The design decisions, code review and exhaustive bench testing on real radars were done by the author. The code is validated by the protocol and config tests that run on every push.

## License

GPL-3.0 — see `LICENSE`.
