"""Switch entities for the on/off radar settings and the running state."""

import esphome.codegen as cg
from esphome.components import switch
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_SWITCH,
    ENTITY_CATEGORY_CONFIG,
    ICON_LIGHTBULB,
    ICON_MOTION_SENSOR,
    ICON_PULSE,
)
from esphome.types import ConfigType

from .. import (
    ALL_MODELS,
    CONF_DFROBOT_MMWAVE_ID,
    PARAMETERS,
    DfrobotMmwave,
    dfrobot_mmwave_ns,
    validate_entities_for_model,
    with_default_name,
)

DEPENDENCIES = ["dfrobot_mmwave"]

ParamSwitch = dfrobot_mmwave_ns.class_("ParamSwitch", switch.Switch)
RunningSwitch = dfrobot_mmwave_ns.class_("RunningSwitch", switch.Switch)

CONF_RUNNING = "running"

# key -> (icon, default name)
PARAM_SWITCHES: dict[str, tuple[str, str]] = {
    # LED on = the radar's heartbeat blink while running (its factory state)
    "led": (ICON_LIGHTBULB, "LED"),
    "speed_micro_motion": (ICON_MOTION_SENSOR, "Speed mode micro motion"),
    "uart_presence_report": (ICON_PULSE, "UART presence report"),
    "uart_target_report": (ICON_PULSE, "UART target report"),
}


def _param_switch_schema(icon: str, default_name: str) -> cv.Schema:
    return with_default_name(
        switch.switch_schema(
            ParamSwitch,
            device_class=DEVICE_CLASS_SWITCH,
            entity_category=ENTITY_CATEGORY_CONFIG,
            icon=icon,
            block_inverted=True,
        ),
        default_name,
    )


CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_DFROBOT_MMWAVE_ID): cv.use_id(DfrobotMmwave),
        # The radar does not persist sensorStart/sensorStop; nothing is restored or written at boot.
        cv.Optional(CONF_RUNNING): with_default_name(
            switch.switch_schema(
                RunningSwitch,
                device_class=DEVICE_CLASS_SWITCH,
                entity_category=ENTITY_CATEGORY_CONFIG,
                icon="mdi:radar",
                block_inverted=True,
            ),
            "Radar enabled",
        ),
        **{
            cv.Optional(key): _param_switch_schema(icon, name)
            for key, (icon, name) in PARAM_SWITCHES.items()
        },
    }
)

FINAL_VALIDATE_SCHEMA = validate_entities_for_model(
    {CONF_RUNNING: ALL_MODELS} | {key: PARAMETERS[key][1] for key in PARAM_SWITCHES}
)


async def to_code(config: ConfigType) -> None:
    hub = await cg.get_variable(config[CONF_DFROBOT_MMWAVE_ID])
    if (conf := config.get(CONF_RUNNING)) is not None:
        s = await switch.new_switch(conf)
        await cg.register_parented(s, config[CONF_DFROBOT_MMWAVE_ID])
        cg.add(hub.set_running_switch(s))
    for key in PARAM_SWITCHES:
        if (conf := config.get(key)) is None:
            continue
        param_id = PARAMETERS[key][0]
        s = cg.new_Pvariable(conf[CONF_ID], param_id)
        await switch.register_switch(s, conf)
        await cg.register_parented(s, config[CONF_DFROBOT_MMWAVE_ID])
        cg.add(hub.set_param_switch(param_id, s))
