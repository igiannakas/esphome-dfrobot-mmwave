"""Number entities for the numeric radar settings."""

import esphome.codegen as cg
from esphome.components import number
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    CONF_MODE,
    DEVICE_CLASS_DISTANCE,
    DEVICE_CLASS_DURATION,
    ENTITY_CATEGORY_CONFIG,
    ICON_MOTION_SENSOR,
    ICON_TIMELAPSE,
    UNIT_METER,
    UNIT_SECOND,
)
from esphome.types import ConfigType

from .. import (
    CONF_DFROBOT_MMWAVE_ID,
    PARAMETERS,
    DfrobotMmwave,
    dfrobot_mmwave_ns,
    hub_model,
    number_limits,
    number_mode,
    validate_entities_for_model,
    with_default_name,
)

DEPENDENCIES = ["dfrobot_mmwave"]

ParamNumber = dfrobot_mmwave_ns.class_("ParamNumber", number.Number)

_DISTANCE = {"device_class": DEVICE_CLASS_DISTANCE, "unit_of_measurement": UNIT_METER}
_DURATION = {"device_class": DEVICE_CLASS_DURATION, "unit_of_measurement": UNIT_SECOND}

NUMBERS: dict[str, dict] = {
    "min_range": _DISTANCE,
    "max_range": _DISTANCE,
    "trigger_range": _DISTANCE,
    "sensitivity": {"icon": ICON_MOTION_SENSOR},
    "hold_sensitivity": {"icon": ICON_MOTION_SENSOR},
    "trigger_sensitivity": {"icon": ICON_MOTION_SENSOR},
    "on_latency": _DURATION,
    "off_latency": _DURATION,
    "inhibit_time": _DURATION,
    "speed_threshold_factor": {"icon": ICON_MOTION_SENSOR},
    "uart_report_period": {"icon": ICON_TIMELAPSE, "unit_of_measurement": UNIT_SECOND},
}

DEFAULT_NAMES = {
    "min_range": "Min range",
    "max_range": "Max range",
    "trigger_range": "Trigger range",
    "sensitivity": "Sensitivity",
    "hold_sensitivity": "Hold sensitivity",
    "trigger_sensitivity": "Trigger sensitivity",
    "on_latency": "On latency",
    "off_latency": "Off latency",
    "inhibit_time": "Inhibit time",
    "speed_threshold_factor": "Speed mode threshold factor",
    "uart_report_period": "UART report period",
}


def _number_schema(key: str, opts: dict) -> cv.Schema:
    return with_default_name(
        number.number_schema(
            ParamNumber, entity_category=ENTITY_CATEGORY_CONFIG, **opts
        ),
        DEFAULT_NAMES[key],
    )


CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_DFROBOT_MMWAVE_ID): cv.use_id(DfrobotMmwave),
        **{
            cv.Optional(key): _number_schema(key, opts) for key, opts in NUMBERS.items()
        },
    }
)

FINAL_VALIDATE_SCHEMA = validate_entities_for_model(
    {key: PARAMETERS[key][1] for key in NUMBERS}
)


async def to_code(config: ConfigType) -> None:
    hub = await cg.get_variable(config[CONF_DFROBOT_MMWAVE_ID])
    model = hub_model(config[CONF_DFROBOT_MMWAVE_ID])
    for key in NUMBERS:
        if (conf := config.get(key)) is None:
            continue
        param_id = PARAMETERS[key][0]
        lo, hi, step = number_limits(model, key)
        if conf[CONF_MODE] == "AUTO":
            conf = {
                **conf,
                CONF_MODE: cv.enum(number.NUMBER_MODES, upper=True)(
                    number_mode(model, key)
                ),
            }
        n = cg.new_Pvariable(conf[CONF_ID], param_id)
        await number.register_number(n, conf, min_value=lo, max_value=hi, step=step)
        await cg.register_parented(n, config[CONF_DFROBOT_MMWAVE_ID])
        cg.add(hub.set_param_number(param_id, n))
