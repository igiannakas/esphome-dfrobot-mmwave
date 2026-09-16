"""DFRobot mmWave radars SEN0395 / SEN0609 / SEN0610 (UART CLI, optional presence pin)."""

from collections.abc import Callable

from esphome import automation, pins
from esphome.automation import maybe_simple_id
import esphome.codegen as cg
from esphome.components import uart
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    CONF_INPUT,
    CONF_MODEL,
    CONF_NAME,
    CONF_PULLDOWN,
    CONF_VALUE,
)
from esphome.core import CORE, ID
from esphome.cpp_generator import MockObj, TemplateArgsType
import esphome.final_validate as fv
from esphome.types import ConfigType

CODEOWNERS = ["@igiannakas"]
DEPENDENCIES = ["uart"]
MULTI_CONF = True

DOMAIN = "dfrobot_mmwave"

dfrobot_mmwave_ns = cg.esphome_ns.namespace("dfrobot_mmwave")
protocol_ns = dfrobot_mmwave_ns.namespace("protocol")
DfrobotMmwave = dfrobot_mmwave_ns.class_("DfrobotMmwave", cg.Component, uart.UARTDevice)
Model = protocol_ns.enum("Model", is_class=True)
ParamId = protocol_ns.enum("ParamId", is_class=True)

CONF_DFROBOT_MMWAVE_ID = "dfrobot_mmwave_id"
CONF_PRESENCE_PIN = "presence_pin"
CONF_TARGET_TIMEOUT = "target_timeout"
CONF_PARAMETER = "parameter"

SEN0395 = "SEN0395"
SEN0609 = "SEN0609"
SEN0610 = "SEN0610"
ALL_MODELS = (SEN0395, SEN0609, SEN0610)
C4001 = (SEN0609, SEN0610)
WITH_OUT_PIN = (SEN0395, SEN0609)

MODELS = {
    SEN0395: Model.MODEL_SEN0395,
    SEN0609: Model.MODEL_SEN0609,
    SEN0610: Model.MODEL_SEN0610,
}
# The component never changes the radar's baud rate, so the UART must use the factory default.
DEFAULT_BAUD = {SEN0395: 115200, SEN0609: 9600, SEN0610: 9600}
# (min, max, step) of every number setting, per model. Must match the Dialect table in mmwave_dialect.h
# (host_tests/check_limits_sync.py compares the two). C4001 values are the ones the firmware keeps.
NUMBER_LIMITS = {
    "SEN0395": {
        "min_range": (0.0, 9.3, 0.15),
        "max_range": (0.15, 9.45, 0.15),
        "sensitivity": (0.0, 9.0, 1.0),
        "on_latency": (0.0, 100.0, 0.025),
        "off_latency": (0.5, 1500.0, 0.1),
        "uart_report_period": (0.025, 1500.0, 0.025),
    },
    "SEN0609": {
        "min_range": (0.3, 25.9, 0.1),
        "max_range": (2.4, 26.0, 0.1),
        "trigger_range": (2.4, 25.0, 0.1),
        "hold_sensitivity": (0.0, 9.0, 1.0),
        "trigger_sensitivity": (0.0, 9.0, 1.0),
        "on_latency": (0.0, 2.0, 0.01),
        "off_latency": (2.0, 1500.0, 0.5),
        "inhibit_time": (0.3, 60.0, 0.1),
        "speed_threshold_factor": (0.0, 65535.0, 1.0),
        "uart_report_period": (0.2, 1500.0, 0.025),
    },
    "SEN0610": {
        "min_range": (0.3, 11.9, 0.1),
        "max_range": (1.2, 12.0, 0.1),
        "trigger_range": (1.2, 12.0, 0.1),
        "hold_sensitivity": (0.0, 9.0, 1.0),
        "trigger_sensitivity": (0.0, 9.0, 1.0),
        "on_latency": (0.0, 2.0, 0.01),
        "off_latency": (2.0, 1500.0, 0.5),
        "inhibit_time": (0.3, 60.0, 0.1),
        "speed_threshold_factor": (0.0, 65535.0, 1.0),
        "uart_report_period": (0.2, 1500.0, 0.025),
    },
}
# Number entities shown as a text box instead of a slider (wide ranges); the entity's `mode:` overrides it.
BOX_NUMBERS = {
    SEN0395: ("on_latency", "off_latency", "uart_report_period"),
    SEN0609: ("off_latency", "uart_report_period", "speed_threshold_factor"),
    SEN0610: ("off_latency", "uart_report_period", "speed_threshold_factor"),
}
# Settings that are selects or switches: value range of the option index / on-off state.
INDEX_RANGES = {
    "work_mode": (0, 1),
    "led": (0, 1),
    "speed_micro_motion": (0, 1),
    "uart_presence_report": (0, 1),
    "uart_target_report": (0, 1),
}

# Settings that map to one radar parameter: YAML key -> (ParamId, models).
PARAMETERS: dict[str, tuple[MockObj, tuple[str, ...]]] = {
    "min_range": (ParamId.PARAM_ID_RANGE_MIN, ALL_MODELS),
    "max_range": (ParamId.PARAM_ID_RANGE_MAX, ALL_MODELS),
    "trigger_range": (ParamId.PARAM_ID_TRIG_RANGE, C4001),
    "sensitivity": (ParamId.PARAM_ID_SENSITIVITY, (SEN0395,)),
    "hold_sensitivity": (ParamId.PARAM_ID_HOLD_SENS, C4001),
    "trigger_sensitivity": (ParamId.PARAM_ID_TRIG_SENS, C4001),
    "on_latency": (ParamId.PARAM_ID_LATENCY_ON, ALL_MODELS),
    "off_latency": (ParamId.PARAM_ID_LATENCY_OFF, ALL_MODELS),
    "inhibit_time": (ParamId.PARAM_ID_INHIBIT, C4001),
    "speed_micro_motion": (ParamId.PARAM_ID_MICRO_MOTION, C4001),
    "speed_threshold_factor": (ParamId.PARAM_ID_THR_FACTOR, C4001),
    "led": (ParamId.PARAM_ID_LED, ALL_MODELS),
    "uart_presence_report": (ParamId.PARAM_ID_UART_PRESENCE_EN, ALL_MODELS),
    "uart_target_report": (ParamId.PARAM_ID_UART_TARGET_EN, (SEN0395,)),
    "uart_report_period": (ParamId.PARAM_ID_UART_REPORT_PERIOD, ALL_MODELS),
    "work_mode": (ParamId.PARAM_ID_WORK_MODE, C4001),
}


def number_limits(model: str, key: str) -> tuple[float, float, float]:
    """(min, max, step) of a number setting on this model."""
    return NUMBER_LIMITS[model][key]


def number_mode(model: str, key: str) -> str:
    """Default entity mode (SLIDER or BOX) of a number setting on this model."""
    return "BOX" if key in BOX_NUMBERS[model] else "SLIDER"


def parameter_range(model: str, key: str) -> tuple[float, float]:
    """Accepted (min, max) for a set_parameter value."""
    if key in INDEX_RANGES:
        return INDEX_RANGES[key]
    lo, hi, _ = number_limits(model, key)
    return (lo, hi)


def _hub_configs() -> list[ConfigType]:
    # final validation sees fv.full_config; code generation reads CORE.config
    for source in (fv.full_config.get(None), CORE.config):
        if source is not None and DOMAIN in source:
            confs = source[DOMAIN]
            return confs if isinstance(confs, list) else [confs]
    return []


def hub_model(hub_id: ID) -> str | None:
    """Model of the hub with this id; works during final validation and code generation."""
    for conf in _hub_configs():
        if str(conf[CONF_ID]) == str(hub_id):
            return conf[CONF_MODEL]
    return None


def with_default_name(schema: cv.Schema, name: str) -> cv.Schema:
    """Entity schema whose `name` defaults to `name`, so an empty entry (`key: {}`) is enough."""
    return schema.extend({cv.Optional(CONF_NAME, default=name): cv.string})


def hub_has_presence_pin(hub_id: ID) -> bool:
    for conf in _hub_configs():
        if str(conf[CONF_ID]) == str(hub_id):
            return CONF_PRESENCE_PIN in conf
    return False


def validate_entities_for_model(
    availability: dict[str, tuple[str, ...]],
) -> Callable[[ConfigType], ConfigType]:
    """Final-validate hook for entity platforms: reject keys the configured model lacks."""

    def validator(config: ConfigType) -> ConfigType:
        hub_id = config[CONF_DFROBOT_MMWAVE_ID]
        model = hub_model(hub_id)
        if model is None:
            return config
        for key, models in availability.items():
            if key in config and model not in models:
                raise cv.Invalid(f"'{key}' is not available on the {model}", path=[key])
        return config

    return validator


def _validate_hub(config: ConfigType) -> ConfigType:
    model = config[CONF_MODEL]
    if CONF_PRESENCE_PIN in config and model not in WITH_OUT_PIN:
        raise cv.Invalid(
            f"the {model} has no presence (OUT) pin; remove '{CONF_PRESENCE_PIN}'",
            path=[CONF_PRESENCE_PIN],
        )
    return config


def presence_pin_schema(value: object) -> ConfigType:
    """Input pin; defaults to a pull-down so an unconnected OUT wire reads as no presence."""
    if CORE.is_esp8266:
        # ESP8266: only GPIO16 has a pull-down
        return pins.gpio_input_pin_schema(value)
    return pins.gpio_pin_schema({CONF_INPUT: True, CONF_PULLDOWN: True})(value)


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(DfrobotMmwave),
            cv.Required(CONF_MODEL): cv.one_of(*ALL_MODELS, upper=True),
            cv.Optional(CONF_PRESENCE_PIN): presence_pin_schema,
            cv.Optional(CONF_TARGET_TIMEOUT, default="2s"): cv.All(
                cv.positive_time_period_milliseconds,
                cv.Range(min=cv.TimePeriod(milliseconds=100)),
            ),
        }
    )
    .extend(uart.UART_DEVICE_SCHEMA)
    .extend(cv.COMPONENT_SCHEMA),
    _validate_hub,
)


def _set_parameter_actions(node: object):
    """Yield every validated dfrobot_mmwave.set_parameter action config found in the full config."""
    if isinstance(node, dict):
        for key, value in node.items():
            if key == f"{DOMAIN}.set_parameter" and isinstance(value, dict):
                yield value
            else:
                yield from _set_parameter_actions(value)
    elif isinstance(node, list):
        for item in node:
            yield from _set_parameter_actions(item)


def _validate_set_parameter_actions(config: ConfigType) -> None:
    """Reject set_parameter actions naming a setting the model lacks or a constant out of range."""
    model = config[CONF_MODEL]
    for action in _set_parameter_actions(fv.full_config.get()):
        if str(action.get(CONF_ID)) != str(config[CONF_ID]):
            continue
        key = action[CONF_PARAMETER]
        if model not in PARAMETERS[key][1]:
            raise cv.Invalid(
                f"{DOMAIN}.set_parameter: '{key}' is not available on the {model}"
            )
        value = action[CONF_VALUE]
        if isinstance(value, cv.Lambda):
            continue  # checked by the component at run time
        lo, hi = parameter_range(model, key)
        if not lo <= value <= hi:
            raise cv.Invalid(
                f"{DOMAIN}.set_parameter: {key} must be between {lo:g} and {hi:g} "
                f"on the {model}, got {value:g}"
            )


def _final_validate(config: ConfigType) -> ConfigType:
    model = config[CONF_MODEL]
    _validate_set_parameter_actions(config)
    return uart.final_validate_device_schema(
        f"{DOMAIN} ({model})",
        baud_rate=DEFAULT_BAUD[model],
        require_tx=True,
        require_rx=True,
        parity="NONE",
        stop_bits=1,
    )(config)


FINAL_VALIDATE_SCHEMA = _final_validate


async def to_code(config: ConfigType) -> None:
    var = cg.new_Pvariable(config[CONF_ID], MODELS[config[CONF_MODEL]])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
    if (pin_config := config.get(CONF_PRESENCE_PIN)) is not None:
        pin = await cg.gpio_pin_expression(pin_config)
        cg.add(var.set_presence_pin(pin))
    cg.add(var.set_target_timeout(config[CONF_TARGET_TIMEOUT]))


# Actions

RefreshAction = dfrobot_mmwave_ns.class_("RefreshAction", automation.Action)
RestartAction = dfrobot_mmwave_ns.class_("RestartAction", automation.Action)
FactoryResetAction = dfrobot_mmwave_ns.class_("FactoryResetAction", automation.Action)
SetParameterAction = dfrobot_mmwave_ns.class_("SetParameterAction", automation.Action)

SIMPLE_ACTION_SCHEMA = maybe_simple_id(
    {
        cv.GenerateID(): cv.use_id(DfrobotMmwave),
    }
)


async def _simple_action_to_code(
    config: ConfigType,
    action_id: ID,
    template_arg: cg.TemplateArguments,
    args: TemplateArgsType,
) -> MockObj:
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    return var


for _name, _cls in (
    ("refresh", RefreshAction),
    ("restart", RestartAction),
    ("factory_reset", FactoryResetAction),
):
    automation.register_action(
        f"{DOMAIN}.{_name}", _cls, SIMPLE_ACTION_SCHEMA, synchronous=True
    )(_simple_action_to_code)


SET_PARAMETER_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.use_id(DfrobotMmwave),
        cv.Required(CONF_PARAMETER): cv.one_of(*PARAMETERS, lower=True),
        cv.Required(CONF_VALUE): cv.templatable(cv.float_),
    }
)


@automation.register_action(
    f"{DOMAIN}.set_parameter",
    SetParameterAction,
    SET_PARAMETER_SCHEMA,
    synchronous=True,
)
async def set_parameter_to_code(
    config: ConfigType,
    action_id: ID,
    template_arg: cg.TemplateArguments,
    args: TemplateArgsType,
) -> MockObj:
    param_id, _ = PARAMETERS[config[CONF_PARAMETER]]
    var = cg.new_Pvariable(action_id, template_arg, param_id)
    await cg.register_parented(var, config[CONF_ID])
    template_ = await cg.templatable(config[CONF_VALUE], args, cg.float_)
    cg.add(var.set_value(template_))
    return var
