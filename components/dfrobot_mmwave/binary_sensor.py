"""Binary sensors of the dfrobot_mmwave hub: occupancy sources and diagnostics."""

import esphome.codegen as cg
from esphome.components import binary_sensor
import esphome.config_validation as cv
from esphome.const import (
    DEVICE_CLASS_CONNECTIVITY,
    DEVICE_CLASS_OCCUPANCY,
    ENTITY_CATEGORY_DIAGNOSTIC,
)
from esphome.types import ConfigType

from . import (
    ALL_MODELS,
    CONF_DFROBOT_MMWAVE_ID,
    CONF_PRESENCE_PIN,
    WITH_OUT_PIN,
    DfrobotMmwave,
    hub_has_presence_pin,
    validate_entities_for_model,
    with_default_name,
)

DEPENDENCIES = ["dfrobot_mmwave"]

CONF_OCCUPANCY = "occupancy"
CONF_UART_OCCUPANCY = "uart_occupancy"
CONF_OUT_PIN_OCCUPANCY = "out_pin_occupancy"
CONF_LINK_OK = "link_ok"

AVAILABILITY = {
    CONF_OCCUPANCY: ALL_MODELS,
    CONF_UART_OCCUPANCY: ALL_MODELS,
    CONF_OUT_PIN_OCCUPANCY: WITH_OUT_PIN,
    CONF_LINK_OK: ALL_MODELS,
}

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_DFROBOT_MMWAVE_ID): cv.use_id(DfrobotMmwave),
        cv.Optional(CONF_OCCUPANCY): with_default_name(
            binary_sensor.binary_sensor_schema(device_class=DEVICE_CLASS_OCCUPANCY),
            "Occupancy",
        ),
        cv.Optional(CONF_UART_OCCUPANCY): with_default_name(
            binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_OCCUPANCY,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            "Occupancy (UART)",
        ),
        cv.Optional(CONF_OUT_PIN_OCCUPANCY): with_default_name(
            binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_OCCUPANCY,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            "Occupancy (OUT pin)",
        ),
        cv.Optional(CONF_LINK_OK): with_default_name(
            binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_CONNECTIVITY,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            "Radar link",
        ),
    }
)


def _final_validate(config: ConfigType) -> ConfigType:
    validate_entities_for_model(AVAILABILITY)(config)
    if CONF_OUT_PIN_OCCUPANCY in config and not hub_has_presence_pin(
        config[CONF_DFROBOT_MMWAVE_ID]
    ):
        raise cv.Invalid(
            f"'{CONF_OUT_PIN_OCCUPANCY}' requires '{CONF_PRESENCE_PIN}' on the "
            "dfrobot_mmwave hub",
            path=[CONF_OUT_PIN_OCCUPANCY],
        )
    return config


FINAL_VALIDATE_SCHEMA = _final_validate


async def to_code(config: ConfigType) -> None:
    hub = await cg.get_variable(config[CONF_DFROBOT_MMWAVE_ID])
    for key in AVAILABILITY:
        if (conf := config.get(key)) is not None:
            sens = await binary_sensor.new_binary_sensor(conf)
            cg.add(getattr(hub, f"set_{key}_binary_sensor")(sens))
