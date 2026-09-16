"""Text sensors of the dfrobot_mmwave hub: versions, status and last error."""

import esphome.codegen as cg
from esphome.components import text_sensor
import esphome.config_validation as cv
from esphome.const import CONF_STATUS, ENTITY_CATEGORY_DIAGNOSTIC, ICON_CHIP
from esphome.types import ConfigType

from . import CONF_DFROBOT_MMWAVE_ID, DfrobotMmwave, with_default_name

DEPENDENCIES = ["dfrobot_mmwave"]

CONF_SOFTWARE_VERSION = "software_version"
CONF_HARDWARE_VERSION = "hardware_version"
CONF_LAST_ERROR = "last_error"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_DFROBOT_MMWAVE_ID): cv.use_id(DfrobotMmwave),
        cv.Optional(CONF_SOFTWARE_VERSION): with_default_name(
            text_sensor.text_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC, icon=ICON_CHIP
            ),
            "Radar firmware",
        ),
        cv.Optional(CONF_HARDWARE_VERSION): with_default_name(
            text_sensor.text_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC, icon=ICON_CHIP
            ),
            "Radar hardware",
        ),
        cv.Optional(CONF_STATUS): with_default_name(
            text_sensor.text_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC, icon="mdi:radar"
            ),
            "Radar status",
        ),
        cv.Optional(CONF_LAST_ERROR): with_default_name(
            text_sensor.text_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                icon="mdi:alert-circle-outline",
            ),
            "Radar last error",
        ),
    }
)


async def to_code(config: ConfigType) -> None:
    hub = await cg.get_variable(config[CONF_DFROBOT_MMWAVE_ID])
    for key in (
        CONF_SOFTWARE_VERSION,
        CONF_HARDWARE_VERSION,
        CONF_STATUS,
        CONF_LAST_ERROR,
    ):
        if (conf := config.get(key)) is not None:
            sens = await text_sensor.new_text_sensor(conf)
            cg.add(getattr(hub, f"set_{key}_text_sensor")(sens))
