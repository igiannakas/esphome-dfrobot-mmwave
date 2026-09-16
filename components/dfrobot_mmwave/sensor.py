"""Target sensors of the dfrobot_mmwave hub."""

import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import (
    DEVICE_CLASS_DISTANCE,
    DEVICE_CLASS_SPEED,
    STATE_CLASS_MEASUREMENT,
    UNIT_METER,
    UNIT_METER_PER_SECOND,
)
from esphome.types import ConfigType

from . import (
    ALL_MODELS,
    C4001,
    CONF_DFROBOT_MMWAVE_ID,
    SEN0395,
    DfrobotMmwave,
    validate_entities_for_model,
    with_default_name,
)

DEPENDENCIES = ["dfrobot_mmwave"]

MAX_TARGETS = 8
CONF_TARGET_COUNT = "target_count"
CONF_TARGET_1_SPEED = "target_1_speed"
CONF_TARGET_1_ENERGY = "target_1_energy"


def _distance_key(slot: int) -> str:
    return f"target_{slot}_distance"


def _snr_key(slot: int) -> str:
    return f"target_{slot}_snr"


# slot 1 = the radar's own target index i=1 ($JYRPO) / the single C4001 target ($DFDMD)
AVAILABILITY: dict[str, tuple[str, ...]] = {
    CONF_TARGET_COUNT: ALL_MODELS,
    _distance_key(1): ALL_MODELS,
    CONF_TARGET_1_SPEED: C4001,
    CONF_TARGET_1_ENERGY: C4001,
}
for _slot in range(1, MAX_TARGETS + 1):
    AVAILABILITY[_snr_key(_slot)] = (SEN0395,)
    if _slot > 1:
        AVAILABILITY[_distance_key(_slot)] = (SEN0395,)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(CONF_DFROBOT_MMWAVE_ID): cv.use_id(DfrobotMmwave),
            cv.Optional(CONF_TARGET_COUNT): with_default_name(
                sensor.sensor_schema(
                    accuracy_decimals=0,
                    state_class=STATE_CLASS_MEASUREMENT,
                    icon="mdi:account-group",
                ),
                "Targets",
            ),
            cv.Optional(CONF_TARGET_1_SPEED): with_default_name(
                sensor.sensor_schema(
                    unit_of_measurement=UNIT_METER_PER_SECOND,
                    device_class=DEVICE_CLASS_SPEED,
                    state_class=STATE_CLASS_MEASUREMENT,
                    accuracy_decimals=3,
                ),
                "Target 1 speed",
            ),
            # dimensionless radar energy: deliberately no device class / unit
            cv.Optional(CONF_TARGET_1_ENERGY): with_default_name(
                sensor.sensor_schema(
                    accuracy_decimals=0,
                    state_class=STATE_CLASS_MEASUREMENT,
                    icon="mdi:signal",
                ),
                "Target 1 energy",
            ),
        }
    )
    .extend(
        {
            cv.Optional(_distance_key(slot)): with_default_name(
                sensor.sensor_schema(
                    unit_of_measurement=UNIT_METER,
                    device_class=DEVICE_CLASS_DISTANCE,
                    state_class=STATE_CLASS_MEASUREMENT,
                    accuracy_decimals=3,
                ),
                f"Target {slot} distance",
            )
            for slot in range(1, MAX_TARGETS + 1)
        }
    )
    .extend(
        {
            cv.Optional(_snr_key(slot)): with_default_name(
                sensor.sensor_schema(
                    accuracy_decimals=3,
                    state_class=STATE_CLASS_MEASUREMENT,
                    icon="mdi:signal-variant",
                ),
                f"Target {slot} SNR",
            )
            for slot in range(1, MAX_TARGETS + 1)
        }
    )
)

FINAL_VALIDATE_SCHEMA = validate_entities_for_model(AVAILABILITY)


async def to_code(config: ConfigType) -> None:
    hub = await cg.get_variable(config[CONF_DFROBOT_MMWAVE_ID])
    for key in (CONF_TARGET_COUNT, CONF_TARGET_1_SPEED, CONF_TARGET_1_ENERGY):
        if (conf := config.get(key)) is not None:
            sens = await sensor.new_sensor(conf)
            cg.add(getattr(hub, f"set_{key}_sensor")(sens))
    for slot in range(1, MAX_TARGETS + 1):
        if (conf := config.get(_distance_key(slot))) is not None:
            sens = await sensor.new_sensor(conf)
            cg.add(hub.set_target_distance_sensor(slot - 1, sens))
        if (conf := config.get(_snr_key(slot))) is not None:
            sens = await sensor.new_sensor(conf)
            cg.add(hub.set_target_snr_sensor(slot - 1, sens))
