"""Select entities for the enumerated radar settings."""

import esphome.codegen as cg
from esphome.components import select
import esphome.config_validation as cv
from esphome.const import CONF_ID, ENTITY_CATEGORY_CONFIG
from esphome.types import ConfigType

from .. import (
    CONF_DFROBOT_MMWAVE_ID,
    PARAMETERS,
    DfrobotMmwave,
    dfrobot_mmwave_ns,
    validate_entities_for_model,
    with_default_name,
)

DEPENDENCIES = ["dfrobot_mmwave"]

ParamSelect = dfrobot_mmwave_ns.class_("ParamSelect", select.Select)

# key -> (options, icon, default name); option index == value sent to / read from the radar
SELECTS: dict[str, tuple[list[str], str, str]] = {
    "work_mode": (["presence", "speed_and_distance"], "mdi:radar", "Work mode"),
}

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_DFROBOT_MMWAVE_ID): cv.use_id(DfrobotMmwave),
        **{
            cv.Optional(key): with_default_name(
                select.select_schema(
                    ParamSelect, entity_category=ENTITY_CATEGORY_CONFIG, icon=icon
                ),
                name,
            )
            for key, (_, icon, name) in SELECTS.items()
        },
    }
)

FINAL_VALIDATE_SCHEMA = validate_entities_for_model(
    {key: PARAMETERS[key][1] for key in SELECTS}
)


async def to_code(config: ConfigType) -> None:
    hub = await cg.get_variable(config[CONF_DFROBOT_MMWAVE_ID])
    for key, (options, _, _) in SELECTS.items():
        if (conf := config.get(key)) is None:
            continue
        param_id = PARAMETERS[key][0]
        s = cg.new_Pvariable(conf[CONF_ID], param_id)
        await select.register_select(s, conf, options=options)
        await cg.register_parented(s, config[CONF_DFROBOT_MMWAVE_ID])
        cg.add(hub.set_param_select(param_id, s))
