"""Buttons that trigger one hub action."""

import esphome.codegen as cg
from esphome.components import button
import esphome.config_validation as cv
from esphome.const import (
    CONF_FACTORY_RESET,
    CONF_ID,
    CONF_RESTART,
    DEVICE_CLASS_RESTART,
    ENTITY_CATEGORY_CONFIG,
    ENTITY_CATEGORY_DIAGNOSTIC,
    ICON_DATABASE,
    ICON_RESTART,
    ICON_RESTART_ALERT,
)
from esphome.types import ConfigType

from .. import (
    CONF_DFROBOT_MMWAVE_ID,
    DfrobotMmwave,
    dfrobot_mmwave_ns,
    with_default_name,
)

DEPENDENCIES = ["dfrobot_mmwave"]

ActionButton = dfrobot_mmwave_ns.class_("ActionButton", button.Button)
ButtonAction = dfrobot_mmwave_ns.enum("ButtonAction", is_class=True)

CONF_REFRESH = "refresh"

BUTTONS = {
    CONF_REFRESH: ButtonAction.BUTTON_ACTION_REFRESH,
    CONF_RESTART: ButtonAction.BUTTON_ACTION_RESTART,
    CONF_FACTORY_RESET: ButtonAction.BUTTON_ACTION_FACTORY_RESET,
}

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_DFROBOT_MMWAVE_ID): cv.use_id(DfrobotMmwave),
        cv.Optional(CONF_REFRESH): with_default_name(
            button.button_schema(
                ActionButton,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                icon=ICON_DATABASE,
            ),
            "Reread radar settings",
        ),
        cv.Optional(CONF_RESTART): with_default_name(
            button.button_schema(
                ActionButton,
                device_class=DEVICE_CLASS_RESTART,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                icon=ICON_RESTART,
            ),
            "Restart radar",
        ),
        cv.Optional(CONF_FACTORY_RESET): with_default_name(
            button.button_schema(
                ActionButton,
                entity_category=ENTITY_CATEGORY_CONFIG,
                icon=ICON_RESTART_ALERT,
            ),
            "Factory reset radar",
        ),
    }
)


async def to_code(config: ConfigType) -> None:
    for key, action in BUTTONS.items():
        if (conf := config.get(key)) is None:
            continue
        b = cg.new_Pvariable(conf[CONF_ID], action)
        await button.register_button(b, conf)
        await cg.register_parented(b, config[CONF_DFROBOT_MMWAVE_ID])
