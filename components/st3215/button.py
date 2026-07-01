import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import button

from . import CONF_ST3215_ID, ST3215_CHILD_SCHEMA, register_child, st3215_ns

DEPENDENCIES = ["st3215"]

ST3215CalibrateButton = st3215_ns.class_(
    "ST3215CalibrateButton", button.Button, cg.Parented
)

CONF_CALIBRATE_CENTER = "calibrate_center"

CONFIG_SCHEMA = ST3215_CHILD_SCHEMA.extend(
    {
        cv.Optional(CONF_CALIBRATE_CENTER): button.button_schema(
            ST3215CalibrateButton,
            icon="mdi:image-filter-center-focus",
        ),
    }
)


async def to_code(config):
    parent, servo_id = await register_child(config)
    if CONF_CALIBRATE_CENTER in config:
        var = await button.new_button(config[CONF_CALIBRATE_CENTER])
        await cg.register_parented(var, config[CONF_ST3215_ID])
        cg.add(var.set_servo_id(servo_id))
