import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import select

from . import CONF_ST3215_ID, ST3215_CHILD_SCHEMA, register_child, st3215_ns

DEPENDENCIES = ["st3215"]

ST3215ModeSelect = st3215_ns.class_("ST3215ModeSelect", select.Select, cg.Parented)

CONF_MODE = "mode"

# Order MUST match MODE_OPTIONS[] in st3215.h and the STS mode register values.
MODE_OPTIONS = ["Position", "Wheel", "PWM", "Step"]

CONFIG_SCHEMA = ST3215_CHILD_SCHEMA.extend(
    {
        cv.Optional(CONF_MODE): select.select_schema(
            ST3215ModeSelect,
            icon="mdi:cog-transfer",
        ),
    }
)


async def to_code(config):
    parent, servo_id = await register_child(config)
    if CONF_MODE in config:
        var = await select.new_select(config[CONF_MODE], options=MODE_OPTIONS)
        await cg.register_parented(var, config[CONF_ST3215_ID])
        cg.add(var.set_servo_id(servo_id))
        cg.add(parent.register_mode_select(servo_id, var))
