import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch

from . import CONF_ST3215_ID, ST3215_CHILD_SCHEMA, register_child, st3215_ns

DEPENDENCIES = ["st3215"]

ST3215TorqueSwitch = st3215_ns.class_(
    "ST3215TorqueSwitch", switch.Switch, cg.Parented
)

CONF_TORQUE = "torque"

CONFIG_SCHEMA = ST3215_CHILD_SCHEMA.extend(
    {
        cv.Optional(CONF_TORQUE): switch.switch_schema(
            ST3215TorqueSwitch,
            icon="mdi:arm-flex",
            default_restore_mode="DISABLED",
        ),
    }
)


async def to_code(config):
    parent, servo_id = await register_child(config)
    if CONF_TORQUE in config:
        var = await switch.new_switch(config[CONF_TORQUE])
        await cg.register_parented(var, config[CONF_ST3215_ID])
        cg.add(var.set_servo_id(servo_id))
        cg.add(parent.register_torque_switch(servo_id, var))
