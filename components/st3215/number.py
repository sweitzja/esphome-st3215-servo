import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import number

from . import CONF_ST3215_ID, ST3215_CHILD_SCHEMA, register_child, st3215_ns

DEPENDENCIES = ["st3215"]

NumberField = st3215_ns.enum("NumberField")
ST3215Number = st3215_ns.class_("ST3215Number", number.Number, cg.Parented)

# key -> (C++ enum value, min, max, step, unit, icon)
NUMBER_TYPES = {
    "target_position": (NumberField.NUM_TARGET_POSITION, 0, 4095, 1, "steps", "mdi:target"),
    "goal_speed": (NumberField.NUM_GOAL_SPEED, 0, 4000, 1, "steps/s", "mdi:speedometer"),
    "acceleration": (NumberField.NUM_ACCELERATION, 0, 255, 1, "", "mdi:rocket-launch"),
    "torque_limit": (NumberField.NUM_TORQUE_LIMIT, 0, 1000, 1, "", "mdi:arm-flex"),
}

CONFIG_SCHEMA = ST3215_CHILD_SCHEMA.extend(
    {
        cv.Optional(key): number.number_schema(ST3215Number, unit_of_measurement=unit, icon=icon)
        for key, (_, _, _, _, unit, icon) in NUMBER_TYPES.items()
    }
)


async def to_code(config):
    parent, servo_id = await register_child(config)
    for key, (field, lo, hi, step, _, _) in NUMBER_TYPES.items():
        if key in config:
            var = await number.new_number(
                config[key], min_value=lo, max_value=hi, step=step
            )
            await cg.register_parented(var, config[CONF_ST3215_ID])
            cg.add(var.set_servo_id(servo_id))
            cg.add(var.set_field(field))
            cg.add(parent.register_number(servo_id, field, var))
