import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.const import DEVICE_CLASS_MOVING

from . import ST3215_CHILD_SCHEMA, register_child

DEPENDENCIES = ["st3215"]

CONF_MOVING = "moving"

CONFIG_SCHEMA = ST3215_CHILD_SCHEMA.extend(
    {
        cv.Optional(CONF_MOVING): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_MOVING,
            icon="mdi:motion-outline",
        ),
    }
)


async def to_code(config):
    parent, servo_id = await register_child(config)
    if CONF_MOVING in config:
        var = await binary_sensor.new_binary_sensor(config[CONF_MOVING])
        cg.add(parent.register_moving(servo_id, var))
