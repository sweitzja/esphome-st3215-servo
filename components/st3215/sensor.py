import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_VOLTAGE,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
    UNIT_PERCENT,
    UNIT_VOLT,
)

from . import ST3215_CHILD_SCHEMA, register_child, st3215_ns

DEPENDENCIES = ["st3215"]

SensorField = st3215_ns.enum("SensorField")

# key -> (C++ enum value, sensor schema)
SENSOR_TYPES = {
    "position": (
        SensorField.FIELD_POSITION,
        sensor.sensor_schema(
            unit_of_measurement="steps",
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
            icon="mdi:rotate-right",
        ),
    ),
    "speed": (
        SensorField.FIELD_SPEED,
        sensor.sensor_schema(
            unit_of_measurement="steps/s",
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
            icon="mdi:speedometer",
        ),
    ),
    "load": (
        SensorField.FIELD_LOAD,
        sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT,
            accuracy_decimals=1,
            state_class=STATE_CLASS_MEASUREMENT,
            icon="mdi:gauge",
        ),
    ),
    "voltage": (
        SensorField.FIELD_VOLTAGE,
        sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_VOLTAGE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
    ),
    "temperature": (
        SensorField.FIELD_TEMPERATURE,
        sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
    ),
    "current": (
        SensorField.FIELD_CURRENT,
        sensor.sensor_schema(
            unit_of_measurement="mA",
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
            icon="mdi:current-dc",
        ),
    ),
}

CONFIG_SCHEMA = ST3215_CHILD_SCHEMA.extend(
    {cv.Optional(key): schema for key, (_, schema) in SENSOR_TYPES.items()}
)


async def to_code(config):
    parent, servo_id = await register_child(config)
    for key, (field, _) in SENSOR_TYPES.items():
        if key in config:
            var = await sensor.new_sensor(config[key])
            cg.add(parent.register_sensor(servo_id, field, var))
