"""ST3215 / ST3235 Feetech STS serial-bus servo component for ESPHome.

Reverse-engineered from Waveshare's official SCServo library (SMS_STS protocol):
  - Bus: half-duplex TTL serial, 1,000,000 baud, 8N1
  - On the Waveshare "Servo Driver with ESP32": S_RXD=GPIO18, S_TXD=GPIO19
  - Little-endian on the wire (STS series, End=0)
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID

CODEOWNERS = ["@sweitzja"]
DEPENDENCIES = ["uart"]
AUTO_LOAD = ["sensor", "binary_sensor", "number", "switch", "select", "button"]
MULTI_CONF = True

st3215_ns = cg.esphome_ns.namespace("st3215")
ST3215Bus = st3215_ns.class_("ST3215Bus", cg.PollingComponent, uart.UARTDevice)

CONF_ST3215_ID = "st3215_id"
CONF_SERVO_ID = "servo_id"

CONFIG_SCHEMA = (
    cv.Schema({cv.GenerateID(): cv.declare_id(ST3215Bus)})
    .extend(cv.polling_component_schema("1s"))
    .extend(uart.UART_DEVICE_SCHEMA)
)

# Shared schema fragment for every child platform (sensor/number/switch/...).
ST3215_CHILD_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ST3215_ID): cv.use_id(ST3215Bus),
        cv.Required(CONF_SERVO_ID): cv.int_range(min=0, max=253),
    }
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)


async def register_child(config):
    """Resolve the parent bus var + servo_id for a child platform config."""
    parent = await cg.get_variable(config[CONF_ST3215_ID])
    return parent, config[CONF_SERVO_ID]
