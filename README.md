# ESPHome component for Waveshare ST3215 / ST3235 serial-bus servos

A custom [ESPHome](https://esphome.io) external component that drives **Feetech STS-series** serial-bus servos (Waveshare **ST3215** / **ST3235**) and exposes every sensor and control the servo offers to Home Assistant.

It's a clean-room reimplementation of the Feetech `SMS_STS` protocol directly on ESPHome's UART — **no Arduino `SCServo` library dependency**. It runs on the [Waveshare "Servo Driver with ESP32"](https://www.waveshare.com/wiki/Servo_Driver_with_ESP32) board (ESP32-WROOM-32 + CP2102), where the servo bus is on `GPIO18`/`GPIO19` at 1 Mbps.

## Features

Per servo ID on the bus, the component exposes:

| Domain | Entities |
| --- | --- |
| **sensor** | position (steps), position_deg (°), speed, load (%), current (mA), voltage (V), temperature (°C) |
| **binary_sensor** | moving |
| **number** | target position, goal speed, acceleration, torque limit |
| **switch** | torque enable |
| **select** | operating mode (Position / Wheel / PWM / Step) |
| **button** | calibrate center |

Multiple servos can be daisy-chained on one bus — just add more platform blocks with different `servo_id`s.

## Hardware

- **Board:** Waveshare Servo Driver with ESP32 (ESP32-WROOM-32, CP2102 USB-UART)
- **Servo bus:** `TX=GPIO19`, `RX=GPIO18`, **1,000,000 baud, 8N1**, little-endian (STS `End=0`)
- **Servos:** ST3215, ST3235, and other Feetech STS-series serial-bus servos (default servo ID `1`)

## Installation

Reference the component straight from GitHub:

```yaml
external_components:
  - source: github://sweitzja/esphome-st3215-servo
    components: [st3215]
```

Then set up the bus and the entities:

```yaml
uart:
  id: servo_uart
  tx_pin: GPIO19
  rx_pin: GPIO18
  baud_rate: 1000000

st3215:
  - id: servo_bus
    uart_id: servo_uart
    update_interval: 500ms

sensor:
  - platform: st3215
    st3215_id: servo_bus
    servo_id: 1
    position:    { name: "Servo Position" }
    current:     { name: "Servo Current" }
    voltage:     { name: "Servo Voltage" }
    temperature: { name: "Servo Temperature" }

number:
  - platform: st3215
    st3215_id: servo_bus
    servo_id: 1
    target_position: { name: "Servo Target Position" }

switch:
  - platform: st3215
    st3215_id: servo_bus
    servo_id: 1
    torque: { name: "Servo Torque" }
```

See [`servo-driver.yaml`](servo-driver.yaml) for a complete config including WiFi, OTA, web server, and a **valve abstraction** built on top (see below).

Copy [`secrets.yaml.example`](secrets.yaml.example) to `secrets.yaml` and fill in your WiFi credentials (`secrets.yaml` is gitignored).

## Valve abstraction (example)

`servo-driver.yaml` layers a valve controller on top of the raw servo using standard ESPHome template entities:

- **Valve On/Off Angle** — calibrate the open/closed servo angles
- **Valve** (0–100 %) — commands the valve between those angles (enables torque and moves)
- **Valve Actual** (0–100 %) — actual position feedback
- **Valve Stalled** — `problem` binary sensor that trips when the servo is jammed mid-travel (torque on, not at target, not moving, still pushing)

This lives in YAML (not the component) so the component stays generic.

## Protocol notes

Key STS registers used (see [`components/st3215/st3215.h`](components/st3215/st3215.h)):

- Mode `33`, Torque Enable `40`, goal block at `41` (ACC, pos, time, speed)
- Torque Limit `48`, EEPROM lock `55`
- Feedback block `56`–`70` (position, speed, load, voltage, temperature, moving, current)
- Current ≈ raw × 6.5 mA; voltage = raw ÷ 10; load sign-bit is bit 10; position/speed/current sign-bit is bit 15

## Credits

Protocol reverse-engineered from Waveshare's official [Servo-Driver-with-ESP32](https://github.com/waveshare/Servo-Driver-with-ESP32) `SCServo` library.

## License

[MIT](LICENSE)
