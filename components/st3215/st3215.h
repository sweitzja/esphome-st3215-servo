#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/number/number.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/select/select.h"
#include "esphome/components/button/button.h"

#include <vector>

namespace esphome {
namespace st3215 {

// ---- Feetech SMS/STS instruction set (INST.h) ----
static const uint8_t INST_PING = 0x01;
static const uint8_t INST_READ = 0x02;
static const uint8_t INST_WRITE = 0x03;

// ---- STS memory table (SMS_STS.h) ----
static const uint8_t STS_ID = 5;
static const uint8_t STS_MODE = 33;
static const uint8_t STS_TORQUE_ENABLE = 40;
static const uint8_t STS_ACC = 41;             // block: ACC, POS_L, POS_H, TIME_L, TIME_H, SPD_L, SPD_H
static const uint8_t STS_GOAL_POSITION_L = 42;
static const uint8_t STS_GOAL_SPEED_L = 46;
static const uint8_t STS_TORQUE_LIMIT_L = 48;
static const uint8_t STS_LOCK = 55;
static const uint8_t STS_PRESENT_POSITION_L = 56;  // feedback block starts here
static const uint8_t STS_PRESENT_CURRENT_H = 70;   // ... ends here (15 bytes)
static const uint8_t FEEDBACK_LEN = STS_PRESENT_CURRENT_H - STS_PRESENT_POSITION_L + 1;  // 15

// TORQUE_ENABLE magic value that triggers mid-point calibration.
static const uint8_t STS_CALIBRATE_MIDPOINT = 128;

enum SensorField : uint8_t {
  FIELD_POSITION = 0,
  FIELD_SPEED,
  FIELD_LOAD,
  FIELD_VOLTAGE,
  FIELD_TEMPERATURE,
  FIELD_CURRENT,
};

enum NumberField : uint8_t {
  NUM_TARGET_POSITION = 0,
  NUM_GOAL_SPEED,
  NUM_ACCELERATION,
  NUM_TORQUE_LIMIT,
};

// Operating-mode select option strings (must match select.py).
static const char *const MODE_OPTIONS[4] = {"Position", "Wheel", "PWM", "Step"};

class ST3215Bus : public PollingComponent, public uart::UARTDevice {
 public:
  void setup() override;
  void update() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  // ---- Registration (called from codegen) ----
  void register_sensor(uint8_t servo_id, SensorField field, sensor::Sensor *obj);
  void register_moving(uint8_t servo_id, binary_sensor::BinarySensor *obj);
  void register_number(uint8_t servo_id, NumberField field, number::Number *obj);
  void register_torque_switch(uint8_t servo_id, switch_::Switch *obj);
  void register_mode_select(uint8_t servo_id, select::Select *obj);

  // ---- Control API (called from entity subclasses) ----
  bool write_position(uint8_t servo_id, int32_t position);
  void set_goal_speed(uint8_t servo_id, uint16_t speed);
  void set_goal_acc(uint8_t servo_id, uint8_t acc);
  bool write_torque_limit(uint8_t servo_id, uint16_t limit);
  bool set_torque(uint8_t servo_id, bool enable);
  bool set_mode(uint8_t servo_id, uint8_t mode);
  bool calibrate_center(uint8_t servo_id);
  int ping(uint8_t servo_id);

 protected:
  struct ServoState {
    uint8_t servo_id;
    uint16_t goal_speed{1000};
    uint8_t goal_acc{50};
  };
  struct SensorReg {
    uint8_t servo_id;
    SensorField field;
    sensor::Sensor *obj;
  };
  struct MovingReg {
    uint8_t servo_id;
    binary_sensor::BinarySensor *obj;
  };
  struct NumberReg {
    uint8_t servo_id;
    NumberField field;
    number::Number *obj;
  };
  struct SwitchReg {
    uint8_t servo_id;
    switch_::Switch *obj;
  };
  struct SelectReg {
    uint8_t servo_id;
    select::Select *obj;
  };

  ServoState *get_state_(uint8_t servo_id);
  void publish_feedback_(uint8_t servo_id, const uint8_t *buf);

  // ---- Wire protocol ----
  bool feedback_read_(uint8_t servo_id, uint8_t *buf);           // 15-byte block from reg 56
  int read_reg_(uint8_t servo_id, uint8_t addr, uint8_t *out, uint8_t len);
  int read_byte_reg_(uint8_t servo_id, uint8_t addr);           // -1 on failure
  bool write_reg_(uint8_t servo_id, uint8_t addr, const uint8_t *data, uint8_t len);
  bool write_byte_reg_(uint8_t servo_id, uint8_t addr, uint8_t value);
  bool write_word_reg_(uint8_t servo_id, uint8_t addr, uint16_t value);
  bool unlock_eeprom_(uint8_t servo_id) { return write_byte_reg_(servo_id, STS_LOCK, 0); }
  bool lock_eeprom_(uint8_t servo_id) { return write_byte_reg_(servo_id, STS_LOCK, 1); }

  void flush_rx_();
  bool find_header_(uint32_t timeout_ms);
  bool read_exact_(uint8_t *dst, int len, uint32_t timeout_ms);
  bool read_ack_(uint8_t servo_id);

  std::vector<ServoState> servos_;
  std::vector<SensorReg> sensors_;
  std::vector<MovingReg> movings_;
  std::vector<NumberReg> numbers_;
  std::vector<SwitchReg> switches_;
  std::vector<SelectReg> selects_;
};

// ---- Entity subclasses ----
class ST3215Number : public number::Number, public Parented<ST3215Bus> {
 public:
  void set_servo_id(uint8_t id) { servo_id_ = id; }
  void set_field(NumberField field) { field_ = field; }

 protected:
  void control(float value) override;
  uint8_t servo_id_{1};
  NumberField field_{NUM_TARGET_POSITION};
};

class ST3215TorqueSwitch : public switch_::Switch, public Parented<ST3215Bus> {
 public:
  void set_servo_id(uint8_t id) { servo_id_ = id; }

 protected:
  void write_state(bool state) override;
  uint8_t servo_id_{1};
};

class ST3215ModeSelect : public select::Select, public Parented<ST3215Bus> {
 public:
  void set_servo_id(uint8_t id) { servo_id_ = id; }

 protected:
  void control(const std::string &value) override;
  uint8_t servo_id_{1};
};

class ST3215CalibrateButton : public button::Button, public Parented<ST3215Bus> {
 public:
  void set_servo_id(uint8_t id) { servo_id_ = id; }

 protected:
  void press_action() override;
  uint8_t servo_id_{1};
};

}  // namespace st3215
}  // namespace esphome
