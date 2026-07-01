#include "st3215.h"
#include "esphome/core/log.h"
#include <cmath>

namespace esphome {
namespace st3215 {

static const char *const TAG = "st3215";

// Response timeouts. At 1 Mbps a 15-byte reply lands in ~150 us; keep these
// small so a missing servo doesn't stall the main loop.
static const uint32_t ACK_TIMEOUT_MS = 12;
static const uint32_t READ_TIMEOUT_MS = 20;

// ============================ lifecycle ============================

void ST3215Bus::setup() {
  ESP_LOGCONFIG(TAG, "Setting up ST3215 bus...");
  for (auto &st : this->servos_) {
    delay(5);
    int id = this->ping(st.servo_id);
    if (id < 0) {
      ESP_LOGW(TAG, "Servo %u did not respond to PING", st.servo_id);
    } else {
      ESP_LOGCONFIG(TAG, "Servo %u present", st.servo_id);
    }

    // Seed the target-position number with the servo's current position so the
    // UI doesn't jump the servo on first interaction.
    uint8_t buf[FEEDBACK_LEN];
    if (this->feedback_read_(st.servo_id, buf)) {
      int32_t pos = buf[0] | (buf[1] << 8);
      if (pos & 0x8000)
        pos = -(pos & 0x7FFF);
      for (auto &n : this->numbers_)
        if (n.servo_id == st.servo_id && n.field == NUM_TARGET_POSITION)
          n.obj->publish_state((float) pos);
    }

    // Seed the torque-limit number from EEPROM.
    uint8_t tl[2];
    if (this->read_reg_(st.servo_id, STS_TORQUE_LIMIT_L, tl, 2) == 2) {
      uint16_t v = tl[0] | (tl[1] << 8);
      for (auto &n : this->numbers_)
        if (n.servo_id == st.servo_id && n.field == NUM_TORQUE_LIMIT)
          n.obj->publish_state((float) v);
    }

    // Seed speed/accel numbers from our stored defaults.
    for (auto &n : this->numbers_) {
      if (n.servo_id != st.servo_id)
        continue;
      if (n.field == NUM_GOAL_SPEED)
        n.obj->publish_state((float) st.goal_speed);
      else if (n.field == NUM_ACCELERATION)
        n.obj->publish_state((float) st.goal_acc);
    }
  }
}

void ST3215Bus::update() {
  for (auto &st : this->servos_) {
    uint8_t id = st.servo_id;

    uint8_t buf[FEEDBACK_LEN];
    if (this->feedback_read_(id, buf)) {
      this->publish_feedback_(id, buf);
    }

    // Reflect live torque-enable state.
    for (auto &sw : this->switches_) {
      if (sw.servo_id != id)
        continue;
      int v = this->read_byte_reg_(id, STS_TORQUE_ENABLE);
      if (v >= 0)
        sw.obj->publish_state(v != 0);
    }

    // Reflect live operating mode.
    for (auto &se : this->selects_) {
      if (se.servo_id != id)
        continue;
      int v = this->read_byte_reg_(id, STS_MODE);
      if (v >= 0 && v < 4)
        se.obj->publish_state(MODE_OPTIONS[v]);
    }
  }
}

void ST3215Bus::dump_config() {
  ESP_LOGCONFIG(TAG, "ST3215 servo bus:");
  ESP_LOGCONFIG(TAG, "  Servos configured: %u", (unsigned) this->servos_.size());
  for (auto &st : this->servos_)
    ESP_LOGCONFIG(TAG, "    - ID %u (default speed %u, accel %u)", st.servo_id, st.goal_speed, st.goal_acc);
  this->check_uart_settings(1000000);
}

// ============================ registration ============================

ST3215Bus::ServoState *ST3215Bus::get_state_(uint8_t servo_id) {
  for (auto &st : this->servos_)
    if (st.servo_id == servo_id)
      return &st;
  this->servos_.push_back(ServoState{servo_id});
  return &this->servos_.back();
}

void ST3215Bus::register_sensor(uint8_t servo_id, SensorField field, sensor::Sensor *obj) {
  this->get_state_(servo_id);
  this->sensors_.push_back(SensorReg{servo_id, field, obj});
}
void ST3215Bus::register_moving(uint8_t servo_id, binary_sensor::BinarySensor *obj) {
  this->get_state_(servo_id);
  this->movings_.push_back(MovingReg{servo_id, obj});
}
void ST3215Bus::register_number(uint8_t servo_id, NumberField field, number::Number *obj) {
  this->get_state_(servo_id);
  this->numbers_.push_back(NumberReg{servo_id, field, obj});
}
void ST3215Bus::register_torque_switch(uint8_t servo_id, switch_::Switch *obj) {
  this->get_state_(servo_id);
  this->switches_.push_back(SwitchReg{servo_id, obj});
}
void ST3215Bus::register_mode_select(uint8_t servo_id, select::Select *obj) {
  this->get_state_(servo_id);
  this->selects_.push_back(SelectReg{servo_id, obj});
}

// ============================ feedback decode ============================

void ST3215Bus::publish_feedback_(uint8_t servo_id, const uint8_t *buf) {
  // Indices are relative to STS_PRESENT_POSITION_L (reg 56).
  int32_t pos = buf[0] | (buf[1] << 8);
  if (pos & 0x8000)
    pos = -(pos & 0x7FFF);

  int32_t speed = buf[2] | (buf[3] << 8);
  if (speed & 0x8000)
    speed = -(speed & 0x7FFF);

  int32_t load_raw = buf[4] | (buf[5] << 8);  // 0..1000, sign bit is bit 10
  float load_pct = (load_raw & 0x0400) ? -((load_raw & 0x03FF) / 10.0f) : ((load_raw & 0x03FF) / 10.0f);

  float voltage = buf[6] / 10.0f;  // 0.1 V units
  float temperature = (float) buf[7];
  bool moving = buf[10] != 0;

  int32_t cur_raw = buf[13] | (buf[14] << 8);
  if (cur_raw & 0x8000)
    cur_raw = -(cur_raw & 0x7FFF);
  float current_ma = cur_raw * 6.5f;  // ~6.5 mA per LSB (Feetech STS)

  for (auto &s : this->sensors_) {
    if (s.servo_id != servo_id)
      continue;
    switch (s.field) {
      case FIELD_POSITION:
        s.obj->publish_state((float) pos);
        break;
      case FIELD_POSITION_DEG:
        s.obj->publish_state(pos * 360.0f / 4096.0f);  // 12-bit encoder -> degrees
        break;
      case FIELD_SPEED:
        s.obj->publish_state((float) speed);
        break;
      case FIELD_LOAD:
        s.obj->publish_state(load_pct);
        break;
      case FIELD_VOLTAGE:
        s.obj->publish_state(voltage);
        break;
      case FIELD_TEMPERATURE:
        s.obj->publish_state(temperature);
        break;
      case FIELD_CURRENT:
        s.obj->publish_state(current_ma);
        break;
    }
  }
  for (auto &m : this->movings_)
    if (m.servo_id == servo_id)
      m.obj->publish_state(moving);
}

// ============================ control API ============================

bool ST3215Bus::write_position(uint8_t servo_id, int32_t position) {
  ServoState *st = this->get_state_(servo_id);
  uint16_t pos = (position < 0) ? ((uint16_t) (-position) | 0x8000) : (uint16_t) position;
  uint16_t speed = st->goal_speed;
  // Block write starting at STS_ACC (41): ACC, POS_L, POS_H, TIME_L, TIME_H, SPD_L, SPD_H
  uint8_t p[7];
  p[0] = st->goal_acc;
  p[1] = pos & 0xFF;
  p[2] = pos >> 8;
  p[3] = 0;
  p[4] = 0;
  p[5] = speed & 0xFF;
  p[6] = speed >> 8;
  bool ok = this->write_reg_(servo_id, STS_ACC, p, 7);
  if (!ok)
    ESP_LOGW(TAG, "Servo %u: move to %d failed", servo_id, position);
  return ok;
}

void ST3215Bus::set_goal_speed(uint8_t servo_id, uint16_t speed) { this->get_state_(servo_id)->goal_speed = speed; }
void ST3215Bus::set_goal_acc(uint8_t servo_id, uint8_t acc) { this->get_state_(servo_id)->goal_acc = acc; }

bool ST3215Bus::write_torque_limit(uint8_t servo_id, uint16_t limit) {
  return this->write_word_reg_(servo_id, STS_TORQUE_LIMIT_L, limit);
}

bool ST3215Bus::set_torque(uint8_t servo_id, bool enable) {
  return this->write_byte_reg_(servo_id, STS_TORQUE_ENABLE, enable ? 1 : 0);
}

bool ST3215Bus::set_mode(uint8_t servo_id, uint8_t mode) {
  // Mode lives in EEPROM; unlock, write, re-lock.
  bool ok = this->unlock_eeprom_(servo_id);
  ok &= this->write_byte_reg_(servo_id, STS_MODE, mode);
  ok &= this->lock_eeprom_(servo_id);
  return ok;
}

bool ST3215Bus::calibrate_center(uint8_t servo_id) {
  ESP_LOGI(TAG, "Servo %u: calibrating mid-point", servo_id);
  return this->write_byte_reg_(servo_id, STS_TORQUE_ENABLE, STS_CALIBRATE_MIDPOINT);
}

// ============================ wire protocol ============================

void ST3215Bus::flush_rx_() {
  uint8_t b;
  while (this->available())
    this->read_byte(&b);
}

bool ST3215Bus::read_exact_(uint8_t *dst, int len, uint32_t timeout_ms) {
  int got = 0;
  uint32_t t = millis();
  while (got < len) {
    if (this->available()) {
      uint8_t b;
      if (this->read_byte(&b)) {
        dst[got++] = b;
        t = millis();
      }
    } else if (millis() - t > timeout_ms) {
      return false;
    }
  }
  return true;
}

bool ST3215Bus::find_header_(uint32_t timeout_ms) {
  uint8_t last = 0, cur = 0;
  int scanned = 0;
  uint32_t t = millis();
  while (true) {
    if (this->available()) {
      uint8_t b;
      if (this->read_byte(&b)) {
        last = cur;
        cur = b;
        t = millis();
        if (last == 0xFF && cur == 0xFF)
          return true;
        if (++scanned > 32)
          return false;
      }
    } else if (millis() - t > timeout_ms) {
      return false;
    }
  }
}

bool ST3215Bus::read_ack_(uint8_t servo_id) {
  if (!this->find_header_(ACK_TIMEOUT_MS))
    return false;
  uint8_t b[4];
  if (!this->read_exact_(b, 4, ACK_TIMEOUT_MS))
    return false;
  if (b[0] != servo_id || b[1] != 2)
    return false;
  uint8_t chk = ~(uint8_t) (b[0] + b[1] + b[2]);
  return chk == b[3];
}

bool ST3215Bus::write_reg_(uint8_t servo_id, uint8_t addr, const uint8_t *data, uint8_t len) {
  this->flush_rx_();
  uint8_t msg_len = len + 3;  // 2 + len + 1(addr)
  uint8_t chk = servo_id + msg_len + INST_WRITE + addr;
  uint8_t head[6] = {0xFF, 0xFF, servo_id, msg_len, INST_WRITE, addr};
  this->write_array(head, 6);
  for (uint8_t i = 0; i < len; i++)
    chk += data[i];
  if (len)
    this->write_array(data, len);
  this->write_byte((uint8_t) ~chk);
  this->flush();
  return this->read_ack_(servo_id);
}

bool ST3215Bus::write_byte_reg_(uint8_t servo_id, uint8_t addr, uint8_t value) {
  return this->write_reg_(servo_id, addr, &value, 1);
}

bool ST3215Bus::write_word_reg_(uint8_t servo_id, uint8_t addr, uint16_t value) {
  uint8_t d[2] = {(uint8_t) (value & 0xFF), (uint8_t) (value >> 8)};  // little-endian (End=0)
  return this->write_reg_(servo_id, addr, d, 2);
}

int ST3215Bus::read_reg_(uint8_t servo_id, uint8_t addr, uint8_t *out, uint8_t len) {
  this->flush_rx_();
  uint8_t msg_len = 4;  // 2 + 1(len byte) + 1(addr)
  uint8_t chk = servo_id + msg_len + INST_READ + addr + len;
  uint8_t pkt[8] = {0xFF, 0xFF, servo_id, msg_len, INST_READ, addr, len, (uint8_t) ~chk};
  this->write_array(pkt, 8);
  this->flush();

  if (!this->find_header_(READ_TIMEOUT_MS))
    return -1;
  uint8_t hdr[3];  // ID, Length, Error
  if (!this->read_exact_(hdr, 3, READ_TIMEOUT_MS))
    return -1;
  if (hdr[0] != servo_id)
    return -1;
  if (!this->read_exact_(out, len, READ_TIMEOUT_MS))
    return -1;
  uint8_t csum;
  if (!this->read_exact_(&csum, 1, READ_TIMEOUT_MS))
    return -1;
  uint8_t calc = hdr[0] + hdr[1] + hdr[2];
  for (uint8_t i = 0; i < len; i++)
    calc += out[i];
  if ((uint8_t) ~calc != csum)
    return -1;
  return len;
}

int ST3215Bus::read_byte_reg_(uint8_t servo_id, uint8_t addr) {
  uint8_t v;
  return this->read_reg_(servo_id, addr, &v, 1) == 1 ? (int) v : -1;
}

bool ST3215Bus::feedback_read_(uint8_t servo_id, uint8_t *buf) {
  return this->read_reg_(servo_id, STS_PRESENT_POSITION_L, buf, FEEDBACK_LEN) == FEEDBACK_LEN;
}

int ST3215Bus::ping(uint8_t servo_id) {
  this->flush_rx_();
  uint8_t msg_len = 2;
  uint8_t chk = servo_id + msg_len + INST_PING;
  uint8_t pkt[6] = {0xFF, 0xFF, servo_id, msg_len, INST_PING, (uint8_t) ~chk};
  this->write_array(pkt, 6);
  this->flush();
  if (!this->find_header_(ACK_TIMEOUT_MS))
    return -1;
  uint8_t b[4];
  if (!this->read_exact_(b, 4, ACK_TIMEOUT_MS))
    return -1;
  if (b[0] != servo_id || b[1] != 2)
    return -1;
  return b[0];
}

// ============================ entity subclasses ============================

void ST3215Number::control(float value) {
  switch (this->field_) {
    case NUM_TARGET_POSITION:
      this->parent_->write_position(this->servo_id_, (int32_t) lroundf(value));
      break;
    case NUM_GOAL_SPEED:
      this->parent_->set_goal_speed(this->servo_id_, (uint16_t) lroundf(value));
      break;
    case NUM_ACCELERATION:
      this->parent_->set_goal_acc(this->servo_id_, (uint8_t) lroundf(value));
      break;
    case NUM_TORQUE_LIMIT:
      this->parent_->write_torque_limit(this->servo_id_, (uint16_t) lroundf(value));
      break;
  }
  this->publish_state(value);
}

void ST3215TorqueSwitch::write_state(bool state) {
  this->parent_->set_torque(this->servo_id_, state);
  this->publish_state(state);
}

void ST3215ModeSelect::control(const std::string &value) {
  auto idx = this->index_of(value);
  if (idx.has_value()) {
    this->parent_->set_mode(this->servo_id_, (uint8_t) idx.value());
    this->publish_state(value);
  }
}

void ST3215CalibrateButton::press_action() { this->parent_->calibrate_center(this->servo_id_); }

}  // namespace st3215
}  // namespace esphome
