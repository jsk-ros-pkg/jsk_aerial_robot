/*
 * can_motor.cpp
 *
 *  Created on: 2016/11/01
 *      Author: anzai
 */

#include "can_motor.h"
#include <algorithm>

namespace
{
void writeUint16(uint8_t* data, uint16_t value)
{
  data[0] = value & 0xFF;
  data[1] = (value >> 8) & 0xFF;
}

uint16_t readUint16(const uint8_t* data)
{
  return static_cast<uint16_t>(data[0]) |
         (static_cast<uint16_t>(data[1]) << 8);
}

uint32_t readUint32(const uint8_t* data)
{
  return static_cast<uint32_t>(data[0]) |
         (static_cast<uint32_t>(data[1]) << 8) |
         (static_cast<uint32_t>(data[2]) << 16) |
         (static_cast<uint32_t>(data[3]) << 24);
}
}

void CANMotor::sendData()
{
}

void CANMotor::receiveDataCallback(uint8_t slave_id, uint8_t message_id,
                                   uint32_t DLC, uint8_t* data)
{
#ifdef STM32H7
  if (slave_id != m_slave_id ||
      message_id != CAN::MESSAGEID_SEND_ROTOR_STATE ||
      DLC != CAN::CANFD_ROTOR_STATE_DLC ||
      data[0] != CAN::CANFD_PROTOCOL_VERSION) {
    return;
  }

  const uint8_t valid_mask = data[2];
  for (uint8_t channel = 0; channel < 2; ++channel) {
    const uint8_t* record = data + 4 + channel * 14;
    RotorTelemetry& telemetry = rotor_state_[channel];
    telemetry.flags = record[0];
    if ((valid_mask & (1U << channel)) == 0) {
      telemetry.flags &= ~CAN::ROTOR_STATE_VALID;
      continue;
    }
    telemetry.temperature = static_cast<int8_t>(record[1]);
    telemetry.voltage = readUint16(record + 2);
    telemetry.current = readUint16(record + 4);
    telemetry.consumption = readUint16(record + 6);
    telemetry.rpm = readUint32(record + 8);
    telemetry.crc_error = record[12];
  }
#else
  (void)slave_id;
  (void)message_id;
  (void)DLC;
  (void)data;
#endif
}

void CANMotorSendDevice::sendData()
{
#ifdef STM32H7
  uint8_t data[CAN::CANFD_ROTOR_TARGET_DLC] = {};
  const uint8_t motor_count = std::min(
    static_cast<size_t>(CAN::CANFD_MAX_NEURON_NUM), can_motor_.size());

  data[0] = CAN::CANFD_PROTOCOL_VERSION;
  data[1] = target_sequence_++;
  data[2] = motor_count;
  data[3] = 0;

  for (uint8_t i = 0; i < motor_count; ++i) {
    CANMotor& motor = can_motor_.at(i).get();
    uint8_t* record = data + 4 + i * 5;
    record[0] = motor.getSlaveId();
    writeUint16(record + 1, motor.getPwm(0));
    writeUint16(record + 3, motor.getPwm(1));
  }

  sendMessage(CAN::MESSAGEID_RECEIVE_ROTOR_TARGET, CAN::BROADCAST_ID,
              CAN::CANFD_ROTOR_TARGET_DLC, data, 0);
#else
  uint8_t pwm_data[8] = {};
  uint16_t motor_pwms[6] = {};

  auto parse = [&]{
    pwm_data[0] = motor_pwms[0] & 0xFF;
    pwm_data[1] = ((motor_pwms[1] << 2) & 0xFC) | ((motor_pwms[0] >> 8) & 0x03);
    pwm_data[2] = ((motor_pwms[2] << 4) & 0xF0) | ((motor_pwms[1] >> 6) & 0x0F);
    pwm_data[3] = ((motor_pwms[2] >> 4) & 0x3F);
    pwm_data[4] = motor_pwms[3] & 0xFF;
    pwm_data[5] = ((motor_pwms[4] << 2) & 0xFC) | ((motor_pwms[3] >> 8) & 0x03);
    pwm_data[6] = ((motor_pwms[5] << 4) & 0xF0) | ((motor_pwms[4] >> 6) & 0x0F);
    pwm_data[7] = ((motor_pwms[5] >> 4) & 0x3F);
  };

  unsigned int send_motor_num = std::min(
    static_cast<int>(can_motor_.size()), 6);
  for (unsigned int i = 0; i < send_motor_num; i++) {
    motor_pwms[i] = can_motor_.at(i).get().getPwm();
  }

  parse();
  sendMessage(CAN::MESSAGEID_RECEIVE_PWM_0_5, CAN::BROADCAST_ID,
              8, pwm_data, 0);

  if (can_motor_.size() <= 6) return;

  send_motor_num = std::min(
    static_cast<int>(can_motor_.size() - 6), 6);
  for (unsigned int i = 0; i < send_motor_num; i++) {
    motor_pwms[i] = can_motor_.at(i + 6).get().getPwm();
  }

  parse();
  sendMessage(CAN::MESSAGEID_RECEIVE_PWM_6_11, CAN::BROADCAST_ID,
              8, pwm_data, 0);
#endif
}

void CANMotorSendDevice::receiveDataCallback(
  uint8_t slave_id, uint8_t message_id, uint32_t DLC, uint8_t* data)
{
  (void)slave_id;
  (void)message_id;
  (void)DLC;
  (void)data;
}
