/*
 * motor.cpp
 *
 *  Created on: 2016/10/28
 *      Author: anzai
 *  Maintainer : Bakui Chou
 */

#include "motor.h"
#include "CAN/can_device_manager.h"

namespace
{
void writeUint16(uint8_t* data, uint16_t value)
{
  data[0] = value & 0xFF;
  data[1] = (value >> 8) & 0xFF;
}

void writeUint32(uint8_t* data, uint32_t value)
{
  data[0] = value & 0xFF;
  data[1] = (value >> 8) & 0xFF;
  data[2] = (value >> 16) & 0xFF;
  data[3] = (value >> 24) & 0xFF;
}

uint16_t readUint16(const uint8_t* data)
{
  return static_cast<uint16_t>(data[0]) |
         (static_cast<uint16_t>(data[1]) << 8);
}
}

void Motor::init(TIM_HandleTypeDef* htim)
{
  pwm_htim_ = htim;
  HAL_TIM_PWM_Start(pwm_htim_, TIM_CHANNEL_1);

#ifdef STM32G4
  HAL_TIM_PWM_Start(pwm_htim_, TIM_CHANNEL_2);
#endif
}

void Motor::sendData()
{
#ifdef STM32G4
  uint8_t valid_mask = 0;
  for (uint8_t channel = 0; channel < rotor_state_.size(); ++channel) {
    if ((rotor_state_[channel].flags & CAN::ROTOR_STATE_VALID) != 0) {
      valid_mask |= 1U << channel;
    }
  }
  if (valid_mask == 0) return;

  uint8_t data[CAN::CANFD_ROTOR_STATE_DLC] = {};
  data[0] = CAN::CANFD_PROTOCOL_VERSION;
  data[1] = state_sequence_++;
  data[2] = valid_mask;
  data[3] = 0; // Reserved for the future ESC telemetry transport type.

  for (uint8_t channel = 0; channel < rotor_state_.size(); ++channel) {
    const RotorTelemetry& telemetry = rotor_state_[channel];
    uint8_t* record = data + 4 + channel * 14;
    record[0] = telemetry.flags;
    record[1] = static_cast<uint8_t>(telemetry.temperature);
    writeUint16(record + 2, telemetry.voltage);
    writeUint16(record + 4, telemetry.current);
    writeUint16(record + 6, telemetry.consumption);
    writeUint32(record + 8, telemetry.rpm);
    record[12] = telemetry.crc_error;
  }

  if (sendMessage(CAN::MESSAGEID_SEND_ROTOR_STATE, m_slave_id,
                  CAN::CANFD_ROTOR_STATE_DLC, data, 0)) {
    __atomic_add_fetch(&feedback_count_, 1, __ATOMIC_RELAXED);
  }
#endif
}

void Motor::receiveDataCallback(uint8_t message_id, uint32_t DLC,
                                uint8_t* data)
{
#ifdef STM32G4
  if (message_id != CAN::MESSAGEID_RECEIVE_ROTOR_TARGET ||
      DLC != CAN::CANFD_ROTOR_TARGET_DLC ||
      data[0] != CAN::CANFD_PROTOCOL_VERSION) {
    return;
  }

  const uint8_t motor_count =
    data[2] > CAN::CANFD_MAX_NEURON_NUM
      ? CAN::CANFD_MAX_NEURON_NUM : data[2];
  for (uint8_t i = 0; i < motor_count; ++i) {
    const uint8_t* record = data + 4 + i * 5;
    if (record[0] == m_slave_id) {
      __atomic_add_fetch(&target_count_, 1, __ATOMIC_RELAXED);
      setSyncPwm(readUint16(record + 1), readUint16(record + 3));
      return;
    }
  }
#else
  auto getPwmFromData = [=](int index)->uint16_t {
    switch (index) {
    case 1:
      return ((data[1] << 8) & 0x300) | (data[0] & 0xFF);
    case 2:
      return ((data[2] << 6) & 0x3C0) | ((data[1] >> 2) & 0x3F);
    case 3:
      return ((data[3] << 4) & 0x3F0) | ((data[2] >> 4) & 0x0F);
    case 4:
      return ((data[5] << 8) & 0x300) | (data[4] & 0xFF);
    case 5:
      return ((data[6] << 6) & 0x3C0) | ((data[5] >> 2) & 0x3F);
    case 6:
      return ((data[7] << 4) & 0x3F0) | ((data[6] >> 4) & 0x0F);
    default:
      return 0;
    }
  };

  if (message_id == CAN::MESSAGEID_RECEIVE_PWM_0_5 &&
      1 <= m_slave_id && m_slave_id <= 6) {
    const uint16_t pwm = getPwmFromData(m_slave_id);
    setSyncPwm(pwm, pwm);
  } else if (message_id == CAN::MESSAGEID_RECEIVE_PWM_6_11 &&
             7 <= m_slave_id && m_slave_id <= 12) {
    const uint16_t pwm = getPwmFromData(m_slave_id - 6);
    setSyncPwm(pwm, pwm);
  }
#endif
}

void Motor::setSyncPwm(uint16_t pwm_0, uint16_t pwm_1)
{
  const uint32_t raw_0 =
    (pwm_0 + 1000.0f) / 2000.0f * pwm_htim_->Init.Period;
  pwm_htim_->Instance->CCR1 = raw_0;

#ifdef STM32G4
  const uint32_t raw_1 =
    (pwm_1 + 1000.0f) / 2000.0f * pwm_htim_->Init.Period;
  pwm_htim_->Instance->CCR2 = raw_1;
#else
  (void)pwm_1;
#endif
}

void Motor::update()
{
  if (CANDeviceManager::isTimeout()) {
    setSyncPwm(0, 0); // force stop all rotors
  }
}

uint32_t Motor::takeTargetCount()
{
  return __atomic_exchange_n(&target_count_, 0, __ATOMIC_ACQ_REL);
}

uint32_t Motor::takeFeedbackCount()
{
  return __atomic_exchange_n(&feedback_count_, 0, __ATOMIC_ACQ_REL);
}
