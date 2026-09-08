/*
 * can_motor.h
 *
 *  Created on: 2016/11/01
 *      Author: anzai
 */

#ifndef APPLICATION_HYDRUS_LIB_CANDEVICE_MOTOR_CAN_MOTOR_H_
#define APPLICATION_HYDRUS_LIB_CANDEVICE_MOTOR_CAN_MOTOR_H_

#include "CAN/can_device.h"
#include <array>
#include <functional>

struct RotorTelemetry
{
  uint8_t flags;
  int8_t temperature;
  uint16_t voltage;
  uint16_t current;
  uint16_t consumption;
  uint32_t rpm;
  uint8_t crc_error;

  RotorTelemetry():flags(0), temperature(0), voltage(0), current(0),
                   consumption(0), rpm(0), crc_error(0){}
};

class CANMotor : public CANDevice
{
private:
  uint16_t pwm_[2];
  std::array<RotorTelemetry, 2> rotor_state_;
public:
  CANMotor():pwm_{0, 0}{}
  CANMotor(uint8_t slave_id) : CANDevice(CAN::DEVICEID_MOTOR, slave_id),
                               pwm_{0, 0}{}
  void sendData() override;
  void receiveDataCallback(uint8_t slave_id, uint8_t message_id,
                           uint32_t DLC, uint8_t* data) override;
  void setPwm(uint16_t pwm){pwm_[0] = pwm; pwm_[1] = pwm;}
  void setPwm(uint8_t channel, uint16_t pwm)
  {
    if (channel < 2) pwm_[channel] = pwm;
  }
  uint16_t getPwm(uint8_t channel = 0) const
  {
    return channel < 2 ? pwm_[channel] : 0;
  }
  const RotorTelemetry& getRotorTelemetry(uint8_t channel) const
  {
    return rotor_state_.at(channel);
  }
};

class CANMotorSendDevice : public CANDevice
{
private:
  std::vector<std::reference_wrapper<CANMotor>> can_motor_;
  uint8_t target_sequence_;
public:
  CANMotorSendDevice():CANDevice(CAN::DEVICEID_MOTOR, CAN::BROADCAST_ID),
                       target_sequence_(0) {}
  void sendData() override;
  void receiveDataCallback(uint8_t slave_id, uint8_t message_id,
                           uint32_t DLC, uint8_t* data) override;
  void addMotor(CANMotor& motor) {can_motor_.push_back(motor);}
};

#endif /* APPLICATION_HYDRUS_LIB_CANDEVICE_MOTOR_CAN_MOTOR_H_ */
