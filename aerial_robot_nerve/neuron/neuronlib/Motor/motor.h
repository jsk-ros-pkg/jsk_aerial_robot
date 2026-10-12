/*
 * motor.h
 *
 *  Created on: 2016/10/28
 *      Author: anzai
 *  Maintainer: bakui chou
 */

#ifndef APPLICATION_MOTOR_TEMP_MOTOR_H_
#define APPLICATION_MOTOR_TEMP_MOTOR_H_

#include "CAN/can_device.h"
#include <array>

#define IDLE_DUTY 0.5f // 1000[usec] / 2000[usec]
#define MIN_DUTY_DEFAULT  0.55f   //1100 / 2000
#define MAX_DUTY_DEFAULT 0.95f // 1900 / 2000

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

class Motor : public CANDevice
{
private:
  TIM_HandleTypeDef* pwm_htim_;
  std::array<RotorTelemetry, 2> rotor_state_;
  uint8_t state_sequence_;
  volatile uint32_t target_count_;
  volatile uint32_t feedback_count_;
  void setSyncPwm(uint16_t pwm_0, uint16_t pwm_1);
public:
  Motor():pwm_htim_(nullptr), state_sequence_(0), target_count_(0),
          feedback_count_(0){}
  Motor(uint8_t slave_id):CANDevice(CAN::DEVICEID_MOTOR, slave_id),
                          pwm_htim_(nullptr), state_sequence_(0), target_count_(0),
                          feedback_count_(0){}
  void init(TIM_HandleTypeDef* htim);
  void update();
  void sendData() override;
  void receiveDataCallback(uint8_t message_id, uint32_t DLC,
                           uint8_t* data) override;
  uint32_t takeTargetCount();
  uint32_t takeFeedbackCount();
  void setRotorTelemetry(uint8_t channel, const RotorTelemetry& telemetry)
  {
    if (channel < rotor_state_.size()) rotor_state_[channel] = telemetry;
  }
};

#endif /* APPLICATION_MOTOR_H_ */
