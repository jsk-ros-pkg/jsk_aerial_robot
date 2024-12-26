/**
******************************************************************************
* File Name          : servo.cpp
* Description        : interface for DJI M2006 with C610
 ------------------------------------------------------------------*/

#include "servo.h"

using namespace DJI_M2006;

Servo::Servo(): servo_state_pub_("servo/states", &servo_state_msg_), servo_cmd_sub_("servo/target_states", &Servo::servoControlCallback, this)
{
  last_connected_time_ = 0;
  servo_last_pub_time_ = 0;
  servo_last_ctrl_time_ = 0;

  rotations_ = 0;
  last_pos_measurement_ = kCountsPerRev;
  counts_ = 0;
  rpm_ = 0;
  current_ = 0;

  goal_current_ = 0;
}

void Servo::init(CAN_GeranlHandleTypeDef* hcan, osMailQId* handle, ros::NodeHandle* nh, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
  {
    /* CAN */
    CANDeviceManager::init(hcan, GPIOx, GPIO_Pin);
    CANDeviceManager::useRTOS(handle);
    CANDeviceManager::addDirectDevice(this);

    /* ros */
    nh_ = nh;
    nh_->advertise(servo_state_pub_);
    nh_->subscribe(servo_cmd_sub_);

    CANDeviceManager::CAN_START();
  }

void Servo::sendData()
{
  uint8_t cmd[8];
  cmd[0] = (goal_current_ >> 8) & 0xFF;
  cmd[1] = goal_current_ & 0xFF;

  sendMessage(canTxId, 8, cmd, 0);
}

void Servo::receiveDataCallback(uint32_t identifier, uint32_t dlc, uint8_t* data)
{
  uint16_t counts = uint16_t((data[0] << 8) | data[1]);
  rpm_ = int16_t((data[2] << 8) | data[3]);
  current_ = int16_t((data[4] << 8) | data[5]);

  if (last_pos_measurement_ == 8192) {
    last_pos_measurement_ = counts;
  }

  int32_t delta = counts - last_pos_measurement_;
  if (delta > kCountsPerRev / 2) {
    // Crossed from >= 0 counts to <= 8191 counts. Could
    // also trigger if spinning super fast (>2000rps)
    rotations_ -= 1;
  } else if (delta < -kCountsPerRev / 2) {
    // Crossed from <= 8191 counts to >= 0 counts. Could
    // also trigger if spinning super fast (>2000rps)
    rotations_ += 1;
  }

  counts_ = rotations_ * kCountsPerRev + counts;
  last_pos_measurement_ = counts;
}

void Servo::update(void)
{
  CANDeviceManager::tick(1);
  uint32_t now_time = HAL_GetTick();

  /* control */
  if( now_time - servo_last_ctrl_time_ >= SERVO_CTRL_INTERVAL)
    {
      sendData();
      servo_last_ctrl_time_ = now_time;
    }

  /* ros publish */

  if( now_time - servo_last_pub_time_ >= SERVO_PUB_INTERVAL)
    {
      /* send servo */
      servo_state_msg_.index = 0;
      servo_state_msg_.angle = counts_;
      servo_state_msg_.rpm = rpm_;
      servo_state_msg_.current = current_;

      servo_state_pub_.publish(&servo_state_msg_);
      servo_last_pub_time_ = now_time;
    }
}

void Servo::servoControlCallback(const spinal::ServoControlCmd& control_msg)
{
  goal_current_ = control_msg.angles[0];
}


