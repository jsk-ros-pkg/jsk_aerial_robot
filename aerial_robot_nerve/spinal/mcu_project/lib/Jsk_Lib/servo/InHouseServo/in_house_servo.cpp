/**
******************************************************************************
* File Name          : in_house_servo.cpp
* Description        : interface for in-house servo
 ------------------------------------------------------------------*/

#include "in_house_servo.h"

using namespace InHouse_Servo;

Servo::Servo():
  servo_state_pub_("servo/states", &servo_state_msg_),
  servo_cmd_sub_("servo/target_states", &Servo::servoControlCallback, this),
  servo_torque_ctrl_sub_("servo/torque_enable", &Servo::servoTorqueControlCallback,this)
{
  servo_last_pub_time_ = 0;
  servo_last_ctrl_time_ = 0;  
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
    nh_->subscribe(servo_torque_ctrl_sub_);

    CANDeviceManager::CAN_START();
  }

void Servo::sendData()
{
  sendMessage(canTxId, 8, inst_data_, 0);
  memset(inst_data_, 0, sizeof(inst_data_));//clear instruction
}

void Servo::receiveDataCallback(uint32_t identifier, uint32_t dlc, uint8_t* data)
{
  // Decode current position
  current_pos_i_ = (static_cast<int16_t>(data[0]) << 8)  |
    static_cast<int16_t>(data[1]);
  current_pos_f_ = (static_cast<float>(current_pos_i_) / 4096.0f) * (2.0f * static_cast<float>(M_PI));

  // Decode current rpm
  current_rpm_i_ = (static_cast<int16_t>(data[2]) << 8)  |
    static_cast<int16_t>(data[3]);
    
  // Decode current current
  current_current_i_ = (static_cast<int16_t>(data[4]) << 8) |
    static_cast<int16_t>(data[5]);
  current_current_f_ = static_cast<float>(current_current_i_) / 100.0f;
    
  // Decode current motor state
  current_state_ = data[6];


  //set date to ros_msg
  servo_state_msg_.index = identifier;
  servo_state_msg_.angle = current_pos_i_;
  servo_state_msg_.rpm = current_rpm_i_;
  servo_state_msg_.current = current_current_i_;
  servo_state_msg_.state = current_state_;
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
      servo_state_pub_.publish(&servo_state_msg_);
      servo_last_pub_time_ = now_time;
    }
}

void Servo::servoControlCallback(const spinal::ServoControlCmd& control_msg)
{
  for (unsigned int i = 0; i < control_msg.index_length; i++) {
    uint8_t index = control_msg.index[i];
    if(index == servo_id_)
      {
        int16_t goal_pos = (control_msg.angles[i]);
        inst_data_[0] =0x03;
        inst_data_[1] = (uint8_t)((goal_pos >> 8) & 0xFF);
        inst_data_[2] = (uint8_t)(goal_pos & 0xFF);
      }
  }
}

void Servo::servoTorqueControlCallback(const spinal::ServoTorqueCmd& control_msg)
{
  for (unsigned int i = 0; i < control_msg.index_length; i++) {
    uint8_t index = control_msg.index[i];
    if(index == servo_id_)
      {
        if(control_msg.torque_enable[i])
          inst_data_[0] = 0x01;
        else
          inst_data_[0] = 0x02;
      }
  }
}
