/**
******************************************************************************
* File Name          : servo.cpp
* Description        : universal servo control interface for Spinal
* Author             : J.Sugihara (2024/3/1)
 ------------------------------------------------------------------*/


#include "servo.h"

#define  SERVO_PUB_INTERVAL 20 // 50Hz
#define SERVO_TORQUE_PUB_INTERVAL  1000 // 1Hz

void DirectServo::init(UART_HandleTypeDef* huart,  ros::NodeHandle* nh, osMutexId* mutex = NULL) //TODO: support encoder
{
  nh_ = nh;
  nh_->subscribe(servo_ctrl_sub_);
  nh_->subscribe(servo_torque_ctrl_sub_);
  nh_->advertise(servo_state_pub_);
  nh_->advertise(servo_torque_state_pub_);

  //temp
  servo_state_msg_.servos_length = 4;
  servo_state_msg_.servos = new spinal::ServoState[4]; 
  servo_torque_state_msg_.torque_enable_length = 4;
  servo_torque_state_msg_.torque_enable = new uint8_t[4];

  servo_handler_.init(huart, mutex);

  servo_last_pub_time_ = 0;
  servo_torque_last_pub_time_ = 0;
}

void DirectServo::update()
{
  servo_handler_.update();
  sendData();

}

void DirectServo::sendData()
{
  uint32_t now_time = HAL_GetTick();
  if( now_time - servo_last_pub_time_ >= SERVO_PUB_INTERVAL)
    {
      for (unsigned int i = 0; i < servo_handler_.getServoNum(); i++) {
        const ServoData& s = servo_handler_.getServo()[i];
        if (s.send_data_flag_ != 0) {
          spinal::ServoState servo;
          servo.index = s.id_;
          servo.angle = s.present_position_;
          servo.temp = s.present_temp_;
          servo.load = s.present_current_;
          servo.error = s.goal_position_;
          servo_state_msg_.servos[i] = servo;
        }
      }
      servo_state_pub_.publish(&servo_state_msg_);
      servo_last_pub_time_ = now_time;
    }
}

void DirectServo::servoControlCallback(const spinal::ServoControlCmd& control_msg)
{
  if (control_msg.index_length != control_msg.angles_length) return;
  for (unsigned int i = 0; i < control_msg.index_length; i++) {
    ServoData& s = servo_handler_.getOneServo(control_msg.index[i]);
    if(s == servo_handler_.getOneServo(0)){ 
      nh_->logerror("Invalid Servo ID!");
      return;
    }
    int32_t goal_pos = static_cast<int32_t>(control_msg.angles[i]);
    s.setGoalPosition(goal_pos);
    if (! s.torque_enable_) {
      s.torque_enable_ = true;
      servo_handler_.setTorque(i);
    }
  }
}

void DirectServo::servoTorqueControlCallback(const spinal::ServoTorqueCmd& control_msg)
{
  if (control_msg.index_length != control_msg.torque_enable_length) return;
  for (unsigned int i = 0; i < control_msg.index_length; i++) {
    ServoData& s = servo_handler_.getServo()[control_msg.index[i]];
    if (! s.torque_enable_) {
      s.torque_enable_ = (control_msg.torque_enable[i] != 0) ? true : false;
      servo_handler_.setTorque(i);
    }
  }
}
