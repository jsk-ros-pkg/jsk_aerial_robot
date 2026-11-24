/**
******************************************************************************
* File Name          : servo.cpp
* Description        : universal servo control interface for Spinal
* Author             : J.Sugihara (2024/3/1)
 ------------------------------------------------------------------*/


#include "servo.h"

#define  SERVO_PUB_INTERVAL 20 // 50Hz
#define SERVO_TORQUE_PUB_INTERVAL  1000 // 1Hz

bool DirectServo::init(UART_HandleTypeDef* huart,  ros::NodeHandle* nh, osMutexId* mutex = NULL) //TODO: support encoder
{
  /*setup pin configuration*/
#if !STM32H7_V2
#ifdef STM32H7
  uint32_t raw_baudrate = huart->Init.BaudRate;
  HAL_UART_DeInit(huart);
  huart->Init.BaudRate = 1000000;
  HAL_UART_Init(huart);
#endif
#endif

  /* initialize */
  servo_handler_.init(huart, mutex);
  unsigned int actual_servo_num = servo_handler_.getServoNum();

  if (actual_servo_num == 0) {
#if !STM32H7_V2
#ifdef STM32H7
    HAL_UART_DeInit(huart);
    huart->Init.BaudRate = raw_baudrate;
    HAL_UART_Init(huart);
#endif
#endif
    connected_ = false;
    return false;
  }

  nh_ = nh;

  connected_ = true;
  return true;
}

void DirectServo::update()
{
  servo_handler_.update();
}

void DirectServo::torqueEnable(const std::map<uint8_t, float>& servo_map)
{
  for (auto servo : servo_map)
    {
      JointProf joint_prof = joint_profiles_[servo.first];
      uint8_t index = servo.first;
      if(index >= servo_handler_.getServoNum())
        {
          nh_->logerror("Invalid Servo ID!");
          return;
        }
      ServoData& s = servo_handler_.getServo()[index];
      if(servo.second && !s.torque_enable_){
        s.torque_enable_ = true;
        servo_handler_.setTorque(index);
      }
      else if(!servo.second && s.torque_enable_){
        s.torque_enable_ = false;
        servo_handler_.setTorque(index);
      }     
    }
}

void DirectServo::setGoalAngle(const std::map<uint8_t, float>& servo_map, uint8_t value_type)
{
  for (auto servo : servo_map)
    {
      JointProf joint_prof = joint_profiles_[servo.first];
      int32_t goal_pos;
      if(value_type == ValueType::BIT){
        goal_pos = static_cast<int32_t>(servo.second);
      }else if(value_type == ValueType::RADIAN){
        goal_pos = static_cast<int32_t>(servo.second*joint_prof.angle_sgn/joint_prof.angle_scale + joint_prof.zero_point_offset);
      }

      uint8_t index = servo.first;
      if(index >= servo_handler_.getServoNum())
        {
          nh_->logerror("Invalid Servo ID!");
          return;
        }
      ServoData& s = servo_handler_.getServo()[index];
      s.setGoalPosition(goal_pos);
      if (! s.torque_enable_) {
        s.torque_enable_ = true;
        servo_handler_.setTorque(index);
      }
      
    }
}
