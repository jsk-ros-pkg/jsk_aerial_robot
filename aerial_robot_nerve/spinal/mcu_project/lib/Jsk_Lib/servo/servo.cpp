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
  nh_->advertiseService(servo_config_srv_);

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
          servo.error = s.hardware_error_status_;
          servo_state_msg_.servos[i] = servo;
        }
      }
      servo_state_pub_.publish(&servo_state_msg_);
      servo_last_pub_time_ = now_time;
    }
}

void DirectServo::setGoalAngle(const std::map<uint8_t, float>& servo_map)
{
  for (auto servo : servo_map)
    {
      uint16_t servo_id = servo.first;
      float goal_angle = servo.second;

      ServoData& s = servo_handler_.getOneServo(servo_id);
      uint8_t index = servo_handler_.getServoIndex(servo_id);
      if(s == servo_handler_.getOneServo(0)){ 
        nh_->logerror("Invalid Servo ID!");
        return;
      }

      uint32_t goal_pos = DirectServo::rad2Pos(goal_angle, s.angle_scale_, s.zero_point_offset_ );
      s.setGoalPosition(goal_pos);
      if (! s.torque_enable_) {
        s.torque_enable_ = true;
        servo_handler_.setTorque(index);
      }
      
    }
}

void DirectServo::servoControlCallback(const spinal::ServoControlCmd& control_msg)
{
  if (control_msg.index_length != control_msg.angles_length) return;
  for (unsigned int i = 0; i < control_msg.index_length; i++) {
    ServoData& s = servo_handler_.getOneServo(control_msg.index[i]);
    uint8_t index = servo_handler_.getServoIndex(control_msg.index[i]);
    if(s == servo_handler_.getOneServo(0)){ 
      nh_->logerror("Invalid Servo ID!");
      return;
    }
    int32_t goal_pos = static_cast<int32_t>(control_msg.angles[i]);
    s.setGoalPosition(goal_pos);
    if (! s.torque_enable_) {
      s.torque_enable_ = true;
      servo_handler_.setTorque(index);
    }
  }
}

void DirectServo::servoTorqueControlCallback(const spinal::ServoTorqueCmd& control_msg)
{
  if (control_msg.index_length != control_msg.torque_enable_length) return;
  for (unsigned int i = 0; i < control_msg.index_length; i++) {
    ServoData& s = servo_handler_.getOneServo(control_msg.index[i]);
    uint8_t index = servo_handler_.getServoIndex(control_msg.index[i]);
    if(s == servo_handler_.getOneServo(0)){ 
      nh_->logerror("Invalid Servo ID!");
      return;
    }
    if (! s.torque_enable_) {
      s.torque_enable_ = (control_msg.torque_enable[i] != 0) ? true : false;
      servo_handler_.setTorque(index);
    }
  }
}

void DirectServo::servoConfigCallback(const spinal::SetDirectServoConfig::Request& req, spinal::SetDirectServoConfig::Response& res)
{

  uint8_t command = req.command;

  /* special case : data[0] is flag value */
  if(command == spinal::SetDirectServoConfig::Request::SET_DYNAMIXEL_TTL_RS485_MIXED)
    {
      servo_handler_.setTTLRS485Mixed(req.data[0]);
      FlashMemory::erase();
      FlashMemory::write();
      res.success = true;
      return;
    }

  uint8_t servo_index = req.data[0];
  ServoData& s = servo_handler_.getOneServo(servo_index);
  if(s == servo_handler_.getOneServo(0)){ 
    nh_->logerror("Invalid Servo ID!");
    return;
  }

  switch (command) {
  case spinal::SetDirectServoConfig::Request::SET_SERVO_HOMING_OFFSET:
    {
      int32_t calib_value = req.data[1];
      s.calib_value_ = calib_value;
      servo_handler_.setHomingOffset(servo_index);
      break;
    }
  case spinal::SetDirectServoConfig::Request::SET_SERVO_PID_GAIN:
    {
      s.p_gain_ = req.data[1];
      s.i_gain_ = req.data[2];
      s.d_gain_ = req.data[3];
      servo_handler_.setPositionGains(servo_index);
      FlashMemory::erase();
      FlashMemory::write();
      break;
    }
  case spinal::SetDirectServoConfig::Request::SET_SERVO_PROFILE_VEL:
    {
      s.profile_velocity_ = req.data[1];
      servo_handler_.setProfileVelocity(servo_index);
      FlashMemory::erase();
      FlashMemory::write();
      break;
    }
  case spinal::SetDirectServoConfig::Request::SET_SERVO_SEND_DATA_FLAG:
    {
      s.send_data_flag_ = req.data[1];
      FlashMemory::erase();
      FlashMemory::write();
      break;
    }
  case spinal::SetDirectServoConfig::Request::SET_SERVO_CURRENT_LIMIT:
    {
      s.current_limit_ = req.data[1];
      servo_handler_.setCurrentLimit(servo_index);
      break;
    }
  case spinal::SetDirectServoConfig::Request::SET_SERVO_EXTERNAL_ENCODER_FLAG:
    {
      if(!s.torque_enable_){
        s.external_encoder_flag_ = req.data[1];
        s.first_get_pos_flag_ = true;
        if(!s.external_encoder_flag_)
          { // if use the servo internal encoder, we directly output the encoder value without scaling by resolution_ratio.
            s.servo_resolution_ = 1;
            s.joint_resolution_ = 1;
            s.resolution_ratio_ = 1;
          }
        FlashMemory::erase();
        FlashMemory::write();
      }
      break;
    }
  case spinal::SetDirectServoConfig::Request::SET_SERVO_RESOLUTION_RATIO:
    {
      if(!s.torque_enable_){
        s.joint_resolution_ = req.data[1];
        s.servo_resolution_ = req.data[2];
        s.hardware_error_status_ &= ((1 << RESOLUTION_RATIO_ERROR) - 1); // 0b00111111: reset

        if(s.servo_resolution_ == 65535 || s.joint_resolution_ == 65535){
          s.hardware_error_status_ |= (1 << RESOLUTION_RATIO_ERROR);  // 0b01000000;
          s.resolution_ratio_ = 1;
        }
        else{
          s.resolution_ratio_ = (float)s.servo_resolution_ / (float)s.joint_resolution_;
          s.first_get_pos_flag_ = true;
          FlashMemory::erase();
          FlashMemory::write();
        }
      }
      break;
    }
  default:
    break;
  }
  res.success = true;
}

