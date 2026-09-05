/**
******************************************************************************
* File Name          : servo.cpp
* Description        : universal servo control interface for Spinal
* Author             : J.Sugihara (2024/3/1)
 ------------------------------------------------------------------*/


#include "servo.h"

#define SERVO_TORQUE_PUB_INTERVAL  1000 // 1Hz
#define SERVO_COMM_STATUS_PUB_INTERVAL 1000 // 1Hz

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
  nh_->subscribe(servo_ctrl_sub_);
  nh_->subscribe(servo_torque_ctrl_sub_);
  nh_->subscribe(joint_profiles_sub_);
  nh_->advertise(servo_state_pub_);
  nh_->advertise(servo_torque_state_pub_);
  nh_->advertise(servo_comm_status_pub_);
  nh_->advertiseService(servo_config_srv_);
  nh_->advertiseService(board_info_srv_);

  servo_state_msg_.servos_length = actual_servo_num;
  servo_state_msg_.servos = new spinal::ServoState[actual_servo_num];
  servo_torque_state_msg_.torque_enable_length = actual_servo_num;
  servo_torque_state_msg_.torque_enable = new uint8_t[actual_servo_num];
  servo_comm_status_msg_.servo_ids_length = actual_servo_num;
  servo_comm_status_msg_.servo_ids = new uint8_t[actual_servo_num];
  servo_comm_status_msg_.cur_vel_pos_update_count_length = actual_servo_num;
  servo_comm_status_msg_.cur_vel_pos_update_count = new uint32_t[actual_servo_num];
  servo_comm_status_msg_.goal_write_count_length = actual_servo_num;
  servo_comm_status_msg_.goal_write_count = new uint32_t[actual_servo_num];

  servo_last_pub_time_ = 0;
  servo_torque_last_pub_time_ = 0;
  servo_comm_status_last_pub_time_ = 0;
  servo_comm_status_initialized_ = false;
  last_bus_map_valid_ = false;

  board_info_res_.boards_length = 1;
  board_info_res_.boards = new spinal::BoardInfo[1];
  board_info_res_.boards[0].servos_length = servo_handler_.getServoNum();
  board_info_res_.boards[0].servos = new spinal::ServoInfo[servo_handler_.getServoNum()];

  connected_ = true;
  return true;
}

void DirectServo::update()
{
  servo_handler_.update();
  sendData(true);
}

void DirectServo::sendData(bool flag_send_asap)
{
  uint32_t now_time = HAL_GetTick();

  const uint16_t feedback_frequency =
    servo_handler_.getServoFeedbackFrequency();
  const uint32_t publish_interval =
    feedback_frequency == 0 ? UINT32_MAX :
    (1000U + feedback_frequency - 1U) / feedback_frequency;
  if ((!flag_send_asap || servo_handler_.getROSCommFlag()) &&
      now_time - servo_last_pub_time_ >= publish_interval)
    {
	  servo_state_msg_.stamp = nh_->now();
      for (unsigned int i = 0; i < servo_handler_.getServoNum(); i++) {
        const ServoData& s = servo_handler_.getServo()[i];
        if (s.send_data_flag_ != 0) {
          spinal::ServoState servo;
          servo.index = i;
          servo.angle = s.present_position_;
          servo.velocity = s.present_velocity;
          servo.temp = s.present_temp_;
          servo.load = s.present_current_;
          servo.error = s.hardware_error_status_;
          servo_state_msg_.servos[i] = servo;
        }
      }
      servo_state_pub_.publish(&servo_state_msg_);
      servo_last_pub_time_ = now_time;
      servo_handler_.setROSCommFlag(false);
    }

  if( now_time - servo_torque_last_pub_time_ >= SERVO_TORQUE_PUB_INTERVAL)
    {
      for (unsigned int i = 0; i < servo_handler_.getServoNum(); i++) {
        const ServoData& s = servo_handler_.getServo()[i];
        if (s.send_data_flag_ != 0) {
          servo_torque_state_msg_.torque_enable[i] = s.torque_enable_;
        }
      }
      servo_torque_state_pub_.publish(&servo_torque_state_msg_);
      servo_torque_last_pub_time_= now_time;
    }  

  const bool bus_map_valid = servo_handler_.busMapValid();
  const bool status_changed =
    !servo_comm_status_initialized_ ||
    bus_map_valid != last_bus_map_valid_;
  if (status_changed ||
      now_time - servo_comm_status_last_pub_time_ >=
        SERVO_COMM_STATUS_PUB_INTERVAL)
    {
      const ServoCommCounters& counters = servo_handler_.getCommCounters();
      servo_comm_status_msg_.stamp = nh_->now();
      servo_comm_status_msg_.transaction_success_count = counters.transaction_success_count;
      servo_comm_status_msg_.timeout_count = counters.timeout_count;
      servo_comm_status_msg_.crc_error_count = counters.crc_error_count;
      servo_comm_status_msg_.invalid_packet_count = counters.invalid_packet_count;
      servo_comm_status_msg_.recovery_count = counters.recovery_count;
      servo_comm_status_msg_.deadline_miss_count = counters.deadline_miss_count;
      servo_comm_status_msg_.queue_overflow_count = counters.queue_overflow_count;
      servo_comm_status_msg_.unexpected_completion_count = counters.unexpected_completion_count;
      servo_comm_status_msg_.watchdog_timeout_count =
        counters.watchdog_timeout_count;
      servo_comm_status_msg_.configuration_error_count =
        counters.configuration_error_count;
      servo_comm_status_msg_.queue_high_water_mark = counters.queue_high_water_mark;
      servo_comm_status_msg_.bus_map_valid = bus_map_valid;
      for (unsigned int i = 0; i < servo_handler_.getServoNum(); ++i) {
        const ServoData& servo = servo_handler_.getServo()[i];
        servo_comm_status_msg_.servo_ids[i] = servo.id_;
        servo_comm_status_msg_.cur_vel_pos_update_count[i] =
          servo.cur_vel_pos_update_count_;
        servo_comm_status_msg_.goal_write_count[i] = servo.goal_write_count_;
      }
      servo_comm_status_pub_.publish(&servo_comm_status_msg_);
      servo_comm_status_last_pub_time_ = now_time;
      servo_comm_status_initialized_ = true;
      last_bus_map_valid_ = bus_map_valid;
    }
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

void DirectServo::servoControlCallback(const spinal::ServoControlCmd& control_msg)
{
  const uint32_t count = control_msg.index_length;
  const bool has_position = control_msg.position_length != 0;
  const bool has_velocity = control_msg.velocity_length != 0;
  const bool has_current = control_msg.current_length != 0;
  if (count == 0 || (!has_position && !has_velocity && !has_current) ||
      (has_position && control_msg.position_length != count) ||
      (has_velocity && control_msg.velocity_length != count) ||
      (has_current && control_msg.current_length != count)) {
    nh_->logerror("Invalid ServoControlCmd array lengths");
    return;
  }

  for (uint32_t i = 0; i < count; ++i) {
    if (control_msg.index[i] >= servo_handler_.getServoNum()) {
      nh_->logerror("Invalid servo index in ServoControlCmd");
      return;
    }
  }

  for (uint32_t i = 0; i < count; ++i) {
    const uint8_t servo_index = control_msg.index[i];
    if (!servo_handler_.setGoalTargets(
          servo_index,
          has_position, has_position ? control_msg.position[i] : 0,
          has_velocity, has_velocity ? control_msg.velocity[i] : 0,
          has_current, has_current ? control_msg.current[i] : 0)) {
      nh_->logerror("Failed to update ServoControlCmd cache");
      return;
    }
    ServoData& servo = servo_handler_.getServo()[servo_index];
    if (!servo.torque_enable_) {
      servo.torque_enable_ = true;
      servo_handler_.setTorque(servo_index);
    }
  }
}

void DirectServo::servoTorqueControlCallback(const spinal::ServoTorqueCmd& control_msg)
{
  if (control_msg.index_length != control_msg.torque_enable_length) return;
  for (unsigned int i = 0; i < control_msg.index_length; i++) {
    uint8_t index = control_msg.index[i];
    if(index >= servo_handler_.getServoNum())
      {
        nh_->logerror("Invalid Servo ID!");
        return;
      }
    ServoData& s = servo_handler_.getServo()[index];
    s.torque_enable_ = control_msg.torque_enable[i] != 0;
    servo_handler_.setTorqueFromPresetnPos(index);

  }
}

void DirectServo::servoConfigCallback(const spinal::SetDirectServoConfig::Request& req, spinal::SetDirectServoConfig::Response& res)
{
  res.success = false;
  res.data_length = 0;
  if (req.data_length > SERVO_SERVICE_DATA_CAPACITY) {
    return;
  }

  res.success = servo_handler_.requestConfig(
    req.command, req.data, static_cast<uint8_t>(req.data_length),
    SERVO_SERVICE_TIMEOUT_MS) == SERVO_RESULT_OK;
}

void DirectServo::boardInfoCallback(const spinal::GetBoardInfo::Request& req, spinal::GetBoardInfo::Response& res)
{
  //TODO: Bad implementation. This features should not be located in servo interface.
  spinal::BoardInfo& board = board_info_res_.boards[0];
  board.imu_send_data_flag = 1;
#if DYNAMIXEL
  board.dynamixel_ttl_rs485_mixed = servo_handler_.getTTLRS485Mixed();
  board.servo_feedback_frequency =
    servo_handler_.getServoFeedbackFrequency();
  board.servo_target_watchdog_timeout =
    static_cast<float>(servo_handler_.getTargetWatchdogTimeoutMs()) /
    1000.0f;
#endif
  board.slave_id = 0;
  for (unsigned int i = 0; i < servo_handler_.getServoNum(); i++) {
    const ServoData& s = servo_handler_.getServo()[i];
    board.servos[i].id = s.id_;
    board.servos[i].configured = s.configured();
    board.servos[i].bus_type = s.bus_type_;
    board.servos[i].mode = s.mode_;
    board.servos[i].servo_mode = s.operating_mode_;
    board.servos[i].kp_integer = s.kp_integer_;
    board.servos[i].kp_decimal = s.kp_decimal_;
    board.servos[i].kd_integer = s.kd_integer_;
    board.servos[i].kd_decimal = s.kd_decimal_;
    board.servos[i].p_gain = s.p_gain_;
    board.servos[i].i_gain = s.i_gain_;
    board.servos[i].d_gain = s.d_gain_;
    board.servos[i].profile_velocity = s.profile_velocity_;
    board.servos[i].current_limit = s.current_limit_;
    board.servos[i].send_data_flag = s.send_data_flag_;
    board.servos[i].external_encoder_flag = s.external_encoder_flag_;
    board.servos[i].joint_resolution = s.joint_resolution_;
    board.servos[i].servo_resolution = s.servo_resolution_;
  }
  res = board_info_res_;
}

void DirectServo::jointProfilesCallback(const spinal::JointProfiles& joint_prof_msg)
{
  for(unsigned int i = 0; i  < joint_prof_msg.joints_length; i++){
    joint_profiles_[i].servo_id = joint_prof_msg.joints[i].servo_id;
    joint_profiles_[i].angle_sgn = joint_prof_msg.joints[i].angle_sgn;
    joint_profiles_[i].angle_scale = joint_prof_msg.joints[i].angle_scale;
    joint_profiles_[i].zero_point_offset = joint_prof_msg.joints[i].zero_point_offset;
  }
}
