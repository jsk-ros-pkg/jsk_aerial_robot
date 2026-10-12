/**
******************************************************************************
* File Name          : servo.cpp
* Description        : universal servo control interface for Spinal
* Author             : J.Sugihara (2024/3/1)
 ------------------------------------------------------------------*/


#include "servo.h"

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
  nh_->subscribe(joint_profiles_sub_);

  connected_ = true;
  return true;
}

bool DirectServo::createRTOSObjects()
{
#if DYNAMIXEL
  return servo_handler_.createRTOSObjects();
#else
  return true;
#endif
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

bool DirectServo::setGoalTargets(
  uint8_t index,
  bool has_position, int32_t position,
  bool has_velocity, int32_t velocity,
  bool has_current, int16_t current)
{
  if (!servo_handler_.setGoalTargets(
        index, has_position, position, has_velocity, velocity,
        has_current, current)) {
    return false;
  }
  ServoData& servo = servo_handler_.getServo()[index];
  if (!servo.torque_enable_) {
    servo.torque_enable_ = true;
    servo_handler_.setTorque(index);
  }
  return true;
}

bool DirectServo::setTorqueEnable(uint8_t index, bool enable)
{
  if (index >= servo_handler_.getServoNum()) return false;
  ServoData& servo = servo_handler_.getServo()[index];
  servo.torque_enable_ = enable;
  servo_handler_.setTorqueFromPresetnPos(index);
  return true;
}

bool DirectServo::requestConfig(
  uint8_t command, const int32_t* data, uint8_t data_length)
{
  if (!connected_ || data_length > SERVO_SERVICE_DATA_CAPACITY) return false;
  return servo_handler_.requestConfig(
    command, data, data_length, SERVO_SERVICE_TIMEOUT_MS) == SERVO_RESULT_OK;
}

bool DirectServo::requestConfigByIndex(
  uint8_t servo_index, uint8_t command,
  const int32_t* data, uint8_t data_length)
{
  if (servo_index >= servo_handler_.getServoNum() ||
      data_length >= SERVO_SERVICE_DATA_CAPACITY ||
      (data_length > 0 && data == nullptr)) return false;

  int32_t driver_data[SERVO_SERVICE_DATA_CAPACITY] = {};
  driver_data[0] = servo_handler_.getServo()[servo_index].id_;
  for (uint8_t i = 0; i < data_length; ++i) {
    driver_data[i + 1] = data[i];
  }
  return requestConfig(command, driver_data, data_length + 1);
}

DirectServoBoardSettings DirectServo::getBoardSettings() const
{
  DirectServoBoardSettings settings;
  settings.servo_feedback_frequency =
    servo_handler_.getServoFeedbackFrequency();
#if DYNAMIXEL
  settings.dynamixel_ttl_rs485_mixed =
    servo_handler_.getTTLRS485Mixed() ? 1 : 0;
  settings.servo_target_watchdog_timeout_ms =
    servo_handler_.getTargetWatchdogTimeoutMs();
#endif
  return settings;
}

void DirectServo::fillBoardInfo(spinal::BoardInfo& board) const
{
  const DirectServoBoardSettings settings = getBoardSettings();
  board.imu_send_data_flag = 1;
  board.dynamixel_ttl_rs485_mixed =
    settings.dynamixel_ttl_rs485_mixed;
  board.servo_pulley_skip_thresh = 0;
  board.servo_feedback_frequency =
    settings.servo_feedback_frequency;
  board.servo_target_watchdog_timeout =
    static_cast<float>(settings.servo_target_watchdog_timeout_ms) /
    1000.0f;
  board.slave_id = 0;
  for (unsigned int i = 0; i < servo_handler_.getServoNum(); i++) {
    const ServoData& s = servo_handler_.getServo()[i];
    board.servos[i].id = s.id_;
    board.servos[i].configured = s.configured();
    board.servos[i].bus_type = s.bus_type_;
    board.servos[i].mode = s.mode_;
    board.servos[i].servo_mode = s.operating_mode_;
    board.servos[i].mit_kp_integer = s.mit_kp_integer_;
    board.servos[i].mit_kp_decimal = s.mit_kp_decimal_;
    board.servos[i].mit_kd_integer = s.mit_kd_integer_;
    board.servos[i].mit_kd_decimal = s.mit_kd_decimal_;
    board.servos[i].mit_current_limit = s.mit_current_limit_;
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
