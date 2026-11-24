/*
 * servo_manager.cpp
 *
 *  Created on: 2026/3/20
 *      Author: K.Sugihara
 */

#include "servo_manager.h"

constexpr uint32_t SERVO_PUB_INTERVAL = 20;           //[ms]
constexpr uint32_t SERVO_TORQUE_PUB_INTERVAL = 1000;  //[ms]

void ServoManager::init(ros::NodeHandle* nh)
{
  nh_ = nh;

  board_info_res_.boards_length = 1 + (use_spine_ ? Spine::getSlaveNum() : 0);
  board_info_res_.boards = new spinal::BoardInfo[board_info_res_.boards_length];

  // count total number of servo connected to spinal (board id 0)
  int spinal_servo_num = 0;
  for (auto servo_handler : servo_handlers_)
    {
      if (servo_handler != NULL)
        {
          spinal_servo_num += servo_handler->getServoHnadler().getServoNum();
        }
    }
  spinal::BoardInfo& board = board_info_res_.boards[0];
  board.servos_length = spinal_servo_num;
  board.servos = new spinal::ServoInfo[board.servos_length];

  if (use_spine_)
    {
      for (unsigned int i = 0; i < Spine::getSlaveNum(); i++)
        {
          spinal::BoardInfo& board = board_info_res_.boards[1 + i];
          board.imu_send_data_flag = Spine::getNeuron().at(i).can_imu_.getSendDataFlag() ? 1 : 0;
          board.dynamixel_ttl_rs485_mixed = Spine::getNeuron().at(i).can_servo_.getDynamixelTTLRS485Mixed() ? 1 : 0;
          board.slave_id = Spine::getNeuron().at(i).getSlaveId();
          board.servos_length = Spine::getNeuron().at(i).can_servo_.servo_.size();
          board.servos = new spinal::ServoInfo[board.servos_length];
        }
    }
  nh_->advertiseService(board_config_srv_);
  nh_->advertiseService(board_info_srv_);

  if (servo_num_ > 0)
  {
    nh_->advertise(servo_state_pub_);
    nh_->advertise(servo_torque_state_pub_);
    nh_->subscribe(servo_position_sub_);
    nh_->subscribe(servo_current_sub_);
    nh_->subscribe(servo_torque_ctrl_sub_);
    nh_->subscribe(joint_profiles_sub_);

    servo_state_msg_.servos_length = servo_num_;
    servo_state_msg_.servos = new spinal::ServoState[servo_num_];
    servo_torque_state_msg_.torque_enable_length = servo_num_;
    servo_torque_state_msg_.torque_enable = new uint8_t[servo_num_];

    servo_last_pub_time_ = 0;
    servo_torque_last_pub_time_ = 0;
  }
}

void ServoManager::update()
{
  servoPublish();
}

void ServoManager::addSpineServo()
{
  int servo_num = Spine::getServoNum();

  for (int i = 0; i < servo_num; i++)
  {
    servo_index_to_servo_handler_[i + servo_num_] = NULL;
    servo_index_in_each_handle_[i + servo_num_] = i;
  }

  servo_handlers_.push_back(NULL);  // spine servo doesn't have direct handler, we set NULL as a placeholder
  servo_num_ += servo_num;
  use_spine_ = true;
}

void ServoManager::addDirectServo(DirectServo* direct_servo)
{
  int servo_num = direct_servo->getServoHnadler().getServoNum();

  for (int i = 0; i < servo_num; i++)
  {
    servo_index_to_servo_handler_[i + servo_num_] = direct_servo;
    servo_index_in_each_handle_[i + servo_num_] = i;
  }

  servo_handlers_.push_back(direct_servo);
  servo_num_ += servo_num;
}

void ServoManager::servoPublish()
{
  if (servo_num_ == 0)
    return;

  uint32_t now_time = HAL_GetTick();
  if (now_time - servo_last_pub_time_ >= SERVO_PUB_INTERVAL)  // servo states
  {
    servo_state_msg_.stamp = nh_->now();

    for (int i = 0; i < servo_num_; i++)
    {
      DirectServo* servo_handler = servo_index_to_servo_handler_[i];
      uint8_t servo_index_in_handle = servo_index_in_each_handle_[i];

      if (servo_handler != NULL)  // direct servo
      {
        const ServoData& s = servo_handler->getServoHnadler().getServo()[servo_index_in_handle];
        if (s.send_data_flag_ != 0)
        {
          spinal::ServoState servo;
          servo.index = i;
          servo.angle = s.present_position_;
          servo.temp = s.present_temp_;
          servo.load = s.present_current_;
          servo.error = s.hardware_error_status_;
          servo_state_msg_.servos[i] = servo;
        }
      }
      else  // spine servo
      {
        std::vector<std::reference_wrapper<Servo>>& servo_with_send_flag = Spine::getServoWithSendFlag();
        const Servo& s = servo_with_send_flag.at(servo_index_in_handle).get();
        if (s.getSendDataFlag())
        {
          spinal::ServoState servo;
          servo.index = i;
          servo.angle = s.getPresentPosition();
          servo.temp = s.getPresentTemperature();
          servo.load = s.getPresentCurrent();
          servo.error = s.getError();
          servo_state_msg_.servos[i] = servo;
        }
      }
    }
    servo_state_pub_.publish(&servo_state_msg_);
    servo_last_pub_time_ = now_time;
  }

  if (now_time - servo_torque_last_pub_time_ >= SERVO_TORQUE_PUB_INTERVAL)  // servo torque states
  {
    for (int i = 0; i < servo_num_; i++)
    {
      DirectServo* servo_handler = servo_index_to_servo_handler_[i];
      uint8_t servo_index_in_handle = servo_index_in_each_handle_[i];

      if (servo_handler != NULL)  // direct servo
      {
        const ServoData& s = servo_handler->getServoHnadler().getServo()[servo_index_in_handle];
        if (s.send_data_flag_ != 0)
        {
          servo_torque_state_msg_.torque_enable[i] = s.torque_enable_;
        }
      }
      else  // spine servo
      {
        std::vector<std::reference_wrapper<Servo>>& servo_with_send_flag = Spine::getServoWithSendFlag();
        const Servo& s = servo_with_send_flag.at(servo_index_in_handle).get();
        if (s.getSendDataFlag())
        {
          servo_torque_state_msg_.torque_enable[i] = s.getTorqueEnable();
        }
      }
      servo_torque_state_pub_.publish(&servo_torque_state_msg_);
      servo_torque_last_pub_time_ = now_time;
    }
  }
}

void ServoManager::servoPositionCallback(const spinal::ServoControlCmd& control_msg)
{
  if (control_msg.index_length != control_msg.angles_length)
    return;

  for (unsigned int i = 0; i < control_msg.index_length; i++)
  {
    uint8_t index = control_msg.index[i];
    uint8_t servo_index = servo_index_in_each_handle_[index];

    if (index >= servo_num_)
    {
      nh_->logerror("Invalid Servo ID!");
      return;
    }

    if (servo_index_to_servo_handler_.find(index) != servo_index_to_servo_handler_.end() &&
        servo_index_to_servo_handler_[index] != NULL)  // direct servo
    {
      DirectServo* direct_servo = servo_index_to_servo_handler_[index];
      ServoData& s = direct_servo->getServoHnadler().getServo()[servo_index];
      int32_t goal_pos = static_cast<int32_t>(control_msg.angles[i]);
      s.setGoalPosition(goal_pos);
      if (!s.torque_enable_)
      {
        s.torque_enable_ = true;
        direct_servo->getServoHnadler().setTorque(servo_index);
      }
    }
    else if (servo_index_to_servo_handler_.find(index) != servo_index_to_servo_handler_.end() &&
             servo_index_to_servo_handler_[index] == NULL)  // spine servo
    {
      if (!Spine::getServoControlFlag())
        continue;

      std::vector<std::reference_wrapper<Servo>>& spine_servos = Spine::getServo();
      spine_servos.at(servo_index).get().setGoalPosition(control_msg.angles[i]);
    }
    else
    {
      nh_->logerror("Invalid Servo ID!");
      return;
    }
  }
}

void ServoManager::servoCurrentCallback(const spinal::ServoControlCmd& control_msg)
{
  if (control_msg.index_length != control_msg.angles_length)
    return;

  for (unsigned int i = 0; i < control_msg.index_length; i++)
  {
    uint8_t index = control_msg.index[i];
    uint8_t servo_index = servo_index_in_each_handle_[index];

    if (index >= servo_num_)
    {
      nh_->logerror("Invalid Servo ID!");
      return;
    }

    if (servo_index_to_servo_handler_.find(index) != servo_index_to_servo_handler_.end() &&
        servo_index_to_servo_handler_[index] != NULL)  // direct servo
    {
      // TODO: implement current control for direct servo
    }
    else if (servo_index_to_servo_handler_.find(index) != servo_index_to_servo_handler_.end() &&
             servo_index_to_servo_handler_[index] == NULL)  // spine servo
    {
      if (!Spine::getServoControlFlag())
        continue;

      std::vector<std::reference_wrapper<Servo>>& spine_servos = Spine::getServo();
      spine_servos.at(servo_index).get().setGoalCurrent(control_msg.angles[i]);
    }
    else
    {
      nh_->logerror("Invalid Servo ID!");
      return;
    }
  }
}

void ServoManager::servoTorqueControlCallback(const spinal::ServoTorqueCmd& control_msg)
{
  if (control_msg.index_length != control_msg.torque_enable_length)
    return;

  for (unsigned int i = 0; i < control_msg.index_length; i++)
  {
    uint8_t index = control_msg.index[i];
    uint8_t servo_index = servo_index_in_each_handle_[index];

    if (index >= servo_num_)
    {
      nh_->logerror("Invalid Servo ID!");
      return;
    }

    if (servo_index_to_servo_handler_.find(index) != servo_index_to_servo_handler_.end() &&
        servo_index_to_servo_handler_[index] != NULL)  // direct servo
    {
      DirectServo* direct_servo = servo_index_to_servo_handler_[index];
      ServoData& s = direct_servo->getServoHnadler().getServo()[servo_index];
      s.torque_enable_ = (control_msg.torque_enable[i] != 0) ? true : false;
      direct_servo->getServoHnadler().setTorqueFromPresetnPos(servo_index);
    }
    else if (servo_index_to_servo_handler_.find(index) != servo_index_to_servo_handler_.end() &&
             servo_index_to_servo_handler_[index] == NULL)  // spine servo
    {
      std::vector<std::reference_wrapper<Servo>>& spine_servos = Spine::getServo();
      spine_servos.at(servo_index).get().setTorqueEnable((control_msg.torque_enable[i] != 0) ? true : false);

      /* update the target angle */
      if (spine_servos.at(servo_index).get().getSendDataFlag())
      {
        spine_servos.at(servo_index).get().setGoalPosition(spine_servos.at(servo_index).get().getPresentPosition());
      }
    }
    else
    {
      nh_->logerror("Invalid Servo ID!");
      return;
    }
  }
}

void ServoManager::jointProfilesCallback(const spinal::JointProfiles& joint_prof_msg)
{
  for (unsigned int i = 0; i < joint_prof_msg.joints_length; i++)
  {
    uint8_t index = joint_prof_msg.joints[i].servo_id;
    uint8_t servo_index = servo_index_in_each_handle_[index];

    if (index >= servo_num_)
    {
      nh_->logerror("Invalid Servo ID!");
      return;
    }

    if (servo_index_to_servo_handler_.find(index) != servo_index_to_servo_handler_.end() &&
        servo_index_to_servo_handler_[index] != NULL)  // direct servo
    {
      DirectServo* direct_servo = servo_index_to_servo_handler_[index];
      direct_servo->joint_profiles_[servo_index].servo_id = servo_index;
      direct_servo->joint_profiles_[servo_index].angle_sgn = joint_prof_msg.joints[i].angle_sgn;
      direct_servo->joint_profiles_[servo_index].angle_scale = joint_prof_msg.joints[i].angle_scale;
      direct_servo->joint_profiles_[servo_index].zero_point_offset = joint_prof_msg.joints[i].zero_point_offset;
    }
    else if (servo_index_to_servo_handler_.find(index) != servo_index_to_servo_handler_.end() &&
             servo_index_to_servo_handler_[index] == NULL)  // spine servo
    {
      // TODO: implement joint profile setting for spine servo
    }
    else
    {
      nh_->logerror("Invalid Servo ID!");
      return;
    }
  }
}

void ServoManager::boardInfoCallback(const spinal::GetBoardInfo::Request& req, spinal::GetBoardInfo::Response& res)
{
  for (uint8_t i = 0; i < servo_num_; i++)
  {
    DirectServo* servo_handler = servo_index_to_servo_handler_[i];
    uint8_t servo_index_in_handle = servo_index_in_each_handle_[i];

    if (servo_handler != NULL)  // direct servo
    {
      spinal::BoardInfo& board = board_info_res_.boards[0];
      board.imu_send_data_flag = 1;
#if DYNAMIXEL
      board.dynamixel_ttl_rs485_mixed = servo_handler->getServoHnadler().getTTLRS485Mixed() ? 1 : 0;
#endif
      board.slave_id = 0;
      const ServoData& s = servo_handler->getServoHnadler().getServo()[servo_index_in_handle];
      board.servos[servo_index_in_handle].id = s.id_;
      board.servos[servo_index_in_handle].p_gain = s.p_gain_;
      board.servos[servo_index_in_handle].i_gain = s.i_gain_;
      board.servos[servo_index_in_handle].d_gain = s.d_gain_;
      board.servos[servo_index_in_handle].profile_velocity = s.profile_velocity_;
      board.servos[servo_index_in_handle].current_limit = s.current_limit_;
      board.servos[servo_index_in_handle].send_data_flag = s.send_data_flag_ ? 1 : 0;
      board.servos[servo_index_in_handle].external_encoder_flag = s.external_encoder_flag_ ? 1 : 0;
      board.servos[servo_index_in_handle].joint_resolution = s.joint_resolution_;
      board.servos[servo_index_in_handle].servo_resolution = s.servo_resolution_;
    }
    else  // spine servo
    {
      for (uint8_t j = 0; j < Spine::getSlaveNum(); j++)
      {
        spinal::BoardInfo& board = board_info_res_.boards[1 + j];
        board.imu_send_data_flag = Spine::getNeuron().at(j).can_imu_.getSendDataFlag() ? 1 : 0;
        board.dynamixel_ttl_rs485_mixed = Spine::getNeuron().at(j).can_servo_.getDynamixelTTLRS485Mixed() ? 1 : 0;
        board.slave_id = Spine::getNeuron().at(j).getSlaveId();

        for (uint8_t k = 0; k < board.servos_length; k++)
        {
          Servo& s = Spine::getNeuron().at(j).can_servo_.servo_.at(k);
          board.servos[k].id = s.getId();
          board.servos[k].p_gain = s.getPGain();
          board.servos[k].i_gain = s.getIGain();
          board.servos[k].d_gain = s.getDGain();
          board.servos[k].profile_velocity = s.getProfileVelocity();
          board.servos[k].current_limit = s.getCurrentLimit();
          board.servos[k].send_data_flag = s.getSendDataFlag() ? 1 : 0;
          board.servos[k].external_encoder_flag = s.getExternalEncoderFlag() ? 1 : 0;
          board.servos[k].joint_resolution = s.getJointResolution();
          board.servos[k].servo_resolution = s.getServoResolution();
        }
      }
    }
  }
  res = board_info_res_;
}

void ServoManager::boardConfigCallback(const spinal::SetBoardConfig::Request& req,
                                       spinal::SetBoardConfig::Response& res)
{
  uint8_t command = req.command;
  uint8_t slave_id = static_cast<uint8_t>(req.data[0]);
  if (slave_id == 0)  // spinal
  {
    DirectServo* servo_handler = servo_index_to_servo_handler_[req.data[1]];
    uint8_t servo_index_in_handle = servo_index_in_each_handle_[req.data[1]];
    ServoData& s = servo_handler->getServoHnadler().getServo()[servo_index_in_handle];

    switch (command)
    {
      case spinal::SetBoardConfig::Request::SET_SERVO_HOMING_OFFSET: {
        if (!s.torque_enable_)
        {
          int32_t calib_value = req.data[2];
          s.calib_value_ = calib_value;
          servo_handler->getServoHnadler().setHomingOffset(servo_index_in_handle);
          res.success = true;
        }
        else
        {
          nh_->logerror("Cannot set homing offset during torque on state.");
          res.success = false;
        }
        break;
      }
      case spinal::SetBoardConfig::Request::SET_SERVO_PID_GAIN: {
        if (!s.torque_enable_)
        {
          s.p_gain_ = req.data[2];
          s.i_gain_ = req.data[3];
          s.d_gain_ = req.data[4];
          servo_handler->getServoHnadler().setPositionGains(servo_index_in_handle);
          FlashMemory::erase();
          FlashMemory::write();
          res.success = true;
        }
        else
        {
          nh_->logerror("Cannot set PID gains during torque on state.");
          res.success = false;
        }
        break;
      }
      case spinal::SetBoardConfig::Request::SET_SERVO_PROFILE_VEL: {
        s.profile_velocity_ = req.data[2];
        servo_handler->getServoHnadler().setProfileVelocity(servo_index_in_handle);
        FlashMemory::erase();
        FlashMemory::write();
        res.success = true;
        break;
      }
      case spinal::SetBoardConfig::Request::SET_SERVO_SEND_DATA_FLAG: {
        s.send_data_flag_ = req.data[2];
        FlashMemory::erase();
        FlashMemory::write();
        res.success = true;
        break;
      }
      case spinal::SetBoardConfig::Request::SET_SERVO_CURRENT_LIMIT: {
        s.current_limit_ = req.data[2];
        servo_handler->getServoHnadler().setCurrentLimit(servo_index_in_handle);
        res.success = true;
        break;
      }
      case spinal::SetBoardConfig::Request::SET_DYNAMIXEL_TTL_RS485_MIXED: {
        servo_handler->getServoHnadler().setTTLRS485Mixed(req.data[2]);
        FlashMemory::erase();
        FlashMemory::write();
        res.success = true;
        break;
      }
      case spinal::SetBoardConfig::Request::SET_SERVO_EXTERNAL_ENCODER_FLAG: {
        if (!s.torque_enable_)
        {
          s.external_encoder_flag_ = req.data[2];
          s.first_get_pos_flag_ = true;
          if (!s.external_encoder_flag_)  // if use the servo internal encoder, we directly output the encoder value
                                          // without scaling by resolution_ratio.
          {
            s.servo_resolution_ = 1;
            s.joint_resolution_ = 1;
            s.resolution_ratio_ = 1;
          }
          FlashMemory::erase();
          FlashMemory::write();
          res.success = true;
        }
        else
        {
          nh_->logerror("Cannot set ex encoder flag during torque on state.");
          res.success = false;
        }
        break;
      }
      case spinal::SetBoardConfig::Request::SET_SERVO_RESOLUTION_RATIO: {
        if (!s.torque_enable_)
        {
          s.joint_resolution_ = req.data[2];
          s.servo_resolution_ = req.data[3];
          s.hardware_error_status_ &= ((1 << RESOLUTION_RATIO_ERROR) - 1);  // 0b00111111: reset

          if (s.servo_resolution_ == 65535 || s.joint_resolution_ == 65535)
          {
            s.hardware_error_status_ |= (1 << RESOLUTION_RATIO_ERROR);  // 0b01000000;
            s.resolution_ratio_ = 1;
          }
          else
          {
            s.resolution_ratio_ = (float)s.servo_resolution_ / (float)s.joint_resolution_;
            s.first_get_pos_flag_ = true;
            FlashMemory::erase();
            FlashMemory::write();
          }
          res.success = true;
        }
        else
        {
          nh_->logerror("Cannot set resolution rate during torque on state.");
          res.success = false;
        }
        break;
      }
      default:
        break;
    }
  }
  else  // neuron
  {
    Spine::setCanTxIdleStartTime(HAL_GetTick());
    Spine::getCANInitializer().configDevice(req);
  }
  res.success = true;
}
