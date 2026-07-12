/**
******************************************************************************
* File Name          : servo.cpp
* Description        : interface for DJI M2006 with C610
 ------------------------------------------------------------------*/

#include "servo.h"

using namespace DJI_M2006;

float clamp(float val, float limit) { return std::min(std::max(val, -limit), limit); }

Servo::Servo()
{
  rotations_ = 0;
  last_pos_measurement_ = kCountsPerRev;

  counts_ = 0;
  rpm_ = 0;
  m_curr_ = 0;

  output_pos_ = 0;
  output_vel_ = 0;
  curr_ = 0;

  external_encoder_connected_ = false;
  external_encoder_initialized_ = false;
  external_encoder_rotations_ = 0;
  external_encoder_last_counts_ = MagEncoder::RESOLUTION;
  external_encoder_last_update_time_ = 0;
  external_output_pos_ = 0;
  external_output_vel_ = 0;

  filter_vel_ = 0;
  filter_vel_p_ = 0;

  control_mode_ = 2;

  goal_curr_ = 0;
  goal_vel_ = 0;

  v_k_p_ = 5.0; // 5.0 is good; 10.0 is oscillated for raw vel
  v_k_i_ = 0.2; // 0.2 is good; 0.5 is too strong.
  v_i_term_ = 0;


  p_k_p_ = 20.0;
  p_k_i_ = 0.0;
  p_k_d_ = 2.0;
  p_i_term_ = 0;
}

void Servo::update(uint16_t counts, int16_t rpm, int16_t m_curr)
{
  rpm_ = rpm;
  m_curr_ = m_curr;

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

  output_pos_ = counts_ / kCountsPerRad; // TODO: rectify the offset;
  output_vel_ = rpm_ / kRPMPerRadS;
  curr_ = m_curr_ / 1000.0f;

  // low pass filer
  filter_vel_p_ -= (filter_vel_p_/GYRO_LPF_FACTOR);
  filter_vel_p_ += output_vel_;
  filter_vel_ = (filter_vel_p_/GYRO_LPF_FACTOR);
}

void Servo::updateExternalEncoder(uint16_t counts, uint32_t now_ms)
{
  if (!external_encoder_initialized_) {
    external_encoder_last_counts_ = counts;
    external_encoder_last_update_time_ = now_ms;
    external_encoder_initialized_ = true;
  }

  int32_t delta = counts - external_encoder_last_counts_;
  if (delta > MagEncoder::RESOLUTION / 2) {
    external_encoder_rotations_ -= 1;
  } else if (delta < -MagEncoder::RESOLUTION / 2) {
    external_encoder_rotations_ += 1;
  }

  int32_t external_counts = external_encoder_rotations_ * MagEncoder::RESOLUTION + counts;
  uint32_t dt_ms = now_ms - external_encoder_last_update_time_;
  float next_external_output_pos = external_counts / kExternalEncoderCountsPerRad;

  if (dt_ms > 0) {
    external_output_vel_ = (next_external_output_pos - external_output_pos_) * 1000.0f / dt_ms;
  }
  external_output_pos_ = next_external_output_pos;

  external_encoder_last_counts_ = counts;
  external_encoder_last_update_time_ = now_ms;
  external_encoder_connected_ = true;
}

void Servo::setExternalEncoderConnected(bool connected)
{
  external_encoder_connected_ = connected;
  if (!connected) {
    external_encoder_initialized_ = false;
    external_encoder_last_counts_ = MagEncoder::RESOLUTION;
    external_encoder_last_update_time_ = 0;
    external_encoder_rotations_ = 0;
    external_output_vel_ = 0;
  }
}

float Servo::getFeedbackAngle() const
{
  return external_encoder_connected_ ? external_output_pos_ : output_pos_;
}

float Servo::getFeedbackVelocity() const
{
  return external_encoder_connected_ ? external_output_vel_ : output_vel_;
}

float Servo::getFeedbackCounts() const
{
  return getFeedbackAngle() * kExternalEncoderCountsPerRad;
}

void Servo::control()
{
  switch (control_mode_)
    {
    case POS_MODE:
      calcPosPid();
      break;
    case VEL_MODE:
      calcVelPid();
      break;
    case CUR_MODE:
      goal_curr_ = goal_value_;
      break;
    default:
      goal_curr_ = 0;
      break;
    }

  goal_curr_ = clamp(goal_curr_, MAX_CURRENT);
}

void Servo::calcVelPid(void)
{
  goal_vel_ = goal_value_;

  // filterd velocity will cause the oscillation, even GYRO_LPF_FACTOR = 2 is invliad
  // so use the raw value from C610 (the raw one might be already filtered in C610)
  float err = goal_vel_ - output_vel_;
  float p_term = v_k_p_ * err;
  p_term = clamp(p_term, MAX_CURRENT);
  v_i_term_ += v_k_i_ * err * SERVO_CTRL_INTERVAL;
  v_i_term_ = clamp(v_i_term_, MAX_CURRENT);

  goal_curr_ = p_term + v_i_term_;
}

void Servo::calcPosPid(void)
{
  goal_pos_ = goal_value_;

  float err = goal_pos_ - getFeedbackAngle();
  float p_term = p_k_p_ * err;
  p_term = clamp(p_term, MAX_CURRENT);
  p_i_term_ += p_k_i_ * err * SERVO_CTRL_INTERVAL;
  p_i_term_ = clamp(p_i_term_, MAX_CURRENT);
  float d_term = p_k_d_ * (-getFeedbackVelocity());
  d_term = clamp(d_term, MAX_CURRENT);

  goal_curr_ = p_term + p_i_term_ + d_term;
}

void Servo::setPidGain(uint8_t mode, float p_gain, float i_gain, float d_gain)
{
  if (mode == POS_MODE)
    {
      p_k_p_ = p_gain;
      p_k_i_ = i_gain;
      p_k_d_ = d_gain;
    }

  if (mode == VEL_MODE)
    {
      v_k_p_ = p_gain;
      v_k_i_ = i_gain;
    }
}

Interface::Interface(): servo_state_pub_("servo/extended_states", &servo_states_msg_),
                        servo_cmd_sub_("servo/extended_cmds", &Interface::servoControlCallback, this),
                        servo_pid_gain_sub_("servo/set_pid_gain", &Interface::servoPIDGainCallback, this)
{
  /* variables */
  init_cnt_ = 1; // for catch the CAN messages from servo, 100 messages.
  servo_states_msg_.servos_length = 0;

  /* timer */
  last_connected_time_ = 0;
  servo_last_pub_time_ = 0;
  servo_last_ctrl_time_ = 0;
  external_encoder_i2c_ = nullptr;
  external_encoder_enabled_ = false;
#ifdef DJI_CAN_SERVO_EXTERNAL_ENCODER_ID
  external_encoder_servo_id_ = DJI_CAN_SERVO_EXTERNAL_ENCODER_ID;
#else
  external_encoder_servo_id_ = 1;
#endif
}

void Interface::init(CAN_GeranlHandleTypeDef* hcan, osMailQId* handle, ros::NodeHandle* nh, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
  init(hcan, handle, nh, GPIOx, GPIO_Pin, nullptr);
}

void Interface::init(CAN_GeranlHandleTypeDef* hcan, osMailQId* handle, ros::NodeHandle* nh, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, I2C_HandleTypeDef* external_encoder_i2c)
{
  /* CAN */
  CANDeviceManager::init(hcan, GPIOx, GPIO_Pin);
  CANDeviceManager::useRTOS(handle);
  CANDeviceManager::addDirectDevice(this);

  /* ros */
  nh_ = nh;
  nh_->advertise(servo_state_pub_);
  nh_->subscribe(servo_cmd_sub_);
  nh_->subscribe(servo_pid_gain_sub_);

  external_encoder_i2c_ = external_encoder_i2c;
#if DJI_CAN_SERVO_EXTERNAL_ENCODER
  external_encoder_enabled_ = external_encoder_i2c_ != nullptr;
  if (external_encoder_enabled_) {
    external_encoder_handler_.init(external_encoder_i2c_);
  }
#endif

  CANDeviceManager::CAN_START();
}

void Interface::sendData()
{
  bool cmd1_flag = false;
  bool cmd2_flag = false;
  uint8_t cmd1[8] = {0}; // 1~4
  uint8_t cmd2[8] = {0}; // 5~8

  for (std::map<int,Servo>::iterator it = servo_list_.begin(); it != servo_list_.end(); it++)
    {
      int id = it->first;
      float goal_curr = it->second.getGoalCurrent();
      int16_t goal_m_curr = goal_curr * 1000;

      if (id < 5)
        {
          cmd1_flag = true;

          cmd1[(id-1) * 2] = (goal_m_curr >> 8) & 0xFF;
          cmd1[(id-1) * 2 + 1] = goal_m_curr & 0xFF;

        }
      else
        {
          cmd2_flag = true;

          cmd2[(id-5) * 2] = (goal_m_curr >> 8) & 0xFF;
          cmd2[(id-5) * 2 + 1] = goal_m_curr & 0xFF;
        }
    }

  if (cmd1_flag) sendMessage(canTxId1, 8, cmd1, 0);
  if (cmd2_flag) sendMessage(canTxId2, 8, cmd2, 0);
}

void Interface::update(void)
{
  CANDeviceManager::tick(1);
  uint32_t now_time = HAL_GetTick();

  if (external_encoder_enabled_) {
    std::map<int,Servo>::iterator it = servo_list_.find(external_encoder_servo_id_);
    if (it != servo_list_.end()) {
      external_encoder_handler_.update(2);
      if (external_encoder_handler_.connected()) {
        it->second.updateExternalEncoder(external_encoder_handler_.getRawValue(), now_time);
      } else {
        it->second.setExternalEncoderConnected(false);
      }
    }
  }

  /* control */
  if(now_time - servo_last_ctrl_time_ >= SERVO_CTRL_INTERVAL)
    {
      for (std::map<int,Servo>::iterator it = servo_list_.begin(); it != servo_list_.end(); it++)
        {
          it->second.control();
        }

      sendData();
      servo_last_ctrl_time_ = now_time;
    }

  /* ros publish */
  if(now_time - servo_last_pub_time_ >= SERVO_PUB_INTERVAL)
    {
      if (servo_states_msg_.servos_length > 0)
        {
          servo_states_msg_.stamp = nh_->now();

          int i = 0;
          for (std::map<int,Servo>::iterator it = servo_list_.begin(); it != servo_list_.end(); it++)
          {
            spinal::ServoExtendedState servo;

            servo.index = it->first - 1;
            servo.angle = it->second.getFeedbackCounts();
            servo.velocity = it->second.getFeedbackVelocity();
            servo.current = it->second.getCurrent();

            servo_states_msg_.servos[i] = servo;
            i++;
          }

          servo_state_pub_.publish(&servo_states_msg_);
        }
      servo_last_pub_time_ = now_time;
    }
}

void Interface::receiveDataCallback(uint32_t identifier, uint32_t dlc, uint8_t* data)
{
  int id = identifier - 0x200;

  // update map
  if (init_cnt_ > 0)
    {
      std::map<int,Servo>::iterator it = servo_list_.find(id);
      if (it == servo_list_.end())
        {
          servo_list_[id] = Servo();
        }
      init_cnt_ --;

      if (init_cnt_ == 0)
        {
          servo_states_msg_.servos_length = servo_list_.size();
          servo_states_msg_.servos = new spinal::ServoExtendedState[servo_list_.size()];
        }
    }

  uint16_t counts = uint16_t((data[0] << 8) | data[1]);
  int16_t rpm = int16_t((data[2] << 8) | data[3]);
  int16_t m_curr = int16_t((data[4] << 8) | data[5]);
  servo_list_.at(id).update(counts, rpm, m_curr);
}


void Interface::servoControlCallback(const spinal::ServoExtendedCmds& msg)
{
  for (uint32_t i = 0; i < msg.servos_length; i++)
    {
      int id = msg.servos[i].index + 1;
      uint8_t mode = msg.servos[i].mode;
      float cmd = msg.servos[i].cmd;

      std::map<int,Servo>::iterator it = servo_list_.find(id);

      if (it != servo_list_.end())
        {
          it->second.setControlMode(mode);
          it->second.setGoalValue(cmd);
        }
    }
}

void Interface::servoPIDGainCallback(const spinal::ServoPIDGain& msg)
{
  std::map<int,Servo>::iterator it = servo_list_.find(msg.index + 1);
  if (it != servo_list_.end())
    {
      it->second.setPidGain(msg.mode, msg.p_gain, msg.i_gain, msg.d_gain);
    }

}
