/**
******************************************************************************
* File Name          : servo.cpp
* Description        : interface for DJI M2006 with C610
 ------------------------------------------------------------------*/

#include "servo.h"

using namespace DJI_M2006;

float clamp(float val, float limit) { return std::min(std::max(val, -limit), limit); }

Servo::Servo(): servo_state_pub_("servo/extended_states", &servo_state_msg_), servo_cmd_sub_("servo/extended_cmd", &Servo::servoControlCallback, this)
{
  last_connected_time_ = 0;
  servo_last_pub_time_ = 0;
  servo_last_ctrl_time_ = 0;

  rotations_ = 0;
  last_pos_measurement_ = kCountsPerRev;

  counts_ = 0;
  rpm_ = 0;
  m_curr_ = 0;

  output_pos_ = 0;
  output_vel_ = 0;
  curr_ = 0;

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
  int16_t goal_m_curr = goal_curr_ * 1000;
  uint8_t cmd[8];
  cmd[0] = (goal_m_curr >> 8) & 0xFF;
  cmd[1] = goal_m_curr & 0xFF;

  sendMessage(canTxId, 8, cmd, 0);
}

void Servo::receiveDataCallback(uint32_t identifier, uint32_t dlc, uint8_t* data)
{
  uint16_t counts = uint16_t((data[0] << 8) | data[1]);
  rpm_ = int16_t((data[2] << 8) | data[3]);
  m_curr_ = int16_t((data[4] << 8) | data[5]);

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

void Servo::update(void)
{
  CANDeviceManager::tick(1);
  uint32_t now_time = HAL_GetTick();

  /* control */
  if( now_time - servo_last_ctrl_time_ >= SERVO_CTRL_INTERVAL)
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

      sendData();
      servo_last_ctrl_time_ = now_time;
    }

  /* ros publish */

  if( now_time - servo_last_pub_time_ >= SERVO_PUB_INTERVAL)
    {
      /* send servo */
      servo_state_msg_.index = 0;
      servo_state_msg_.angle = output_pos_;
      servo_state_msg_.velocity = output_vel_;
      servo_state_msg_.current = curr_;

      servo_state_msg_.current = filter_vel_;

      servo_state_pub_.publish(&servo_state_msg_);
      servo_last_pub_time_ = now_time;
    }
}

void Servo::servoControlCallback(const spinal::ServoExtendedCmd& msg)
{
  control_mode_ = msg.mode;
  goal_value_ = msg.cmd;
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

  float err = goal_pos_ - output_pos_;
  float p_term = p_k_p_ * err;
  p_term = clamp(p_term, MAX_CURRENT);
  p_i_term_ += p_k_i_ * err * SERVO_CTRL_INTERVAL;
  p_i_term_ = clamp(p_i_term_, MAX_CURRENT);
  float d_term = p_k_d_ * (-output_vel_);
  d_term = clamp(d_term, MAX_CURRENT);

  goal_curr_ = p_term + p_i_term_ + d_term;
}
