/*
******************************************************************************
* File Name          : mag_encoder.cpp
* Description        : Magnetic Encoder Interface
******************************************************************************
*/

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#include "sensors/encoder/mag_encoder.h"

MagEncoder::MagEncoder():
  angle_pub_("encoder_angle", &angle_msg_)
{
}

void MagEncoder::init(I2C_HandleTypeDef* hi2c, ros::NodeHandle* nh)
{
  hi2c_ = hi2c;
  nh_ = nh;
  nh_->advertise(angle_pub_);
  raw_encoder_value_ = 0;

  last_time_ = HAL_GetTick() + 6000; // after 6s
}

void MagEncoder::update(void)
{
  uint32_t now_time = HAL_GetTick();
  if(now_time >= last_time_ + UPDATE_INTERVAL)
    {
      last_time_ = now_time;
      uint8_t val[1];
      val[0] = AS5600_REG_RAW_ANGLE;
      int i2c_status = HAL_I2C_Master_Transmit(hi2c_, AS5600_I2C_ADDRESS, val, 1, 100);
      if(i2c_status == HAL_OK)
        {
          uint8_t adc[2];
          HAL_I2C_Master_Receive(hi2c_, AS5600_I2C_ADDRESS , adc, 2, 100);
          raw_encoder_value_ = (uint16_t)(adc[0] << 8 | adc[1]);
        }
      else
        {
          raw_encoder_value_ = 65535;
        }
      angle_msg_.data = raw_encoder_value_;
      if(nh_->connected()) angle_pub_.publish(&angle_msg_);
    }

}
