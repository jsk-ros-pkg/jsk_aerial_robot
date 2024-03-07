/*
******************************************************************************
* File Name          : mag_encoder.cpp
* Description        : Magnetic Encoder Interface
******************************************************************************
*/

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#include "mag_encoder.h"

MagEncoder::MagEncoder()
{
}

void MagEncoder::init(I2C_HandleTypeDef* hi2c)
{
  hi2c_ = hi2c;
  raw_encoder_value_ = 0;
  connection_ = false;
}

void MagEncoder::update(void)
{
  uint8_t val[1];
  val[0] = AS5600_REG_RAW_ANGLE;
  int i2c_status = HAL_I2C_Master_Transmit(hi2c_, AS5600_I2C_ADDRESS, val, 1, 100);
  if(i2c_status == HAL_OK)
    {
      uint8_t adc[2];
      HAL_I2C_Master_Receive(hi2c_, AS5600_I2C_ADDRESS , adc, 2, 100);
      raw_encoder_value_ = (uint16_t)(adc[0] << 8 | adc[1]);
      int16_t value = (int16_t)raw_encoder_value_ + offset_;
      if(value >= RESOLUTION) value_ = value - RESOLUTION;
      else if(value < 0) value_ = value + RESOLUTION;
      else value_ = value;

      connection_ = true;
    }
  else
    {
      connection_ = false;
      raw_encoder_value_ = ERROR_VALUE;
      value_ = ERROR_VALUE;
    }

}
