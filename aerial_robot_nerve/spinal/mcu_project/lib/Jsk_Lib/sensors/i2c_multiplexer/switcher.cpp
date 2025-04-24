/*
******************************************************************************
* File Name          : switcher.cpp
* Description        : I2C Multi Plexer Interface
******************************************************************************
*/

#include "switcher.h"

namespace I2C_MultiPlexer
{
  namespace
  {
    I2C_HandleTypeDef* hi2c_;

    // the default I2C adress of PCA9547D
    uint8_t hub_address_ = 0xE0; // 0x70 << 1 (need to shift one bit left). TODO: reconfigurable
  };

  void init(I2C_HandleTypeDef* hi2c)
  {
    hi2c_ = hi2c;
  }

  bool changeChannel(uint8_t ch)
  {
    uint8_t val[1];
    val[0] = ch & 0x07; // assign the channel
    val[0] = ch | 0x08; // enable switch the channel

    int i2c_status = HAL_I2C_Master_Transmit(hi2c_, hub_address_, val, 1, 100);

    if(i2c_status == HAL_OK) return true;
    else return false;
  }
};
