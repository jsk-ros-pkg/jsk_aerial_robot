/*
******************************************************************************
* File Name          : switcher.cpp
* Description        : I2C Multi Plexer Interface (PCA9546A)
* URL                : https://www.ti.com/lit/ds/symlink/pca9546a.pdf
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

  HAL_StatusTypeDef changeChannel(uint8_t ch)
  {
    uint8_t val[1];

    val[0] = (1 << ch) & 0x07; // assign the channel

    HAL_StatusTypeDef i2c_status = HAL_I2C_Master_Transmit(hi2c_, hub_address_, val, 1, 100);
    return i2c_status;
  }
};
