/*
******************************************************************************
* File Name          : switcher.h
* Description        : I2C Multi Plexer Interface
******************************************************************************
*/

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#ifndef __I2C_MULTIPLEXER_H
#define __I2C_MULTIPLEXER_H

#include "config.h"

namespace I2C_MultiPlexer
{
  void init(I2C_HandleTypeDef* hi2c);
  bool changeChannel(uint8_t ch);
};



#endif
