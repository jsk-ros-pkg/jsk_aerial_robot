/*
 * mag_encoder.h: Magnetic Encoder Interface
 *
 *  Created on: 2020/07/19
 *      Author: zhao
 */

#ifndef MAG_ENCODER_H
#define MAG_ENCODER_H

#include "stm32f4xx_hal.h"

class MagEncoder
{
public:
  MagEncoder();
  ~MagEncoder(){};

  static const uint8_t AS5600_I2C_ADDRESS =  0x6c; // 0x36 << 1; NOTE: STM: i2c_address << 1 !!!
  static const uint8_t AS5600_REG_RAW_ANGLE =  0x0C;
  static const uint16_t ERROR_VALUE =  65535;
  static const int16_t RESOLUTION =  4096;


  void init(I2C_HandleTypeDef* hi2c);
  void update(void);

  uint16_t getRawValue(void) const { return raw_encoder_value_;}
  int16_t getValue(void) const { return value_;}
  int16_t getOffset(void) const { return offset_;}
  void setOffset(int16_t offset) { offset_ = offset;}
  bool connected(void) { return connection_;}

private:
  I2C_HandleTypeDef* hi2c_;

  bool connection_;
  uint16_t raw_encoder_value_;
  int16_t offset_;
  int16_t value_;
};

#endif
