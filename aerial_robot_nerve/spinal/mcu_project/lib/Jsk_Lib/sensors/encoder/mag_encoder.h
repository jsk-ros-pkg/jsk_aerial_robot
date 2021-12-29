/*
******************************************************************************
* File Name          : mag_encoder.h
* Description        : Magnetic Encoder Interface
******************************************************************************
*/

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#ifndef __MAG_ENCODER_H
#define __MAG_ENCODER_H

#include <ros.h>
#include <std_msgs/UInt16.h>
#include "stm32f7xx_hal.h"

class MagEncoder
{
public:
  MagEncoder();
  ~MagEncoder(){};
  
  static const uint8_t AS5600_I2C_ADDRESS =  0x6c; // 0x36 << 1; NOTE: STM: i2c_address << 1 !!!
  static const uint8_t AS5600_REG_RAW_ANGLE =  0x0C;
  static const uint8_t UPDATE_INTERVAL = 20; //20 -> 50Hz

  void init(I2C_HandleTypeDef* hi2c, ros::NodeHandle* nh);
  void update(void);

private:
  I2C_HandleTypeDef* hi2c_;
  ros::NodeHandle* nh_;
  ros::Publisher angle_pub_;

  uint16_t raw_encoder_value_;
  uint32_t last_time_;

  std_msgs::UInt16 angle_msg_;

};

#endif
