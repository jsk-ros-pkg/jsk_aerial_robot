//
// Created by jinjie on 24/01/18.
//

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#ifndef ESC_TELEM_H
#define ESC_TELEM_H

#include "util/ring_buffer.h"
#include "math/AP_Math.h"
#include "config.h"
#include <ros.h>
#include <std_msgs/UInt8.h>

#define ESC_BUFFER_SIZE 512

namespace
{
#ifdef STM32H7
  uint8_t rx_buf_[ESC_BUFFER_SIZE] __attribute__((section(".EscRxBufferSection")));
#else
  uint8_t rx_buf_[ESC_BUFFER_SIZE];
#endif
  uint32_t rd_ptr_ = 0;
}

class ESCReader
{
public:
  ESCReader();
  ~ESCReader(){};

  UART_HandleTypeDef *huart_;
  ros::NodeHandle* nh_;
//  ros::Publisher esc_pub_;
//  spinal::EscTelem esc_msg_;

  void init(UART_HandleTypeDef* huart, ros::NodeHandle* nh);
  void update();

  bool available();
  int read();

private:
  uint8_t step_ = 0;
  uint8_t msg_id_ = 0;
  uint16_t payload_length_ = 0;
  uint16_t payload_counter_ = 0;
  uint8_t ck_a_ = 0;
  uint8_t ck_b_ = 0;
  uint8_t class_ = 0;
};

#endif  // ESC_TELEM_H
