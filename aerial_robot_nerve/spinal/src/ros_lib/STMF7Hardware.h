/* 
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2016, UT.
 * All rights reserved.
 *
  */

#ifndef ROS_STMF7_HARDWARE_H_
#define ROS_STMF7_HARDWARE_H_

#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_uart.h"
#include "stm32f7xx_hal_dma.h"

#define BUFFER_EMPTY  -1
#define TX_BUFFER_SIZE 50 // can not be too small, to handle the first void negotiateTopics() process which send the topic information at the same time.
#define TX_BUFFER_WIDTH 512 // can not be too small, to handle the large size topic such as board information
#define RX_BUFFER_SIZE 512

template <typename T,  int SIZE>
class RingBuffer
{
public:
  RingBuffer()
  {
    byte_in_progress_ = 0;
    byte_to_add_ = 0;
    buffer_length_ = (uint16_t)SIZE;
  }
  ~RingBuffer(){  }

  bool pop(T& pop_value)
  {
    if (byte_in_progress_ != byte_to_add_)
      {
        pop_value =  buf_[byte_in_progress_];

        byte_in_progress_++;
        if (byte_in_progress_ == buffer_length_)
          byte_in_progress_ = 0;

        return true;
      }
    return false;
  }

  bool push(T new_value)
  {
    // the process node should have higher priority than the rx it callback
#if 0
    if ((byte_in_progress_ == (byte_to_add_ + 1)) || ( (byte_to_add_ == (buffer_length_ - 1) )&& (byte_in_progress_ == 0)) ) return false;
#endif

    buf_[byte_to_add_] = new_value;

    byte_to_add_++;

    if (byte_to_add_ == buffer_length_)
      {
        byte_to_add_ = 0;
      }
    return true;
  }

  uint16_t length()
  {
    if(byte_to_add_ - byte_in_progress_ >= 0)
      return (byte_to_add_ - byte_in_progress_);
    else 
      return (byte_to_add_ - (buffer_length_ - byte_in_progress_));
  }

private:
  T buf_[SIZE];
  int16_t byte_in_progress_, byte_to_add_;
  uint16_t buffer_length_;
};



/* RX */
namespace rx
{
  void init(UART_HandleTypeDef *huart);
  int read();
  bool available();

};

/* TX */
template<int BUFFER_LENGTH>
struct TxBufferUnit{
  uint8_t tx_data_[BUFFER_LENGTH];//this means each txdata packet should be shorter than 255
  uint16_t tx_len_;
} ;

namespace tx
{
  void init(UART_HandleTypeDef *huart);
  int publish();
  void write(uint8_t * new_data, unsigned int new_size);

  uint8_t subscriptInProgress();
  uint8_t subscriptToAdd();
  uint8_t  getCurrentTransmitBufferLen();
  uint8_t*  getCurrentTransmitBufferP();
};


class STMF7Hardware {
public:
  typedef UART_HandleTypeDef serial_class;

  STMF7Hardware(){}

  void init()
  {
    // do nothing
  }

  void init(char *portName)
  {
    // do nothing
  }

  void init(serial_class* huart)
  {
	  rx::init(huart);
	  tx::init(huart);
  }

  bool rx_available()
  {
    return rx::available();
  }

  int read()
  {
    return rx::read();
  }

  void write(uint8_t* data, unsigned int length){
    tx::write(data, length);
  }

  int publish(){
    return tx::publish();
  }

  uint32_t time(){return HAL_GetTick();}
};



#endif
