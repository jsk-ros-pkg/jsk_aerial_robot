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

#define TX_BUFFER_SIZE 50
#define TX_BUFFER_WIDTH 512
#define RX_BUFFER_SIZE 400
#define RX_PACKET_SIZE 16

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
  uint8_t* getRxValueP();
  RingBuffer<uint8_t, RX_PACKET_SIZE>* getRxBuffer();
  uint8_t getBurstSize();
};

/* TX */
template<int BUFFER_LENGTH>
struct TxBufferUnit{
  uint8_t tx_data_[BUFFER_LENGTH];//this means each txdata packet should be shorter than 255
  uint8_t tx_len_;
} ;

namespace tx
{
  void init(UART_HandleTypeDef *huart);
  int publish();
  void write(uint8_t * new_data, uint8_t new_size);

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

  int read()
  {
    return rx::read();
  }

  void write(uint8_t* data, int length){
    tx::write(data, length);
  }

  int publish(){
    return tx::publish();
  }

  uint32_t time(){return HAL_GetTick();}
};



#endif
