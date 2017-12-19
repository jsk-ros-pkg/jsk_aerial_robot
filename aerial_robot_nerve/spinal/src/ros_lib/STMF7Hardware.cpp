/* 
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2016, UT.
 * All rights reserved.
 */

#include "STMF7Hardware.h"
#include <string.h>

/* RX */
namespace
{
uint8_t rx_value_[RX_PACKET_SIZE];
RingBuffer<uint8_t, RX_BUFFER_SIZE>  rx_buf_;
}

namespace rx
{
 void init(UART_HandleTypeDef *huart)
 {
    HAL_UART_Receive_DMA(huart, rx_value_, RX_PACKET_SIZE); //1byte receive protocol
    __HAL_UART_DISABLE_IT(huart, UART_IT_RXNE);
  }

  int read()
  {
    if(!rx::available()) return -1;

    uint8_t r_data = 0;
    rx_buf_.pop(r_data);
    return  r_data;
  }

  bool available() { return rx_buf_.length(); }
  uint8_t* getRxValueP() { return rx_value_; }
};


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  for(int i = 0; i < RX_PACKET_SIZE; i++)
    rx_buf_.push(rx_value_[i]);
}


/* TX */
namespace
{
  UART_HandleTypeDef *tx_huart_;
  struct TxBufferUnit<TX_BUFFER_WIDTH> tx_buffer_unit_[TX_BUFFER_SIZE];
  uint8_t subscript_in_progress_;
  uint8_t subscript_to_add_;
  bool idle_flag_;
}

namespace tx
{
  void init(UART_HandleTypeDef *huart)
  {
    tx_huart_ = huart;
    idle_flag_ = true;
    subscript_in_progress_ = 0;
    subscript_to_add_ = 0;

    for(int i = 0; i < TX_BUFFER_SIZE; i++)
      {
        tx_buffer_unit_[i].tx_len_ = 1;
        for(int j = 0; j < TX_BUFFER_WIDTH; j++)
          tx_buffer_unit_[i].tx_data_[j]  = 1;
      }
  }

  uint8_t subscriptInProgress(){return subscript_in_progress_;}
  uint8_t subscriptToAdd(){return subscript_to_add_;}
  uint8_t  getCurrentTransmitBufferLen()
  {
    return tx_buffer_unit_[subscript_in_progress_].tx_len_;
  }

  uint8_t*  getCurrentTransmitBufferP()
  {
    return tx_buffer_unit_[subscript_in_progress_].tx_data_;
  }

  int publish()
  {
    int status = HAL_OK;
    if (subscript_in_progress_ != subscript_to_add_)
      {
        status = HAL_UART_Transmit(tx_huart_, tx_buffer_unit_[subscript_in_progress_].tx_data_, tx_buffer_unit_[subscript_in_progress_].tx_len_, 10);

        subscript_in_progress_++;
        if (subscript_in_progress_ == TX_BUFFER_SIZE) subscript_in_progress_ = 0;
      }
    return status;
  }

  void write(uint8_t * new_data, uint8_t new_size)
  {
    /* if subscript comes around and get to one in progress_, then wait. */
    if (subscript_in_progress_ == subscript_to_add_ + 1 || ( subscript_to_add_ == TX_BUFFER_SIZE - 1 && subscript_in_progress_ == 0) )
      {
        //TODO: address the overflow
        while(subscript_in_progress_ == subscript_to_add_ + 1 || ( subscript_to_add_ == TX_BUFFER_SIZE - 1 && subscript_in_progress_ == 0)){}
      //  return;
      }

    tx_buffer_unit_[subscript_to_add_].tx_len_ = new_size;
    memcpy(tx_buffer_unit_[subscript_to_add_].tx_data_, new_data, new_size);

    subscript_to_add_++;

    if (subscript_to_add_ == TX_BUFFER_SIZE) subscript_to_add_ = 0;
  }

};
