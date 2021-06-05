/* 
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2016, UT.
 * All rights reserved.
 */

#include "STMF7Hardware.h"
#include <string.h>
#include "it.h"

/* RX */
namespace
{
  uint8_t rx_buf_[RX_BUFFER_SIZE];
  UART_HandleTypeDef *rx_huart_;
  uint32_t rd_ptr_ = 0;
}

namespace rx
{
 void init(UART_HandleTypeDef *huart)
 {
   __HAL_UART_DISABLE_IT(huart, UART_IT_PE);
   __HAL_UART_DISABLE_IT(huart, UART_IT_ERR);
   HAL_UART_Receive_DMA(huart, rx_buf_, RX_BUFFER_SIZE);

   memset(rx_buf_, 0, sizeof(rx_buf_));
   rx_huart_ = huart;
  }

  bool available()
  {
    uint32_t dma_write_ptr =  (RX_BUFFER_SIZE - rx_huart_->hdmarx->Instance->NDTR) % (RX_BUFFER_SIZE);
    return (rd_ptr_ != dma_write_ptr);
  }

  int read()
  {
    uint32_t dma_write_ptr =  (RX_BUFFER_SIZE - rx_huart_->hdmarx->Instance->NDTR) % (RX_BUFFER_SIZE);
    int c = -1;
    if(rd_ptr_ != dma_write_ptr)
      {
        c = (int)rx_buf_[rd_ptr_++];
        rd_ptr_ %= RX_BUFFER_SIZE;
      }
    return c;
  }
};



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

  void write(uint8_t * new_data, unsigned int new_size)
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
