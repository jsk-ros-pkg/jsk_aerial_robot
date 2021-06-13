/* 
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2016, UT.
 * All rights reserved.
 */

#include "STMF7Hardware.h"
#include <string.h>

void STMF7Hardware::init(UART_HandleTypeDef *huart, osMutexId *mutex)
{
  huart_ = huart;

  /* rx */
  __HAL_UART_DISABLE_IT(huart, UART_IT_PE);
  __HAL_UART_DISABLE_IT(huart, UART_IT_ERR);
  HAL_UART_Receive_DMA(huart, rx_buf_, RX_BUFFER_SIZE);
  memset(rx_buf_, 0, sizeof(rx_buf_));


  /* tx */
  tx_mutex_ = mutex;

  subscript_in_progress_ = 0;
  subscript_to_add_ = 0;

  for(int i = 0; i < TX_BUFFER_SIZE; i++)
    {
      tx_buffer_unit_[i].tx_len_ = 1;
      memset(tx_buffer_unit_[i].tx_data_, 0, TX_BUFFER_WIDTH);
    }
}

bool STMF7Hardware::rx_available()
{
  uint32_t dma_write_ptr =  (RX_BUFFER_SIZE - huart_->hdmarx->Instance->NDTR) % (RX_BUFFER_SIZE);
  return (rd_ptr_ != dma_write_ptr);
}

int STMF7Hardware::read()
{
  /* handle RX Overrun Error */
  if ( __HAL_UART_GET_FLAG(huart_, UART_FLAG_ORE) )
    {
      __HAL_UART_CLEAR_FLAG(huart_,
                            UART_CLEAR_NEF | UART_CLEAR_OREF | UART_FLAG_RXNE | UART_FLAG_ORE);
      HAL_UART_Receive_DMA(huart_, rx_buf_, RX_BUFFER_SIZE);
    }


  uint32_t dma_write_ptr =  (RX_BUFFER_SIZE - huart_->hdmarx->Instance->NDTR) % (RX_BUFFER_SIZE);
  int c = -1;
  if(rd_ptr_ != dma_write_ptr)
    {
      c = (int)rx_buf_[rd_ptr_++];
      rd_ptr_ %= RX_BUFFER_SIZE;
    }
  return c;
}


int STMF7Hardware::publish()
{
  if (subscript_in_progress_ != subscript_to_add_)
    {
      /*
        Timeout value of > 1 ms is necessary for the initial pub/sub/srv topic info transmit (check with 2 ms, with max length of 120 (+8))
        1 ms will induce "Rejecting message on topicId=x, length=xx with bad checksum.", whcih prevent the propoer send of TWO topic info.

        P.S.: 10 ms might be necessary to handle large service response, e.g., spinal/GetBoardInfo for multilinks with more than eight links

        This occurs for the long topic name and topic types, such as
        - p_matrix_pseudo_inverse_inertia, type: spinal/PMatrixPseudoInverseWithInertia, length=120
        - extra_servo_torque_enable, type: spinal/ServoTorqueCmd, length=97
        - rpy/feedback_state, type: spinal/RollPitchYawTerm, length=92

        This Timeout is not HAL_Delay().
        Please check HAL_UART_Transmit and UART_WaitOnFlagUntilTimeout(huart, UART_FLAG_TXE, RESET, tickstart, Timeout)
      */
      int status = HAL_UART_Transmit(huart_, tx_buffer_unit_[subscript_in_progress_].tx_data_, tx_buffer_unit_[subscript_in_progress_].tx_len_, 10);


      /* mutex to avoid access from task of higher priority running "write" */
      if(tx_mutex_ != NULL) osMutexWait(*tx_mutex_, osWaitForever);

      subscript_in_progress_++;
      subscript_in_progress_ %= TX_BUFFER_SIZE;

      if(tx_mutex_ != NULL)  osMutexRelease(*tx_mutex_);

      return status;
    }
  else
    {
      return BUFFER_EMPTY;
    }
}

void STMF7Hardware::write(uint8_t * new_data, unsigned int new_size)
{
  /* mutex to avoid access from task with higher priority to call "write" or "publish" */
  if(tx_mutex_ != NULL) osMutexWait(*tx_mutex_, osWaitForever);

  /* if subscript comes around and get to one in progress_, skip */
  uint8_t packet_num = new_size / TX_BUFFER_WIDTH + (new_size % TX_BUFFER_WIDTH > 0?1:0);
  bool overflow = false;

  if(subscript_to_add_ >= subscript_in_progress_ && (subscript_to_add_ + packet_num) >= TX_BUFFER_SIZE)
    {
      if ((subscript_to_add_ + packet_num) % TX_BUFFER_SIZE >= subscript_in_progress_)
        overflow = true;
    }
  if(subscript_to_add_ < subscript_in_progress_)
    {
      if ((subscript_to_add_ + packet_num)  >= subscript_in_progress_)
        overflow = true;
    }

  if(!overflow)
    {
      for(unsigned int i = 0; i <  new_size / TX_BUFFER_WIDTH; i++)
        {
          tx_buffer_unit_[subscript_to_add_].tx_len_ = TX_BUFFER_WIDTH;
          memcpy(tx_buffer_unit_[subscript_to_add_].tx_data_, new_data + TX_BUFFER_WIDTH * i, TX_BUFFER_WIDTH);

          subscript_to_add_++;
          subscript_to_add_ %= TX_BUFFER_SIZE;
        }

      if(new_size % TX_BUFFER_WIDTH > 0)
        {
          tx_buffer_unit_[subscript_to_add_].tx_len_ = new_size % TX_BUFFER_WIDTH;
          memcpy(tx_buffer_unit_[subscript_to_add_].tx_data_, new_data + TX_BUFFER_WIDTH * (packet_num - 1), new_size % TX_BUFFER_WIDTH);

          subscript_to_add_++;
          subscript_to_add_ %= TX_BUFFER_SIZE;
        }
    }

  if(tx_mutex_ != NULL) osMutexRelease(*tx_mutex_);
}
