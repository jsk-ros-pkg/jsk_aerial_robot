/* 
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2021, UT.
 * All rights reserved.
 *
  */

#ifndef STMH32_HARDWARE_H_
#define STMH32_HARDWARE_H_

#define SUPPORT_RTOS 1
#define SUPPORT_LWIP 1

#if defined (STM32F756xx) || defined (STM32F746xx) || defined (STM32F745xx) || defined (STM32F765xx) || \
    defined (STM32F767xx) || defined (STM32F769xx) || defined (STM32F777xx) || defined (STM32F779xx) || \
    defined (STM32F722xx) || defined (STM32F723xx) || defined (STM32F732xx) || defined (STM32F733xx) || \
    defined (STM32F730xx) || defined (STM32F750xx)
#include "stm32f7xx_hal.h"
#define STM32F7
#endif

#if defined (STM32H743xx) || defined (STM32H753xx)  || defined (STM32H750xx) || defined (STM32H742xx) || \
    defined (STM32H745xx) || defined (STM32H755xx)  || defined (STM32H747xx) || defined (STM32H757xx) || \
    defined (STM32H7A3xx) || defined (STM32H7A3xxQ) || defined (STM32H7B3xx) || defined (STM32H7B3xxQ) || defined (STM32H7B0xx)  || defined (STM32H7B0xxQ) || \
    defined (STM32H735xx) || defined (STM32H733xx)  || defined (STM32H730xx) || defined (STM32H730xxQ)  || defined (STM32H725xx) || defined (STM32H723xx)
#include "stm32h7xx_hal.h"
#define STM32H7
#endif

#if SUPPORT_RTOS
#include "cmsis_os.h"
#endif

#if SUPPORT_LWIP
#include "lwip.h"
#include "lwip/udp.h"
#endif

#include <cstring>

#define BUFFER_EMPTY  -1
#define TX_BUFFER_SIZE 128
#define TX_BUFFER_WIDTH 64 // byte
#define RX_BUFFER_SIZE 512

template <typename T,  int SIZE>
class RingBuffer
{
public:
  RingBuffer()
  {
    byte_to_pop_ = 0;
    byte_to_push_ = 0;
    buffer_length_ = (uint16_t)SIZE;
  }
  ~RingBuffer(){  }

  bool pop(T* pop_buf, uint16_t len = 1)
  {
    if (byte_to_pop_ + len <= byte_to_push_)
      {
        std::memcpy(pop_buf, &buf_[byte_to_pop_], sizeof(T) * len);
        byte_to_pop_ += len;
        return true;
      }
    else
      {
        if(byte_to_pop_ <= byte_to_push_ || byte_to_pop_ + len > byte_to_push_ + buffer_length_)
          {
            return false;
          }
        else
          {
            if (byte_to_pop_ + len <= buffer_length_)
              {
                std::memcpy(pop_buf, &buf_[byte_to_pop_], sizeof(T) * len);
                byte_to_pop_ += len;
                if (byte_to_pop_ == buffer_length_) byte_to_pop_ = 0;
              }
            else
              {
                std::memcpy(pop_buf, &buf_[byte_to_pop_], sizeof(T) * (buffer_length_ - byte_to_pop_));
                std::memcpy(&pop_buf[buffer_length_ - byte_to_pop_], &buf_[0], sizeof(T) * (byte_to_pop_ + len - buffer_length_));
                byte_to_pop_ = byte_to_pop_ + len - buffer_length_;
                return true;
              }
          }
      }

    return false;
  }

  bool push(T* push_buf, uint16_t len = 1)
  {
    /* TODO: forcibly write the buffer, regardless of the byte_to_pop */

    if (byte_to_push_ + len <=  buffer_length_)
      {
        std::memcpy(&buf_[byte_to_push_], push_buf, sizeof(T) * len);

        byte_to_push_ += len;

        if (byte_to_push_ == buffer_length_)
          {
            byte_to_push_ = 0;
          }
      }
    else
      {
        std::memcpy(&buf_[byte_to_push_], push_buf, sizeof(T) * (buffer_length_ - byte_to_push_));
        uint16_t residual = len - buffer_length_ + byte_to_push_;
        std::memcpy(&buf_[0], &push_buf[buffer_length_ - byte_to_push_], sizeof(T) * residual);
        byte_to_push_ = residual;
      }

    return true;
  }

  bool available()
  {
    return (byte_to_push_ != byte_to_pop_);
  }

  T* getBuf()
  {
    return buf_;
  }

private:
  T buf_[SIZE];
  uint16_t byte_to_pop_, byte_to_push_;
  uint16_t buffer_length_;
};

/* for uart TX buffer */
template<int BUFFER_LENGTH>
struct TxBufferUnit{
  uint8_t tx_data_[BUFFER_LENGTH];//this means each txdata packet should be shorter than 255
  uint16_t tx_len_;
} ;


class STM32Hardware {

private:
  // UART is default supportted
  UART_HandleTypeDef *huart_;
  /* rx */
  uint32_t uart_rxdma_rd_ptr_ = 0;
  /* tx */
  uint8_t uart_tx_subscript_in_progress_;
  uint8_t uart_tx_subscript_to_add_;

#if SUPPORT_RTOS
  osMutexId *tx_mutex_;
  bool rtos_mode_;
#endif

#if SUPPORT_LWIP
  struct udp_pcb *udp_pcb_;
  struct pbuf *udp_p_;
  ip4_addr_t udp_dst_addr_;
  uint16_t udp_dst_port_;
  bool eth_mode_;
#endif

public:

  STM32Hardware()
  {
#if SUPPORT_RTOS
    rtos_mode_ = false;
#endif

#if SUPPORT_LWIP
    eth_mode_ = false;
#endif
  }
  ~STM32Hardware(){}

  void init() {}

#if SUPPORT_RTOS
  void init(UART_HandleTypeDef* huart, osMutexId* mutex, osSemaphoreId  *semaphore);
  
#endif

#if SUPPORT_LWIP
  void init(ip4_addr_t dst_addr, uint16_t src_port, uint16_t dst_port);
#endif
  bool rx_available();
  int read();
  void write(uint8_t* data, uint16_t length);
  int publish();
  uint32_t time(){return HAL_GetTick();}
};



#endif
