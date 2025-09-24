/* 
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2021, UT.
 * All rights reserved.
 */

#include "STM32Hardware.h"
#include <string.h>

namespace
{
#ifdef STM32H7
  RingBuffer<uint8_t, RX_BUFFER_SIZE>  rx_buf_ __attribute__((section(".UartRxBufferSection")));
  struct TxBufferUnit<TX_BUFFER_WIDTH> uart_tx_buffer_unit_[TX_BUFFER_SIZE]  __attribute__((section(".UartTxBufferSection")));
#else
  RingBuffer<uint8_t, RX_BUFFER_SIZE>  rx_buf_;
  struct TxBufferUnit<TX_BUFFER_WIDTH> uart_tx_buffer_unit_[TX_BUFFER_SIZE];
#endif

#if SUPPORT_RTOS
  osSemaphoreId *tx_semaphore_;
#endif

#if SUPPORT_LWIP
  /* used in udpRecCb */
  uint16_t udp_src_port_;
#endif
}

#if SUPPORT_RTOS
// DMA + RTOS using in UART TX
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  osSemaphoreRelease (*tx_semaphore_);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if ((huart->ErrorCode & HAL_UART_ERROR_DMA) != 0U)
    {
      osSemaphoreRelease (*tx_semaphore_); // force relase semaphore if DMA TX fails
    }
}

void STM32Hardware::init(UART_HandleTypeDef *huart, osMutexId *mutex, osSemaphoreId  *semaphore)
{
  rtos_mode_ = true;

  huart_ = huart;
  __HAL_UART_DISABLE_IT(huart, UART_IT_PE);
  __HAL_UART_DISABLE_IT(huart, UART_IT_ERR);
  __HAL_UART_DISABLE_IT(huart, UART_IT_TXE);
  __HAL_UART_DISABLE_IT(huart, UART_IT_RXNE);
  __HAL_UART_DISABLE_IT(huart, UART_IT_RTO);
  __HAL_UART_DISABLE_IT(huart, UART_IT_IDLE);

  /* rx */
  HAL_UART_Receive_DMA(huart, rx_buf_.getBuf(), RX_BUFFER_SIZE);
  memset(rx_buf_.getBuf(), 0, RX_BUFFER_SIZE);

  /* tx */
  tx_mutex_ = mutex;
  tx_semaphore_ = semaphore;

  uart_tx_subscript_in_progress_ = 0;
  uart_tx_subscript_to_add_ = 0;

  for(int i = 0; i < TX_BUFFER_SIZE; i++)
    {
      uart_tx_buffer_unit_[i].tx_len_ = 1;
      memset(uart_tx_buffer_unit_[i].tx_data_, 0, TX_BUFFER_WIDTH);
    }
}
#endif

#if SUPPORT_LWIP
void udpRecvCb(void *arg, struct udp_pcb *tpcb, struct pbuf *p,
               const ip_addr_t *addr, uint16_t port)
{
  if (p != NULL)
    {
      if(port == udp_src_port_)
        {
          rx_buf_.push(reinterpret_cast<uint8_t*>(p->payload), p->len);
          pbuf_free(p);
        }
    }
}

void STM32Hardware::init(ip4_addr_t dst_addr, uint16_t src_port, uint16_t dst_port)
{
  eth_mode_ = true;

  udp_dst_addr_ = dst_addr;
  udp_src_port_ = src_port;
  udp_dst_port_ = dst_port;

  /* get new pcb */
  udp_pcb_ = udp_new();
  if (udp_pcb_ == NULL) return;

  /* bind to any IP address on port 7 */
  if (udp_bind(udp_pcb_, IP_ADDR_ANY, udp_src_port_) != ERR_OK)
    {
      memp_free(MEMP_UDP_PCB, udp_pcb_);
      return;
    }

  udp_recv(udp_pcb_, udpRecvCb, NULL);
}

#endif

bool STM32Hardware::rx_available()
{
#if SUPPORT_RTOS
  if (rtos_mode_)
    {
      uint32_t dma_write_ptr = (RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart_->hdmarx)) % (RX_BUFFER_SIZE);
      return (uart_rxdma_rd_ptr_ != dma_write_ptr);
    }
#endif

#if SUPPORT_LWIP
  if (eth_mode_)
    {
      return rx_buf_.available();
    }
#endif

  return false;
}

int STM32Hardware::read()
{
#if SUPPORT_RTOS
  if (rtos_mode_)
    {
      /* handle RX Overrun Error */
      if ( __HAL_UART_GET_FLAG(huart_, UART_FLAG_ORE) )
        {
          __HAL_UART_CLEAR_FLAG(huart_,
                                UART_CLEAR_NEF | UART_CLEAR_OREF | UART_FLAG_RXNE | UART_FLAG_ORE);
          HAL_UART_Receive_DMA(huart_, rx_buf_.getBuf(), RX_BUFFER_SIZE);
        }


      uint32_t dma_write_ptr =  (RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart_->hdmarx)) % (RX_BUFFER_SIZE);
      int c = -1;
      if(uart_rxdma_rd_ptr_ != dma_write_ptr)
        {
          c = (int)(rx_buf_.getBuf()[uart_rxdma_rd_ptr_++]);
          uart_rxdma_rd_ptr_ %= RX_BUFFER_SIZE;
        }
      return c;
    }
#endif


#if SUPPORT_LWIP
  if (eth_mode_)
    {
      if(!rx_buf_.available()) return -1;

      uint8_t r_data = 0;
      rx_buf_.pop(&r_data);
      return  r_data;
    }
#endif

  return -1;
}

int STM32Hardware::publish()
{
#if SUPPORT_RTOS
  if (rtos_mode_)
    {
      if (uart_tx_subscript_in_progress_ != uart_tx_subscript_to_add_)
        {
          int status = HAL_OK;
          if(tx_semaphore_ == NULL)
            {
              status = HAL_UART_Transmit(huart_, uart_tx_buffer_unit_[uart_tx_subscript_in_progress_].tx_data_, uart_tx_buffer_unit_[uart_tx_subscript_in_progress_].tx_len_, 10);
            }
          else
            {
              // use DMA with semaphore
              osSemaphoreWait(*tx_semaphore_, osWaitForever);
              status = HAL_UART_Transmit_DMA(huart_, uart_tx_buffer_unit_[uart_tx_subscript_in_progress_].tx_data_, uart_tx_buffer_unit_[uart_tx_subscript_in_progress_].tx_len_);
            }

          /* mutex to avoid access from task of higher priority running "write" */
          if(tx_mutex_ != NULL) osMutexWait(*tx_mutex_, osWaitForever);

          uart_tx_subscript_in_progress_++;
          uart_tx_subscript_in_progress_ %= TX_BUFFER_SIZE;

          if(tx_mutex_ != NULL)  osMutexRelease(*tx_mutex_);

          return status;
        }
      else
        {
          return BUFFER_EMPTY;
        }
    }
#endif

  return BUFFER_EMPTY;
}

void STM32Hardware::write(uint8_t * new_data, uint16_t new_size)
{
#if SUPPORT_RTOS
  if (rtos_mode_)
    {
      /* mutex to avoid access from task with higher priority to call "write" or "publish" */
      if(tx_mutex_ != NULL) osMutexWait(*tx_mutex_, osWaitForever);

      /* if subscript comes around and get to one in progress_, skip */
      uint8_t packet_num = new_size / TX_BUFFER_WIDTH + (new_size % TX_BUFFER_WIDTH > 0?1:0);
      bool overflow = false;

      if(uart_tx_subscript_to_add_ >= uart_tx_subscript_in_progress_ && (uart_tx_subscript_to_add_ + packet_num) >= TX_BUFFER_SIZE)
        {
          if ((uart_tx_subscript_to_add_ + packet_num) % TX_BUFFER_SIZE >= uart_tx_subscript_in_progress_)
            overflow = true;
        }
      if(uart_tx_subscript_to_add_ < uart_tx_subscript_in_progress_)
        {
          if ((uart_tx_subscript_to_add_ + packet_num)  >= uart_tx_subscript_in_progress_)
            overflow = true;
        }

      if(!overflow)
        {
          for(unsigned int i = 0; i <  new_size / TX_BUFFER_WIDTH; i++)
            {
              uart_tx_buffer_unit_[uart_tx_subscript_to_add_].tx_len_ = TX_BUFFER_WIDTH;
              memcpy(uart_tx_buffer_unit_[uart_tx_subscript_to_add_].tx_data_, new_data + TX_BUFFER_WIDTH * i, TX_BUFFER_WIDTH);

              uart_tx_subscript_to_add_++;
              uart_tx_subscript_to_add_ %= TX_BUFFER_SIZE;
            }

          if(new_size % TX_BUFFER_WIDTH > 0)
            {
              uart_tx_buffer_unit_[uart_tx_subscript_to_add_].tx_len_ = new_size % TX_BUFFER_WIDTH;
              memcpy(uart_tx_buffer_unit_[uart_tx_subscript_to_add_].tx_data_, new_data + TX_BUFFER_WIDTH * (packet_num - 1), new_size % TX_BUFFER_WIDTH);

              uart_tx_subscript_to_add_++;
              uart_tx_subscript_to_add_ %= TX_BUFFER_SIZE;
            }
        }

      if(tx_mutex_ != NULL) osMutexRelease(*tx_mutex_);

    }
#endif

#if SUPPORT_LWIP
  if (eth_mode_)
    {
      udp_p_ = pbuf_alloc(PBUF_TRANSPORT, new_size, PBUF_RAM);
      memcpy(udp_p_->payload, new_data, new_size);
      udp_sendto(udp_pcb_, udp_p_, &udp_dst_addr_, udp_dst_port_);
      pbuf_free(udp_p_);  /* necessary: free the pbuf */
    }
#endif
}


