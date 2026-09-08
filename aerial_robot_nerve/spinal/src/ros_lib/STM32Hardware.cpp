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
  UART_HandleTypeDef *tx_huart_;
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
  if (huart == tx_huart_ && tx_semaphore_ != NULL && *tx_semaphore_ != NULL)
    osSemaphoreRelease(*tx_semaphore_);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == tx_huart_ && tx_semaphore_ != NULL &&
      *tx_semaphore_ != NULL &&
      (huart->ErrorCode & HAL_UART_ERROR_DMA) != 0U)
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
  tx_huart_ = huart;
  tx_mutex_owner_ = NULL;
  tx_mutex_depth_ = 0;

  uart_tx_subscript_in_progress_ = 0;
  uart_tx_subscript_to_add_ = 0;

  for(int i = 0; i < TX_BUFFER_SIZE; i++)
    {
      uart_tx_buffer_unit_[i].tx_len_ = 1;
      memset(uart_tx_buffer_unit_[i].tx_data_, 0, TX_BUFFER_WIDTH);
    }
}

bool STM32Hardware::lock()
{
  if (!rtos_mode_ || tx_mutex_ == NULL)
    return true;

  if (*tx_mutex_ == NULL)
    return false;

  const osThreadId current_thread = osThreadGetId();
  if (tx_mutex_owner_ == current_thread && tx_mutex_depth_ > 0)
    {
      ++tx_mutex_depth_;
      return true;
    }

  if (osMutexWait(*tx_mutex_, osWaitForever) != osOK)
    return false;

  tx_mutex_owner_ = current_thread;
  tx_mutex_depth_ = 1;
  return true;
}

void STM32Hardware::unlock()
{
  if (!rtos_mode_ || tx_mutex_ == NULL || *tx_mutex_ == NULL)
    return;

  const osThreadId current_thread = osThreadGetId();
  if (tx_mutex_owner_ != current_thread || tx_mutex_depth_ == 0)
    return;

  --tx_mutex_depth_;
  if (tx_mutex_depth_ > 0)
    return;

  tx_mutex_owner_ = NULL;
  osMutexRelease(*tx_mutex_);
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
              if (osSemaphoreWait(*tx_semaphore_, osWaitForever) != osOK)
                return HAL_TIMEOUT;
              status = HAL_UART_Transmit_DMA(huart_, uart_tx_buffer_unit_[uart_tx_subscript_in_progress_].tx_data_, uart_tx_buffer_unit_[uart_tx_subscript_in_progress_].tx_len_);
              if (status != HAL_OK)
                osSemaphoreRelease(*tx_semaphore_);
            }

          if (status != HAL_OK)
            return status;

          /* mutex to avoid access from task of higher priority running "write" */
          if (!lock())
            return HAL_ERROR;

          uart_tx_subscript_in_progress_++;
          uart_tx_subscript_in_progress_ %= TX_BUFFER_SIZE;

          unlock();

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
      if (!lock())
        return;

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

      unlock();

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
