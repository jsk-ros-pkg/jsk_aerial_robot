//
// Created by jinjie on 24/01/18.
//

#include "esc_telem.h"

ESCReader::ESCReader()
{
}

void ESCReader::init(UART_HandleTypeDef* huart, ros::NodeHandle* nh)
{
  huart_ = huart;
  nh_ = nh;

  // use DMA for UART RX
  __HAL_UART_DISABLE_IT(huart, UART_IT_PE);
  __HAL_UART_DISABLE_IT(huart, UART_IT_ERR);
  HAL_UART_Receive_DMA(huart, rx_buf_, RX_BUFFER_SIZE);

  memset(rx_buf_, 0, RX_BUFFER_SIZE);
}

void ESCReader::update()
{
  while (true)
  {
    int data = ESCReader::read();
    if (data < 0)
      return; // finish

    printf("%d\n", data);
  }
}

bool ESCReader::available()
{
  uint32_t dma_write_ptr = (ESC_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart_->hdmarx)) % (ESC_BUFFER_SIZE);
  return (rd_ptr_ != dma_write_ptr);
}

int ESCReader::read()
{
  /* handle RX Overrun Error */
  if (__HAL_UART_GET_FLAG(huart_, UART_FLAG_ORE))
  {
    __HAL_UART_CLEAR_FLAG(huart_, UART_CLEAR_NEF | UART_CLEAR_OREF | UART_FLAG_RXNE | UART_FLAG_ORE);
    HAL_UART_Receive_DMA(huart_, rx_buf_, ESC_BUFFER_SIZE);  // restart
  }

  uint32_t dma_write_ptr = (ESC_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart_->hdmarx)) % (ESC_BUFFER_SIZE);
  int c = -1;
  if (rd_ptr_ != dma_write_ptr)
  {
    c = (int)rx_buf_[rd_ptr_++];
    rd_ptr_ %= ESC_BUFFER_SIZE;
  }
  return c;
}