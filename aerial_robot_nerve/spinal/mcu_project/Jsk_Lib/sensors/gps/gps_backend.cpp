#include "sensors/gps/gps_backend.h"

namespace
{
  RingBufferFiFo<uint8_t, RING_BUFFER_SIZE> gps_rx_buf_;
  uint8_t gps_rx_value_[GPS_RX_SIZE];
  uint16_t gps_rx_size_;
}

void GPS_Backend::init(UART_HandleTypeDef* huart, ros::NodeHandle* nh)
{
  huart_ =huart;
  nh_ = nh;
  nh_->subscribe< ros::Subscriber<std_msgs::UInt8, GPS_Backend> >(gps_config_sub_);

  gps_rx_buf_.init();
}

void GPS_Backend::startReceiveDMA()
{
  CLEAR_BIT(huart_->Instance->CR1, USART_CR1_RE);
  HAL_UART_Receive_DMA(huart_, gps_rx_value_, GPS_RX_SIZE);
  huart_->hdmarx->XferCpltCallback = UBLOX_UART_DMAReceiveCpltUBLOX; //change the registerred func
  __HAL_UART_DISABLE_IT(huart_, UART_IT_RXNE);
  SET_BIT(huart_->Instance->CR1, USART_CR1_RE);
  huart_->State = HAL_UART_STATE_READY;
}

uint16_t GPS_Backend::available() { return gps_rx_buf_.length(); }

bool GPS_Backend::pop(uint8_t& pop_value)
{
  bool is_valid = gps_rx_buf_.pop(pop_value);
  return is_valid;
}

void GPS_Backend::UBLOX_UART_DMAReceiveCpltUBLOX(DMA_HandleTypeDef *hdma)
{
  UART_HandleTypeDef* huart = ( UART_HandleTypeDef* )((DMA_HandleTypeDef* )hdma)->Parent;

  /* DMA Normal mode */
  if((hdma->Instance->CR & DMA_SxCR_CIRC) == 0)
    {
      huart->RxXferCount = 0;

      /* Disable the DMA transfer for the receiver request by setting the DMAR bit 
         in the UART CR3 register */
      huart->Instance->CR3 &= (uint32_t)~((uint32_t)USART_CR3_DMAR);

      /* Check if a transmit Process is ongoing or not */
      if(huart->State == HAL_UART_STATE_BUSY_TX_RX)
        {
          huart->State = HAL_UART_STATE_BUSY_TX;
        }
      else
        {
          huart->State = HAL_UART_STATE_READY;
        }
    }

  for(std::size_t i = 0; i < GPS_RX_SIZE; i++)
    {
      if(!gps_rx_buf_.push(gps_rx_value_[i]))
        return;
    }
}
