/*
******************************************************************************
* File Name          : gps_ublox.cpp
* Description        : Ublox interface
******************************************************************************
*/

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#include "sensors/gps/gps_ublox.h"
#include "config.h"

GPS::GPS():
  GPS_Backend(),
  step_(0),
  msg_id_(0),
  payload_length_(0),
  payload_counter_(0),
  ck_a_(0),
  ck_b_(0),
  class_(0),
  last_update_time_(0)
{
}

void GPS::init(UART_HandleTypeDef *huart, ros::NodeHandle* nh)
{
  GPS_Backend::init (huart, nh);

  /* change the rate */
  configureRate(INIT_RATE);
  CLEAR_BIT(huart_->Instance->CR1, USART_CR1_RE);
  HAL_Delay(100);

  //DMA
  //start usart revceive dma interrupt
  HAL_UART_Receive_DMA(huart_, getRxPointer(), getRxSize());
  huart_->hdmarx->XferCpltCallback = UBLOX_UART_DMAReceiveCpltUBLOX; //change the registerred func
  __HAL_UART_DISABLE_IT(huart_, UART_IT_RXNE);
  SET_BIT(huart_->Instance->CR1, USART_CR1_RE);
  huart_->State = HAL_UART_STATE_READY;
}

void GPS::update()
{
  uint8_t data = 0;
  while (available() > 0)
    {
      pop(data);
      read(data);
    }
}

bool GPS::read(uint8_t data)
{
  bool parsed = false;

  LED2_L; //debug

 reset:
  switch(step_)
    {
    case 1:
      if (PREAMBLE2 == data) {
        step_++;
        break;
      }
      step_ = 0;
      /* no break */
    case 0:
      if(PREAMBLE1 == data)
        step_++;
      break;

    case 2:
      step_++;
      class_ = data;
      ck_b_ = ck_a_ = data;                               // reset the checksum accumulators
      break;
    case 3:
      step_++;
      ck_b_ += (ck_a_ += data);                   // checksum byte
      msg_id_ = data;
      break;
    case 4:
      step_++;
      ck_b_ += (ck_a_ += data);                   // checksum byte
      payload_length_ = data;                             // payload length low byte
      break;
    case 5:
      step_++;
      ck_b_ += (ck_a_ += data);                   // checksum byte

      payload_length_ += (uint16_t)(data<<8);
      if (payload_length_ > sizeof(buffer_))
      {
        // assume any payload bigger then what we know about is noise
        payload_length_ = 0;
        step_ = 0;
        goto reset;
      }
      payload_counter_ = 0; // prepare to receive payload
      break;
    case 6: /* Receive message data */
      ck_b_ += (ck_a_ += data); // checksum byte
      if (payload_counter_ < sizeof(buffer_))
        buffer_.bytes[payload_counter_] = data;

      if (++payload_counter_ == payload_length_) step_++;
      break;
    case 7:     /* Checksum and message processing */
      step_++;
      if (ck_a_ != data) {
        step_ = 0;
        goto reset;
      }
      break;
    case 8:
      step_ = 0;
      if (ck_b_ != data) {
        break; // bad checksum
      }

      /* receive some package */
      parsed = true;
      parsePacket();

      break;
    }
  return parsed;
}

bool GPS::parsePacket(void)
{
  switch (msg_id_)
    {
    case MSG_PVT:
      {
        LED2_H;
        state_.location.lng    = buffer_.pvt.longitude;
        state_.location.lat    = buffer_.pvt.latitude;
        state_.location.alt    = buffer_.pvt.altitude_msl / 10;

        state_.status = buffer_.pvt.fix_type;

        state_.horizontal_accuracy = buffer_.pvt.horizontal_accuracy*1.0e-3f;
        state_.vertical_accuracy = buffer_.pvt.vertical_accuracy*1.0e-3f;
        state_.have_horizontal_accuracy = true;
        state_.have_vertical_accuracy = true;

        state_.ground_speed     = buffer_.pvt.speed_2d*0.001f;          // m/s
        state_.ground_course_cd = (buffer_.pvt.heading_2d / 1000);       // Heading 2D deg * 100000 rescaled to deg * 100
        state_.have_vertical_velocity = true;
        state_.velocity.x = buffer_.pvt.ned_north * 0.001f;
        state_.velocity.y = buffer_.pvt.ned_east * 0.001f;
        state_.velocity.z = buffer_.pvt.ned_down * 0.001f;
        state_.have_speed_accuracy = true;
        state_.speed_accuracy = buffer_.pvt.speed_accuracy*0.001f;
        state_.hdop        = buffer_.pvt.position_DOP;
        state_.num_sats    = buffer_.pvt.satellites;

        // change the rate
        if(HAL_GetTick() - last_update_time_ > (INIT_RATE + MEASURE_RATE) / 2 )
          configureRate(MEASURE_RATE);

        last_update_time_  = HAL_GetTick();

        /* asynchronized update */
        update_ = true;

        return true;
      }
    case MSG_POSLLH:
      {
        state_.location.lng    = buffer_.posllh.longitude;
        state_.location.lat    = buffer_.posllh.latitude;
        state_.location.alt    = buffer_.posllh.altitude_msl / 10;
        state_.horizontal_accuracy = buffer_.posllh.horizontal_accuracy*1.0e-3f;
        state_.vertical_accuracy = buffer_.posllh.vertical_accuracy*1.0e-3f;
        state_.have_horizontal_accuracy = true;
        state_.have_vertical_accuracy = true;

        return true;
        //break;
      }
    case MSG_VELNED:
      {
        state_.ground_speed     = buffer_.velned.speed_2d*0.01f;          // m/s
        state_.ground_course_cd = (buffer_.velned.heading_2d / 1000);       // Heading 2D deg * 100000 rescaled to deg * 100
        state_.have_vertical_velocity = true;
        state_.velocity.x = buffer_.velned.ned_east * 0.01f;
        state_.velocity.y = buffer_.velned.ned_east * 0.01f;
        state_.velocity.z = buffer_.velned.ned_down * 0.01f;
        state_.have_speed_accuracy = true;
        state_.speed_accuracy = buffer_.velned.speed_accuracy*0.01f;

        return true;
        // break;
      }
    default:
      {
        break;
      }
    }
}

void GPS::updateChecksum(uint8_t *data, uint16_t len, uint8_t &ck_a, uint8_t &ck_b)
{
  while (len--)
    {
      ck_a += *data;
      ck_b += ck_a;
      data++;
    }
}

void GPS::sendMessage(uint8_t msg_class, uint8_t msg_id, void *msg, uint16_t size)
{
  struct ubx_header header;
  uint8_t ck_a=0, ck_b=0;
  header.preamble1 = PREAMBLE1;
  header.preamble2 = PREAMBLE2;
  header.msg_class = msg_class;
  header.msg_id    = msg_id;
  header.length    = size;

  updateChecksum((uint8_t *)&header.msg_class, sizeof(header)-2, ck_a, ck_b);
  updateChecksum((uint8_t *)msg, size, ck_a, ck_b);

  write((const uint8_t *)&header, sizeof(header));
  write((const uint8_t *)msg, size);
  write((const uint8_t *)&ck_a, 1);
  write((const uint8_t *)&ck_b, 1);
}

bool GPS::configureMessageRate(uint8_t msg_class, uint8_t msg_id, uint8_t rate)
{
  struct ubx_cfg_msg_rate msg;
  msg.msg_class = msg_class;
  msg.msg_id    = msg_id;
  msg.rate          = rate;
  sendMessage(CLASS_CFG, MSG_CFG_MSG, &msg, sizeof(msg));
  return true;
}

void GPS::configureRate(uint16_t rate)
{
  struct ubx_cfg_nav_rate msg;
  msg.measure_rate_ms = rate; // MEASURE_RATE;
  msg.nav_rate        = 1;
  msg.timeref         = 0;     // UTC time
  sendMessage(CLASS_CFG, MSG_CFG_RATE, &msg, sizeof(msg));
}


void
GPS::UBLOX_UART_DMAReceiveCpltUBLOX(DMA_HandleTypeDef *hdma)
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

  for(std::size_t i = 0; i < gps_rx_size_; i++)
    {
      if(!gps_rx_buf_.push(gps_rx_value_[i]))
        return;
    }
}
