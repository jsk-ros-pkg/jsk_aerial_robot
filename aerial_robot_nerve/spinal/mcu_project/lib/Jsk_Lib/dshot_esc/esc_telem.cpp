//
// Created by jinjie on 24/01/18.
//

#include "esc_telem.h"

void ESCReader::init(UART_HandleTypeDef* huart)
{
  huart_ = huart;

  // use DMA for UART RX
  __HAL_UART_DISABLE_IT(huart, UART_IT_PE);
  __HAL_UART_DISABLE_IT(huart, UART_IT_ERR);
  HAL_UART_Receive_DMA(huart, rx_buf_, RX_BUFFER_SIZE);

  memset(rx_buf_, 0, RX_BUFFER_SIZE);
}

void ESCReader::update(int motor_id)
{
  if (!available()) return;
//  while (!available())
//  {
//    HAL_Delay(1);
//  }

  switch (motor_id)
  {
    case 1:
      readOnePacket(esc_msg_1_);
      break;
    case 2:
      readOnePacket(esc_msg_2_);
      break;
    case 3:
      readOnePacket(esc_msg_3_);
      break;
    case 4:
      readOnePacket(esc_msg_4_);
      break;
  }

  clearDMABuffer();
}

void ESCReader::readOnePacket(spinal::ESCTelemetry& esc_msg)
{
  // Byte 0: Temperature
  // Byte 1: Voltage high byte
  // Byte 2: Voltage low byte
  // Byte 3: Current high byte
  // Byte 4: Current low byte
  // Byte 5: Consumption high byte
  // Byte 6: Consumption low byte
  // Byte 7: Rpm high byte
  // Byte 8: Rpm low byte
  // Byte 9: 8-bit CRC

  uint8_t buffer[10];  // buffer for KISS esc telemetry data

  for (int i = 0; i < 10; i++)
  {
    /* try to read three times */
    int data = ESCReader::readOneByte();
    if (data < 0)
    {
      data = ESCReader::readOneByte();
    }
    if (data < 0)
    {
      data = ESCReader::readOneByte();
    }
    if (data < 0)
      return;

    buffer[i] = data;
  }


  /* check crc */
  uint8_t crc = get_crc8(buffer, 9);
  if (crc != buffer[9])  // crc error
  {
    return;
  }

  /* save data in esc_msg */
  esc_msg.temperature = buffer[0];
  esc_msg.voltage = buffer[1] << 8 | buffer[2];
  esc_msg.current = buffer[3] << 8 | buffer[4];
  esc_msg.consumption = buffer[5] << 8 | buffer[6];
  esc_msg.erpm = buffer[7] << 8 | buffer[8];

  is_new_msg_ = true;  // even if only one new_msg is received, the ros msg will be sent
}

bool ESCReader::available()
{
  uint32_t dma_write_ptr = (ESC_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart_->hdmarx)) % (ESC_BUFFER_SIZE);
  return (rd_ptr_ != dma_write_ptr);
}

void ESCReader::clearDMABuffer()
{
  rd_ptr_ = (ESC_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart_->hdmarx)) % (ESC_BUFFER_SIZE);
}

int ESCReader::readOneByte()
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

uint8_t update_crc8(uint8_t crc, uint8_t crc_seed){
  uint8_t crc_u, i;
  crc_u = crc;
  crc_u ^= crc_seed;
  for ( i=0; i<8; i++) crc_u = ( crc_u & 0x80 ) ? 0x7 ^ ( crc_u << 1 ) : ( crc_u << 1 );
  return (crc_u);
}

uint8_t get_crc8(uint8_t *Buf, uint8_t BufLen){
  uint8_t crc = 0, i;
  for( i=0; i<BufLen; i++) crc = update_crc8(Buf[i], crc);
  return (crc);
}
