#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#include "encoder.h"

void Encoder::init(I2C_HandleTypeDef* hi2c)
{
  encoder_handler_.init(hi2c);
}

void Encoder::update()
{
  encoder_handler_.update();
}

void Encoder::sendData()
{
  uint16_t raw_value = encoder_handler_.getRawValue();
  int16_t value = encoder_handler_.getValue();
  uint8_t data[4];
  data[0] = static_cast<uint8_t>(raw_value & 0xFF);
  data[1] = static_cast<uint8_t>((raw_value >> 8) & 0xFF);
  data[2] = static_cast<uint8_t>(value & 0xFF);
  data[3] = static_cast<uint8_t>((value >> 8) & 0xFF);
  sendMessage(CAN::MESSAGEID_SEND_ENCODER_DATA, m_slave_id, 4, data, 0);
}

void Encoder::receiveDataCallback(uint8_t message_id, uint32_t DLC, uint8_t* data)
{
  if (message_id == CAN::MESSAGEID_RECEIVE_ENCODER_OFFSET) {
    int16_t offset = static_cast<int16_t>((data[1] << 8) | data[0]);
    encoder_handler_.setOffset(offset);
  }
}
