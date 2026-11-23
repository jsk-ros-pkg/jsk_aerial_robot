#ifndef APPLICATION_ENCODER_ENCODER_H_
#define APPLICATION_ENCODER_ENCODER_H_

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#include "CAN/can_device.h"
#include "Servo/Encoder/mag_encoder.h"

class Encoder : public CANDevice {
public:
  Encoder(){}
  Encoder(uint8_t slave_id):CANDevice(CAN::DEVICEID_ENCODER, slave_id){}
  void init(I2C_HandleTypeDef* hi2c);
  void update();
  void sendData() override;
  void receiveDataCallback(uint8_t message_id, uint32_t DLC, uint8_t* data) override;

private:
  MagEncoder encoder_handler_;
};

#endif /* APPLICATION_ENCODER_ENCODER_H_ */
