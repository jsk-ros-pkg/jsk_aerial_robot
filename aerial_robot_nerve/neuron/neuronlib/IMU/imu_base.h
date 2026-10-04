/*
 * imu_base.h
 *
 *  Created on: 2016/10/25
 *      Author: anzai
 */

#ifndef APPLICATION_IMU_IMU_BASE_H_
#define APPLICATION_IMU_IMU_BASE_H_

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#include "CAN/can_device.h"
#include "Flashmemory/flashmemory.h"

using Vector3d = std::array<int16_t, 3>;

class Initializer;

class IMU : public CANDevice {
public:

  IMU(){}
  IMU(uint8_t slave_id):CANDevice(CAN::DEVICEID_IMU, slave_id){}
  ~IMU(){}
  virtual void init(SPI_HandleTypeDef* hspi);
  void update();

  bool getUpdate() { return update_; }
  void setUpdate(bool update) { update_ = update; }

  static const uint8_t GYRO_DLPF_CFG = 0x01;//0x04 //0: 250Hz, 0.97ms; 3: 41Hz, 5.9ms(kduino); 4: 20Hz: 9.9ms
  static const uint8_t ACC_DLPF_CFG = 0x03; //0x03: 41Hz, 11.80ms
  static const uint8_t MAG_ADDRESS = 0x0C;
  static const uint8_t MAG_DATA_REGISTER = 0x03;

  static const uint8_t GYRO_ADDRESS =  0x43;
  static const uint8_t ACC_ADDRESS =  0x3B;
  static const uint8_t MAG_SPI_ADDRESS = 0x49;

  static const uint8_t DMA_RESET_STAGE = 0x00;
  static const uint8_t DMA_GYRO_STAGE = 0x01;
  static const uint8_t DMA_ACC_STAGE = 0x02;
  static const uint8_t DMA_MAG_STAGE = 0x03;
  static const uint8_t DMA_READ_STAGE = 0x04;

  Vector3d  getAcc(){return acc_;}
  Vector3d  getGyro(){return gyro_;}
  Vector3d  getMag(){return mag_;}

  void sendData() override;
  void receiveDataCallback(uint8_t message_id, uint32_t DLC, uint8_t* data) override;

protected:

  SPI_HandleTypeDef* hspi_;

  Vector3d acc_, gyro_, mag_;

  bool update_;
  uint16_t send_data_flag_;

  virtual void gyroInit(void){};
  virtual void accInit(void){};
  virtual void magInit(void){};

  virtual void pollingRead (void){};

  friend class Initializer;
};

#endif /* APPLICATION_IMU_IMU_BASE_H_ */
