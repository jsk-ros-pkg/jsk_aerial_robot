/*
 * imu_mpu9250.h
 *
 *  Created on: 2016/10/25
 *      Author: anzai
 */

#ifndef APPLICATION_IMU_IMU_MPU9250_H_
#define APPLICATION_IMU_IMU_MPU9250_H_

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#include "can_device.h"
#include "flashmemory.h"

using Vector3d = std::array<int16_t, 3>;

#define SPI_POLLING_MODE 0
#define SPI_DMA_MODE 1
#define SPI_MODE SPI_POLLING_MODE
#define SENSOR_DATA_LENGTH 7

#define IMU_SPI_CS_H       HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET)
#define IMU_SPI_CS_L      HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET)

/* magenetometer update at about 100Hz, but the entire sensor data polling process is at 1KHz, should add prescaler for mag */
#define MAG_PRESCALER 4
/* magnetometer has bad noise bacause of the polling process, I think. So, we use a threshold filter method to remove the outlier */

class Initializer;

class IMU : public CANDevice {
public:

  IMU(){}
  IMU(uint8_t slave_id):CANDevice(CAN::DEVICEID_IMU, slave_id){}
  ~IMU(){}
  void init(SPI_HandleTypeDef* hspi);
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

  static  uint8_t adc_[SENSOR_DATA_LENGTH];

  void sendData() override;
  void receiveDataCallback(uint8_t message_id, uint32_t DLC, uint8_t* data) override;

private:

  SPI_HandleTypeDef* hspi_;

  Vector3d acc_, gyro_, mag_;

  uint8_t dummy_[SENSOR_DATA_LENGTH];

  bool update_;
  bool ahb_tx_suspend_flag_; // to avoid the confliction between SPI1 and USART1TX(ros)
  uint16_t send_data_flag_;

  void gyroInit(void);
  void accInit(void);
  void magInit(void);

  void pollingRead (void);

  void process (void);

  void mpuWrite(uint8_t address, uint8_t value);
  uint8_t mpuRead(uint8_t address);

  friend class Initializer;
};

#endif /* APPLICATION_IMU_IMU_MPU9250_H_ */
