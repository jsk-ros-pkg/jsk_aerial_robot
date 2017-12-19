/*
******************************************************************************
* File Name          : imu_mpu9250.h
* Description        : IMU(MPU9250) Interface
******************************************************************************
*/

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#ifndef __IMU_H
#define __IMU_H

#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_dma.h"
#include "config.h"
/* #include "arm_math.h" */
#include "math/AP_Math.h"
#include "imu_basic.h"


#define SENSOR_DATA_LENGTH 7

#define IMU_SPI_CS_H       HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET)
#define IMU_SPI_CS_L      HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET)

#define MAG_PRESCALER 4 /* magenetometer update at about 200Hz, but the entire sensor data polling process is at 1KHz, should add prescaler for meg */
/* magnetometer has bad noise bacause of the polling process, I think. So, we use a threshold filter method to remove the outlier */

#define EXTERNAL_MAG_PRESCALER 13 // external mag(HMC5883): 75Hz
#define EXTERNAL_MAG_RATE 0.092f // 0.92mG/LSb -> 1G = 10e-4T -> uT = 0.092f

class IMUOnboard : public IMU {
public:
  IMUOnboard():IMU(){}
  ~IMUOnboard(){}
  void init(SPI_HandleTypeDef* hspi, I2C_HandleTypeDef* hi2c, ros::NodeHandle* nh);

private:
  static const uint8_t GYRO_DLPF_CFG = 0x03;//0x01 //0: 250Hz, 0.97ms; 3: 41Hz, 5.9ms(kduino); 4: 20Hz: 9.9ms
  static const uint8_t ACC_DLPF_CFG = 0x05; //0x03: 41Hz, 11.80ms
  static const uint8_t MAG_ADDRESS = 0x0C;
  static const uint8_t MAG_DATA_REGISTER = 0x03;
  static const uint8_t GYRO_ADDRESS =  0x43;
  static const uint8_t ACC_ADDRESS =  0x3B;
  static const uint8_t MAG_SPI_ADDRESS = 0x49;
  /* external magnetometer: HMC5883 builin UBLOX */
  static const uint8_t  EXTERNAL_MAG_DU = 10;
  static const uint8_t EXTERNAL_MAG_REGISTER = 0x3C; //0x1E << 1;
  static const uint8_t  HMC58X3_R_CONFA = 0x00;
  static const uint8_t  HMC58X3_R_CONFB = 0x01;
  static const uint8_t HMC58X3_R_MODE = 0x02;
  static const uint8_t HMC58X3_DATA_REGISTER =0x03;

  static  uint8_t adc_[SENSOR_DATA_LENGTH];
  static  uint32_t last_mag_time_;

  SPI_HandleTypeDef* hspi_;
  I2C_HandleTypeDef* hi2c_;
  ros::NodeHandle* nh_;

  bool use_external_mag_flag_;

  void gyroInit(void);
  void accInit(void);
  void magInit(void);

  void updateRawData (void) override;
  void mpuWrite(uint8_t address, uint8_t value);
  uint8_t mpuRead(uint8_t address);
  uint8_t dummy_[SENSOR_DATA_LENGTH];

};
#endif
