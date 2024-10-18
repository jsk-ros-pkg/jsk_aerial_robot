/*
******************************************************************************
* File Name          : imu_mpu9250.h
* Description        : IMU(MPU9250) Interface
******************************************************************************
*/

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#ifndef __IMU_MPU_H
#define __IMU_MPU_H

#include "config.h"
#include "math/AP_Math.h"
#include "sensors/imu/imu_basic.h"

#define SENSOR_DATA_LENGTH 7

class IMUOnboard : public IMU {
public:
  IMUOnboard():IMU(){}
  ~IMUOnboard(){}
  void init(SPI_HandleTypeDef* hspi, I2C_HandleTypeDef* hi2c, ros::NodeHandle* nh,
            GPIO_TypeDef* spi_cs_port, uint16_t spi_cs_pin,
            GPIO_TypeDef* led_port, uint16_t led_pin);

  enum{INTERNAL_MAG = 0, LIS3MDL = 1, HMC58X3 = 2,};

protected:
  static  uint8_t adc_[SENSOR_DATA_LENGTH];
  static  uint32_t last_mag_time_;

  uint8_t mag_id_;

  SPI_HandleTypeDef* hspi_;
  I2C_HandleTypeDef* hi2c_;
  ros::NodeHandle* nh_;

  GPIO_TypeDef* spi_cs_port_;
  uint16_t spi_cs_pin_;
  GPIO_TypeDef* led_port_;
  uint16_t led_pin_;

  bool use_external_mag_flag_;
  uint32_t calib_indicator_time_; // ms

  uint8_t checkExternalMag(void);
private:
  static const uint8_t GYRO_DLPF_CFG = 0x03;//0x01 //0: 250Hz, 0.97ms; 3: 41Hz, 5.9ms(kduino); 4: 20Hz: 9.9ms
  static const uint8_t ACC_DLPF_CFG = 0x05; //0x03: 41Hz, 11.80ms
  static const uint8_t MAG_ADDRESS = 0x0C;
  static const uint8_t MAG_DATA_REGISTER = 0x03;
  static const uint8_t GYRO_ADDRESS =  0x43;
  static const uint8_t ACC_ADDRESS =  0x3B;
  static const uint8_t MAG_SPI_ADDRESS = 0x49;
  static const uint8_t MAG_READ_PRESCALER = 4; /* magenetometer update at about 200Hz, but the entire sensor data polling process is at 1KHz, should add prescaler for meg */
/* magnetometer has bad noise bacause of the polling process, I think. So, we use a threshold filter method to remove the outlier */


  /* external magnetometer: */
  static const uint8_t EXTERNAL_MAG_CHECK_COUNT = 10;

  /* LIS3MDL builin GPS module */
  /* https://github.com/STMicroelectronics/stm32-lis3mdl/blob/main/lis3mdl.h */
  static const uint8_t LIS3MDL_MAG_REGISTER = 0x3C; // 0x3C or 0x38, 00111x0b << 1 (x: 0 or 1)
  static const uint8_t LIS3MDL_PING = 0x0F;

  static const uint8_t LIS3MDL_CTRL_REG1 = 0x20;
  static const uint8_t LIS3MDL_CTRL_REG2 = 0x21;
  static const uint8_t LIS3MDL_CTRL_REG3 = 0x22;
  static const uint8_t LIS3MDL_CTRL_REG4 = 0x23;
  static const uint8_t LIS3MDL_CTRL_REG5 = 0x24;
  static const uint8_t LIS3MDL_MAG_FS_16_GA = 0x60; /*!< Full scale = ±16 Gauss */
  static const uint8_t LIS3MDL_OM_XY_HIGH = 0x40; // high performance for XY axes, 300Hz
  static const uint8_t LIS3MDL_OM_Z_HIGH = 0x08;
  static const uint8_t LIS3MDL_FAST_ODR = 0x02; // enable fast output data rate
  static const uint8_t LIS3MDL_CONTINUOUS_MODE = 0x00;
  static const uint8_t LIS3MDL_BDU_MSBLSB = 0x40;
  static const uint8_t LIS3MDL_OUTX_L = 0x28;
  static constexpr float LIS3MDL_MAG_RATE = 0.0146;  // Default: sensitivity value for 4G full scale   => 0.0146mT/LSb
  static const uint32_t LIS3MDL_READ_PRESCALER = 4; // 1000Hz/4= 250Hz

  /* HMC5883 builin GPS module */
  /* https://cdn-shop.adafruit.com/datasheets/HMC5883L_3-Axis_Digital_Compass_IC.pdf */
  static const uint8_t HMC58X3_MAG_REGISTER = 0x3C; //0x1E << 1;
  static const uint8_t HMC58X3_R_CONFA = 0x00;
  static const uint8_t HMC58X3_R_CONFB = 0x01;
  static const uint8_t HMC58X3_R_MODE = 0x02;
  static const uint8_t HMC58X3_DATA_REGISTER = 0x03;
  static const uint32_t HMC58X3_READ_PRESCALER = 13; // 1000Hz/13= 75Hz
  static constexpr float HMC58X3_RATE = 0.092; // 0.92mG/LSb (1.3G) -> 1G = 10e-4T -> 0.092mT/LSb

  virtual void gyroInit(void);
  virtual void accInit(void);
  virtual void magInit(void);

  // external mag
  void hmc58x3Init(void);
  void lis3mdlInit(void);
  void hmc58x3Update(void);
  void lis3mdlUpdate(void);

  void ledOutput(void)  override;
  void updateRawData (void) override;
  void mpuWrite(uint8_t address, uint8_t value);
  uint8_t mpuRead(uint8_t address);
  uint8_t dummy_[SENSOR_DATA_LENGTH];

};
#endif
