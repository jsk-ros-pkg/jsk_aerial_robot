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
#include "AP_Math.h"

// sensor scales
#define GYRO_SCALE 2000.0f / 32767.0f * M_PI / 180.0f
#define GRAVITY_MSS 9.80665f
#define ACC_GRAVITY_RESOLUTION 4096
#define ACC_SCALE GRAVITY_MSS / ACC_GRAVITY_RESOLUTION
#define MAG_SCALE 4912.0f / 32760.0f

// complementary filter
#define GYR_CMPF_FACTOR 600
#define GYR_CMPFM_FACTOR 250
#define INV_GYR_CMPF_FACTOR   (1.0f / (GYR_CMPF_FACTOR  + 1.0f))
#define INV_GYR_CMPFM_FACTOR  (1.0f / (GYR_CMPFM_FACTOR + 1.0f))
#define PRESCLAER_ACC 3 // if value=1, it means same rate with gyro, for genral attitude estimation
#define G_MIN 72
#define G_MAX 133


#define SPI_POLLING_MODE 0
#define SPI_DMA_MODE 1
#define SPI_MODE SPI_POLLING_MODE
#define SENSOR_DATA_LENGTH 7

#define IMU_SPI_CS_H       HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET)
#define IMU_SPI_CS_L      HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_RESET)


/* magenetometer update at about 100Hz, but the entire sensor data polling process is at 1KHz, should add prescaler for mag */
#define MAG_PRESCALER 4
/* magnetometer has bad noise bacause of the polling process, I think. So, we use a threshold filter method to remove the outlier */

using namespace ap;

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

  // we set a different lpf parameter from spinal, which let us do not need an additional FIR filter

  static const uint8_t GYRO_DLPF_CFG = 0x05; // https://github.com/tongtybj/aerial_robot/issues/196
  static const uint8_t ACC_DLPF_CFG = 0x06; // https://github.com/tongtybj/aerial_robot/issues/196

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

  static constexpr uint32_t GYRO_DEFAULT_CALIB_DURATION = 5000; // 5s
  static constexpr uint32_t ACC_DEFAULT_CALIB_DURATION = 5000; // 5s

  Vector3i  getAccAdc(){return acc_adc_;}
  Vector3i  getGyroAdc(){return gyro_adc_;}
  Vector3i  getMagAdc(){return mag_adc_;}

  Vector3f  getAcc(){return acc_;}
  Vector3f  getGyro(){return gyro_;}
  Vector3f  getMag(){return mag_;}

  static  uint8_t adc_[SENSOR_DATA_LENGTH]; // TODO: why static?

  void sendData() override;
  void receiveDataCallback(uint8_t message_id, uint32_t DLC, uint8_t* data) override;

private:

  SPI_HandleTypeDef* hspi_;

  Vector3i raw_acc_adc_, raw_gyro_adc_, raw_mag_adc_;
  Vector3i acc_adc_, gyro_adc_, mag_adc_;
  Vector3i gyro_bias_, acc_bias_, mag_bias_;
  Vector3l gyro_bias_sum_, acc_bias_sum_;
  Vector3f acc_, gyro_, mag_;
  Vector3f prev_mag_;

  bool update_;
  bool ahb_tx_suspend_flag_; // to avoid the confliction between SPI1 and USART1TX(ros)
  uint16_t send_data_flag_;

  // calibrate
  bool calib_acc_, calib_gyro_, calib_mag_;
  uint32_t gyro_calib_duration_, acc_calib_duration_; // ms
  uint32_t gyro_calib_time_, acc_calib_time_; // ms
  uint32_t gyro_calib_cnt_, acc_calib_cnt_;
  bool send_calib_data_;  // flag to send imu calib data via CAN

  // attitude estimator
  uint16_t acc_filter_cnt_;
  Vector3f est_g_,  est_m_;
  Vector3f rpy_;
  uint32_t attitdue_est_timestamp_;

  void gyroInit(void);
  void accInit(void);
  void magInit(void);

  void pollingRead (void);

  void calibrate (void);
  void attitude_estimate(void);

  void mpuWrite(uint8_t address, uint8_t value);
  uint8_t mpuRead(uint8_t address);

  friend class Initializer;
};

#endif /* APPLICATION_IMU_IMU_MPU9250_H_ */
