/*
 * imu_basic.h
 *
 *  Created on: 2016/11/01
 *      Author: anzai
 *  Modify on:  2020/10/27
 *      Author: zhao
 */

#ifndef APPLICATION_JSK_LIB_SENSORS_IMU_ONBOARD_H_
#define APPLICATION_JSK_LIB_SENSORS_IMU_ONBOARD_H_

#include "imu_basic.h"
#include "config.h"

class IMUOnboard : public IMU {
protected:
  static constexpr uint8_t ACC_LPF_FACTOR = 42; // old: 16
  static constexpr uint8_t GYRO_LPF_FACTOR = 12; //old: 8
  static constexpr int MAG_OUTLIER_MAX_COUNT = 500; //= 500ms, 1count = 1ms

  Vector3f raw_gyro_adc_, raw_acc_adc_, raw_mag_adc_;
  bool calib_acc_, calib_gyro_, calib_mag_;

  Vector3f raw_acc_, raw_gyro_, raw_mag_;
  Vector3f raw_gyro_p_, raw_acc_p_;
  Vector3f mag_max_, mag_min_;
  bool mag_filtering_flag_;
  uint16_t mag_outlier_counter_;
  uint32_t gyro_calib_duration_, acc_calib_duration_, mag_calib_duration_; // ms
  uint32_t gyro_calib_time_, acc_calib_time_, mag_calib_time_; // ms
  uint32_t gyro_calib_cnt_, acc_calib_cnt_, mag_calib_cnt_;

  static constexpr uint32_t GYRO_DEFAULT_CALIB_DURATION = 7000; // 7s
  static constexpr uint32_t ACC_DEFAULT_CALIB_DURATION = 5000; // 5s
  static constexpr uint32_t MAG_DEFAULT_CALIB_DURATION = 60000; // 60s
  static constexpr float MAG_GENERAL_THRESH = 20.0f;

  void readCalibData(void);
  void process (void);

  virtual void ledOutput() {}
  virtual void updateRawData() = 0;

public:
  IMUOnboard();
  void init() override;
  void update() override;

  void gyroCalib(bool flag, float duration) override;
  void accCalib(bool flag, float duration) override;
  void magCalib(bool flag, float duration) override;
  void resetCalib() override;
  void writeCalibData(void) override;

  bool getCalibrated();


};


#endif /* APPLICATION_JSK_LIB_SENSORS_IMU_IMU_BASIC_H_ */
