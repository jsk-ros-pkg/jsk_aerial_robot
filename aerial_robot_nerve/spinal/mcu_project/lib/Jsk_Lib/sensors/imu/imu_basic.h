/*
 * imu_basic.h
 *
 *  Created on: 2016/11/01
 *      Author: anzai
 */

#ifndef APPLICATION_JSK_LIB_SENSORS_IMU_IMU_BASIC_H_
#define APPLICATION_JSK_LIB_SENSORS_IMU_IMU_BASIC_H_

#include "math/AP_Math.h"
#include <ros.h>
#include <std_msgs/UInt8.h>
#include "config.h"
#include "math/definitions.h"

using namespace ap;

class IMU
{
private:
  bool update_;
  static constexpr uint8_t ACC_LPF_FACTOR = 42; // old: 16
  static constexpr uint8_t GYRO_LPF_FACTOR = 12; //old: 8
  static constexpr int MAG_OUTLIER_MAX_COUNT = 500; //= 500ms, 1count = 1ms
  Vector3f gyro_, acc_, mag_;
  Vector3f raw_acc_, raw_gyro_, raw_mag_;
  Vector3f raw_acc_p_, raw_gyro_p_;
  Vector3f mag_max_, mag_min_;
  bool mag_filtering_flag_;
  uint16_t mag_outlier_counter_;
  uint32_t gyro_calib_duration_, acc_calib_duration_, mag_calib_duration_; // ms
  uint32_t gyro_calib_time_, acc_calib_time_, mag_calib_time_; // ms
  uint32_t gyro_calib_cnt_, acc_calib_cnt_, mag_calib_cnt_;
  Vector3f acc_bias_, gyro_bias_, mag_bias_, mag_scale_;

  void readCalibData(void);
  void writeCalibData(void);
  void process (void);

protected:

  virtual void ledOutput() {}
  virtual void updateRawData() = 0;
  Vector3f raw_gyro_adc_, raw_acc_adc_, raw_mag_adc_;

  bool calib_acc_, calib_gyro_, calib_mag_;

public:
  IMU();
  void init();
  Vector3f getGyro() { return gyro_; }
  Vector3f getAcc() { return acc_; }
  Vector3f getMag() { return mag_; }

  bool getUpdate() {return update_;}
  void setUpdate(bool update) {update_ = update;}
  bool getCalibrated() {
    if(!calib_acc_ && !calib_gyro_ && !calib_mag_) return true;
    else return false;
  }
  void update();
  inline Vector3f getGyroBias() {return gyro_bias_;}
  inline Vector3f getAccBias() {return acc_bias_;}
  inline Vector3f getMagBias() {return mag_bias_;}
  inline Vector3f getMagScale() {return mag_scale_;}
  inline void setGyroBias(Vector3f data) { gyro_bias_ = data;}
  inline void setAccBias(Vector3f data) { acc_bias_ = data;}
  inline void setMagBias(Vector3f data) { mag_bias_ = data;}
  inline void setMagScale(Vector3f data) { mag_scale_ = data;}
  void gyroCalib(bool flag, float duration);
  void accCalib(bool flag, float duration);
  void magCalib(bool flag, float duration);
  void resetCalib() ;


  static constexpr uint32_t GYRO_DEFAULT_CALIB_DURATION = 3; // 3s
  static constexpr uint32_t ACC_DEFAULT_CALIB_DURATION = 5; // 5s
  static constexpr uint32_t MAG_DEFAULT_CALIB_DURATION = 60; // 60s
  static constexpr float MAG_GENERAL_THRESH = 20.0f;

};


#endif /* APPLICATION_JSK_LIB_SENSORS_IMU_IMU_BASIC_H_ */
