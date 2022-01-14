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
  Vector3f gyro_, acc_, mag_; //board frame
  Vector3f gyro_v_, acc_v_, mag_v_; //virtual frame
  bool virtual_frame_; /* flag to decide wether use virtual frame */

  Vector3f raw_acc_, raw_gyro_, raw_mag_;
  Vector3f raw_gyro_p_, raw_acc_p_;
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
  Vector3f getGyro(bool force_board_frame = false)
  {
    if(force_board_frame) return gyro_;

    if(virtual_frame_) return gyro_v_;
    else return gyro_;
  }

  Vector3f getAcc(bool force_board_frame = false)
  {
    if(force_board_frame) return acc_;

    if(virtual_frame_) return acc_v_;
    else return acc_;
  }

  Vector3f getMag(bool force_board_frame = false)
  {
    if(force_board_frame) return mag_;

    if(virtual_frame_) return mag_v_;
    else return mag_;
  }

  inline void setGyroV(Vector3f gyro_v) { gyro_v_ = gyro_v; }
  inline void setAccV(Vector3f acc_v) { acc_v_ = acc_v; }
  inline void setMagV(Vector3f mag_v) { mag_v_ = mag_v; }

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
  inline bool getVirtualFrame() {return virtual_frame_;}
  inline void setVirtualFrame(bool virtual_frame) { virtual_frame_ = virtual_frame;}

  static constexpr uint32_t GYRO_DEFAULT_CALIB_DURATION = 3; // 3s
  static constexpr uint32_t ACC_DEFAULT_CALIB_DURATION = 5; // 5s
  static constexpr uint32_t MAG_DEFAULT_CALIB_DURATION = 60; // 60s
  static constexpr float MAG_GENERAL_THRESH = 20.0f;

};


#endif /* APPLICATION_JSK_LIB_SENSORS_IMU_IMU_BASIC_H_ */
