/*
 * imu_basic.h
 *
 *  Created on: 2016/11/01
 *      Author: anzai
 *  Modify on:  2020/10/27
 *      Author: zhao
 */

#ifndef APPLICATION_JSK_LIB_SENSORS_IMU_IMU_BASIC_H_
#define APPLICATION_JSK_LIB_SENSORS_IMU_IMU_BASIC_H_

#include "math/AP_Math.h"

using namespace ap;

class IMU
{
protected:

  bool update_;
  Vector3f gyro_, acc_, mag_;
  Vector3f acc_bias_, gyro_bias_, mag_bias_, mag_scale_;

public:
  IMU() {}
  ~IMU() {}
  virtual void init()
  {
    acc_.zero();
    gyro_.zero();
    mag_.zero();
    acc_bias_.zero();
    gyro_bias_.zero();
    mag_bias_.zero();
    mag_scale_(1,1,1);
  }

  inline Vector3f getGyro() { return gyro_; }
  inline Vector3f getAcc() { return acc_; }
  inline Vector3f getMag() { return mag_; }

  bool getUpdate() {return update_;}
  void setUpdate(bool update) {update_ = update;}

  inline Vector3f getGyroBias() {return gyro_bias_;}
  inline Vector3f getAccBias() {return acc_bias_;}
  inline Vector3f getMagBias() {return mag_bias_;}
  inline Vector3f getMagScale() {return mag_scale_;}

  inline void setGyroBias(Vector3f data) { gyro_bias_ = data;}
  inline void setAccBias(Vector3f data) { acc_bias_ = data;}
  inline void setMagBias(Vector3f data) { mag_bias_ = data;}
  inline void setMagScale(Vector3f data) { mag_scale_ = data;}

  virtual void gyroCalib(bool flag, float duration) = 0;
  virtual void accCalib(bool flag, float duration) = 0;
  virtual void magCalib(bool flag, float duration) = 0;
  virtual void resetCalib() = 0;

  virtual void writeCalibData() = 0;
  virtual void update() = 0;
};


#endif /* APPLICATION_JSK_LIB_SENSORS_IMU_IMU_BASIC_H_ */
