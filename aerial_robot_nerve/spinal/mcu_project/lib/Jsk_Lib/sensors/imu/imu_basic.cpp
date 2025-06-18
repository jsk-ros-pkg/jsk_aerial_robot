/*
 * imu_basic.cpp
 *
 *  Created on: 2016/11/01
 *      Author: anzai
 */

#include "imu_basic.h"
#include "flashmemory/flashmemory.h"

IMU::IMU()
{

}

void IMU::init()
{
  acc_.zero();
  gyro_.zero();
  mag_.zero();
  mag_outlier_counter_ = 0;
  gyro_bias_.zero();
  calib_gyro_ = true;
  calib_acc_ = false;
  calib_mag_ = false;
  gyro_calib_duration_ = GYRO_DEFAULT_CALIB_DURATION * 1000;
  acc_calib_duration_ = ACC_DEFAULT_CALIB_DURATION * 1000;
  mag_calib_duration_ = MAG_DEFAULT_CALIB_DURATION * 1000;
  gyro_calib_time_ = HAL_GetTick();
  acc_calib_time_ = HAL_GetTick();
  mag_calib_time_ = HAL_GetTick();
  gyro_calib_cnt_ = 0;
  acc_calib_cnt_ = 0;
  mag_calib_cnt_ = 0;
  mag_filtering_flag_ = false;
  raw_gyro_p_.zero();
  raw_acc_p_.zero();
  for (int i = 0; i < 3; i++) {
    FlashMemory::addValue(&(acc_bias_[i]), sizeof(float));
    FlashMemory::addValue(&(mag_bias_[i]), sizeof(float));
    FlashMemory::addValue(&(mag_scale_[i]), sizeof(float));
  }
}

void IMU::readCalibData()
{
  FlashMemory::read();
}

void IMU::writeCalibData()
{
  FlashMemory::erase();
  FlashMemory::write();
}

void IMU::update()
{
  updateRawData();
  setUpdate(true);
  process();

  /* LED indicator */
  ledOutput();
}

void IMU::process (void)
{
  /* gyro part */
  raw_gyro_= raw_gyro_adc_;
  if (calib_gyro_)
    {
      gyro_bias_ += raw_gyro_adc_;
      gyro_calib_cnt_++;

      if (gyro_calib_duration_ > 0 && HAL_GetTick() - gyro_calib_time_ >= gyro_calib_duration_)
        {
          gyro_bias_ /= (float)gyro_calib_cnt_;
          calib_gyro_ = false;
        }
    }
  else
    raw_gyro_ -= gyro_bias_;

  raw_gyro_p_ -= (raw_gyro_p_/GYRO_LPF_FACTOR);
  raw_gyro_p_ += raw_gyro_;
  gyro_ = (raw_gyro_p_/GYRO_LPF_FACTOR);

  /* acc part */
  raw_acc_ = raw_acc_adc_;
  if (calib_acc_)
    {
      acc_bias_ += raw_acc_adc_;
      acc_calib_cnt_++;

      if (acc_calib_duration_ > 0 && HAL_GetTick() - acc_calib_time_ >= acc_calib_duration_)
        {
          acc_bias_[0] /= (float)acc_calib_cnt_;
          acc_bias_[1] /= (float)acc_calib_cnt_;
          acc_bias_[2] =  acc_bias_[2]/(float)acc_calib_cnt_ - GRAVITY_MSS;
          calib_acc_ = false;
        }
    }
  else
    raw_acc_ -= acc_bias_;

  raw_acc_p_ -= (raw_acc_p_/ACC_LPF_FACTOR);
  raw_acc_p_ += raw_acc_;
  acc_ = (raw_acc_p_/ACC_LPF_FACTOR);


  /* mag part */
  if (calib_mag_)
    {
      /*
        simple calibration algorithm:
        https://github.com/kriswiner/MPU6050/wiki/Simple-and-Effective-Magnetometer-Calibration

        (option) min-least-square algorithm:
        https://github.com/juancamilog/calibrate_imu/blob/master/nodes/calibrate_imu.py
      */
      for (int i = 0; i < 3; i++)
        {
          if (raw_mag_adc_[i] < mag_min_[i]) mag_min_[i] = raw_mag_adc_[i];
          if (raw_mag_adc_[i] > mag_max_[i]) mag_max_[i] = raw_mag_adc_[i];
        }


      if (mag_calib_duration_ > 0 && HAL_GetTick() - mag_calib_time_ >= mag_calib_duration_)
        {
          mag_bias_ = (mag_min_ + mag_max_) / 2;

          Vector3f ellipsoid_rad = (mag_min_ - mag_max_) / 2;
          float avg_rad = (ellipsoid_rad[0] + ellipsoid_rad[1] + ellipsoid_rad[2])  / 3;
          mag_scale_[0] = avg_rad / ellipsoid_rad[0];
          mag_scale_[1] = avg_rad / ellipsoid_rad[1];
          mag_scale_[2] = avg_rad / ellipsoid_rad[2];
          calib_mag_ = false;
        }
    }
  /* transform coordinate */
  raw_mag_ = raw_mag_adc_ - mag_bias_; //bias
  raw_mag_[0] *= mag_scale_[0];
  raw_mag_[1] *= mag_scale_[1];
  raw_mag_[2] *= mag_scale_[2];

  /* filtering => because the magnetemeter generates too much outlier, not know the reason */
  if(mag_filtering_flag_)
    {
      bool mag_outlier_flag = false;
      for(int i = 0; i < 3; i++)
        {
          if(fabs(raw_mag_[i] - mag_[i]) > MAG_GENERAL_THRESH) mag_outlier_flag = true;
        }

      if(!mag_outlier_flag)
        {
          mag_ = raw_mag_;
          mag_outlier_counter_ = 0;
        }
      else
        {
          if(++mag_outlier_counter_ > MAG_OUTLIER_MAX_COUNT)
            {
              mag_outlier_counter_ = 0;
              mag_ = raw_mag_;
            }
        }
    }
  else
    {
      if(!raw_mag_adc_.is_zero())
        {//should notice that the raw_mag_adc may be 0 in the early stage
          mag_ = raw_mag_;
          mag_filtering_flag_  = true;
        }
    }

}

void IMU::gyroCalib(bool flag, float duration)
{
  if(flag)
    { // start re-calib
      gyro_bias_.zero();
      calib_gyro_ = true;
      gyro_calib_duration_ = duration * 1000;
      gyro_calib_time_ = HAL_GetTick();
      gyro_calib_cnt_ = 0;
    }
  else
    { // stop re-calib
      gyro_calib_duration_ = 100; // stop re-calib in 100ms
    }
}

void IMU::accCalib(bool flag, float duration)
{
  if(flag)
    { // start re-calib
      acc_bias_.zero();
      calib_acc_ = true;
      acc_calib_duration_ = duration * 1000;
      acc_calib_time_ = HAL_GetTick();
      acc_calib_cnt_ = 0;

      /* also calibrate gyro at the same time */
      gyroCalib(flag, duration);
    }
  else
    { // stop re-calib
      acc_calib_duration_ = 100; // stop re-calib in 100ms
      gyro_calib_duration_ = 100; // stop re-calib in 100ms
    }
}

void IMU::magCalib(bool flag, float duration)
{
  if(flag)
    { // start re-calib
      mag_bias_.zero();
      mag_min_ = Vector3f(1000,1000,1000);
      mag_max_ = Vector3f(-1000,-1000,-1000);
      mag_scale_ = Vector3f(1,1,1);
      calib_mag_ = true;
      mag_calib_duration_ = duration * 1000;
      mag_calib_time_ = HAL_GetTick();
      mag_calib_cnt_ = 0;
    }
  else
    { // stop re-calib
      mag_calib_duration_ = 100; // stop re-calib in 100ms
    }
}

void IMU::resetCalib()
{
  gyro_bias_.zero();
  acc_bias_.zero();
  mag_bias_.zero();
  mag_min_ = Vector3f(1000,1000,1000);
  mag_max_ = Vector3f(-1000,-1000,-1000);
  mag_scale_ = Vector3f(1,1,1);
}


