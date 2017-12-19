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
	acc_v_.zero();
	gyro_v_.zero();
	mag_v_.zero();
	mag_outlier_counter_ = 0;
	calibrate_gyro_ = CALIBRATING_STEP;
	calibrate_acc_ = 0;
	calibrate_mag_ = 0;
	mag_filtering_flag_ = false;
	virtual_frame_ = false;
	raw_gyro_p_.zero();
	raw_acc_p_.zero();
	for (int i = 0; i < 3; i++) {
		FlashMemory::addValue(&(acc_offset_[i]), sizeof(float));
		FlashMemory::addValue(&(mag_offset_[i]), sizeof(float));
	}
}

void IMU::ledOutput()
{
  if(calibrate_acc_ || calibrate_gyro_ || calibrate_mag_) LED0_L;
  else LED0_H;
}

void IMU::readCalibData()
{
	FlashMemory::read();
}

void IMU::writeCalibData()
{
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
  if (calibrate_gyro_ > 0)
    {
      if (calibrate_gyro_ == CALIBRATING_STEP)
        gyro_offset_.zero();

      gyro_offset_ += raw_gyro_adc_;
      if (calibrate_gyro_ == 1)
        {
          gyro_offset_ /= (float)CALIBRATING_STEP;
        }
      calibrate_gyro_--;
    }
  else
    {
      raw_gyro_= raw_gyro_adc_ - gyro_offset_;
      raw_gyro_p_  -= (raw_gyro_p_/GYRO_LPF_FACTOR);
      raw_gyro_p_   += raw_gyro_;
      gyro_  = (raw_gyro_p_/GYRO_LPF_FACTOR);
    }

  /* acc part */
  if (calibrate_acc_ > 0) {
    if (calibrate_acc_ == CALIBRATING_STEP) acc_offset_.zero();
    acc_offset_ += raw_acc_adc_;

    if (calibrate_acc_ == 1) {
      acc_offset_[0] /= (float)CALIBRATING_STEP;
      acc_offset_[1] /= (float)CALIBRATING_STEP;
      acc_offset_[2] =  acc_offset_[2]/(float)CALIBRATING_STEP - GRAVITY_MSS;
    }
    calibrate_acc_--;
  }
  else
    {
      raw_acc_ = raw_acc_adc_ - acc_offset_;
      raw_acc_p_    -= (raw_acc_p_/ACC_LPF_FACTOR);
      raw_acc_p_    += raw_acc_;
      acc_ = (raw_acc_p_/ACC_LPF_FACTOR);
    }

  /* mag part */
  if (calibrate_mag_ > 0)
    {
      if(calibrate_mag_  == CALIBRATING_MAG_STEP)
        {
          mag_offset_.zero();
          mag_min_ = raw_mag_adc_;
          mag_max_ = raw_mag_adc_;
        }
      // 30s: you have 30s to turn the multi in all directions
      for (int i = 0; i < 3; i++)
        {
          if (raw_mag_adc_[i] < mag_min_[i]) mag_min_[i] = raw_mag_adc_[i];
          if (raw_mag_adc_[i] > mag_max_[i]) mag_max_[i] = raw_mag_adc_[i];
        }
      // http://www.aichi-mi.com/old_pages/5_2_transistor_gijutu/transistor_gijutu.htm

      if(calibrate_mag_ == 1)
        {
          mag_offset_ = (mag_min_ + mag_max_) / 2;
        }
      calibrate_mag_ --;
    }
  else
    {
      /* transform coordinate */
	  raw_mag_ = raw_mag_adc_ - mag_offset_;

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
}


