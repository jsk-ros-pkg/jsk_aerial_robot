/*
******************************************************************************
* File Name          : pos_estimate.h
* Description        : pos estimate interface
******************************************************************************
*/

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#ifndef __POS_ESTIMATE_H
#define __POS_ESTIMATE_H

#include "config.h"
/* #include "arm_math.h" */
#include <ros.h>

/* sensors */
////////////////////////////////////////
//TODO: should include the super class//
////////////////////////////////////////
#include "sensors/imu/drivers/mpu9250/imu_mpu9250.h"
#include "sensors/imu/drivers/icm20948/icm_20948.h"
#include "sensors/gps/gps_ublox.h"

class PosEstimate
{
public:
  PosEstimate() {}
  ~PosEstimate(){}

  void  init(IMU* imu, GPS* gps, ros::NodeHandle* nh)
  {
    nh_ = nh;
    imu_ = imu;
    gps_ = gps;
  }

  void update()
  {
  }

private:
  ros::NodeHandle* nh_;

  IMU* imu_;
  GPS* gps_;
  //nav-ekf2
};

#endif
