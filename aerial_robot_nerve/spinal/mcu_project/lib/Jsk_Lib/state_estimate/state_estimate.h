/*
******************************************************************************
* File Name          : state_estimate.h
* Description        : state(attitude, altitude, pos) estimate interface
******************************************************************************
*/

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#ifndef __STATE_ESTIMATE_H
#define __STATE_ESTIMATE_H

#ifndef SIMULATION
#include "config.h"
#include <ros.h>
#include <spinal/MagDeclination.h>

/* sensors */
#include "sensors/imu/drivers/mpu9250/imu_mpu9250.h"
#include "sensors/imu/drivers/icm20948/icm_20948.h"
#include "sensors/baro/baro_ms5611.h"
#include "sensors/gps/gps_ublox.h"
#endif

/* */
#include "state_estimate/attitude/attitude_estimate.h"
#ifndef SIMULATION
#include "state_estimate/altitude/altitude_estimate.h"
#include "state_estimate/pos/pos_estimate.h"
#endif

class StateEstimate
{
public:
  StateEstimate()
  {
  }
  ~StateEstimate(){}

#ifdef SIMULATION
  void  init(ros::NodeHandle* nh)
  {
    attitude_estimate_flag_ = true;
    attitude_estimator_.init(nh);
  }
#else
  void  init(IMU* imu, Baro* baro, GPS* gps, ros::NodeHandle* nh)
  {
    nh_ = nh;
    imu_ = imu;
    baro_ = baro;
    gps_ = gps;

    if(imu == NULL)
      {
        attitude_estimate_flag_ = false;
      }
    else
      {
        attitude_estimate_flag_ = true;
        attitude_estimator_.init(imu_, gps_, nh_);
      }

    if(baro == NULL)
      {
        altitude_estimate_flag_ = false;
      }
    else
      {
        altitude_estimate_flag_ = true;
        altitude_estimator_.init(imu_, baro_, nh_);
      }

    if(gps == NULL)
      {
        pos_estimate_flag_ = false;
      }
    else
      {
        pos_estimate_flag_ = true;
        pos_estimator_.init(imu_, gps_, nh_);
      }
  }
#endif

  void update()
  {
    if(attitude_estimate_flag_) attitude_estimator_.update();
#ifndef SIMULATION
    if(altitude_estimate_flag_) altitude_estimator_.update();
    if(pos_estimate_flag_) pos_estimator_.update();
#endif
  }

  AttitudeEstimate* getAttEstimator(){ return &attitude_estimator_;}
#ifndef SIMULATION
  AltitudeEstimate* getAltEstimator(){ return &altitude_estimator_;}
  PosEstimate* getPosEstimator(){ return &pos_estimator_;}

  IMU* getImu() {return imu_;}
  Baro* getBaro() {return baro_;}
  GPS* getGPS() {return gps_;}
#endif

private:
#ifndef SIMULATION
  ros::NodeHandle* nh_;

  IMU* imu_;
  Baro* baro_;
  GPS* gps_;
#endif

  AttitudeEstimate attitude_estimator_;
#ifndef SIMULATION
  AltitudeEstimate altitude_estimator_;
  PosEstimate pos_estimator_;
#endif

  bool attitude_estimate_flag_;
  bool altitude_estimate_flag_;
  bool pos_estimate_flag_;

};

#endif
