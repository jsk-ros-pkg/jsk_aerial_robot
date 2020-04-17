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

#include "stm32f7xx_hal.h"
#include "config.h"
/* #include "arm_math.h" */
#include <ros.h>
#include <spinal/Gps.h>
#include <spinal/GpsFull.h>

/* sensors */
////////////////////////////////////////
//TODO: should include the super class//
////////////////////////////////////////
#include "sensors/imu/imu_mpu9250.h"
#include "sensors/gps/gps_ublox.h"

class PosEstimate
{
public:
  PosEstimate(): gps_pub_("gps", &gps_msg_), gps_full_pub_("gps_full", &gps_full_msg_)
  {}
  ~PosEstimate(){}

  void  init(IMU* imu, GPS* gps, ros::NodeHandle* nh)
  {
    nh_ = nh;
    nh_->advertise(gps_pub_);
    nh_->advertise(gps_full_pub_);

    imu_ = imu;
    gps_ = gps;
  }

  void update()
  {
    if(gps_->getUpdate())
      {
        /* altitude estimation */
        //nav-ekf2

        /* send message */
        publish();
        gps_->setUpdate(false);
      }
  }

private:
  ros::NodeHandle* nh_;
  ros::Publisher gps_pub_, gps_full_pub_;
  spinal::Gps gps_msg_;
  spinal::GpsFull gps_full_msg_;

  IMU* imu_;
  GPS* gps_;
  //nav-ekf2

  void publish()
  {
    gps_full_msg_.stamp = nh_->now();
    gps_full_msg_.status = gps_->getGpsState().status; // fix status
    gps_full_msg_.year = gps_->getGpsState().utc_year;
    gps_full_msg_.month = gps_->getGpsState().utc_month;
    gps_full_msg_.hour = gps_->getGpsState().utc_hour;
    gps_full_msg_.day = gps_->getGpsState().utc_day;
    gps_full_msg_.min = gps_->getGpsState().utc_min;
    gps_full_msg_.sec = gps_->getGpsState().utc_sec;
    gps_full_msg_.nano = gps_->getGpsState().utc_nano;
    gps_full_msg_.time_valid = gps_->getGpsState().utc_valid;

    gps_full_msg_.location[0] = gps_->getGpsState().location.lat / 1e7L; // lat
    gps_full_msg_.location[1] = gps_->getGpsState().location.lng / 1e7L; // lng
    gps_full_msg_.h_acc = gps_->getGpsState().horizontal_accuracy; // lng

    gps_full_msg_.velocity[0] = gps_->getGpsState().velocity.x;
    gps_full_msg_.velocity[1] = gps_->getGpsState().velocity.y;
    gps_full_msg_.v_acc = gps_->getGpsState().speed_accuracy; // lng

    gps_full_msg_.sat_num = gps_->getGpsState().num_sats;

    //gps_full_pub_.publish(&gps_full_msg_); // reserve

    gps_msg_.stamp = gps_full_msg_.stamp;
    gps_msg_.location[0] = gps_full_msg_.location[0];
    gps_msg_.location[1] = gps_full_msg_.location[1];
    gps_msg_.velocity[0] = gps_full_msg_.velocity[0];
    gps_msg_.velocity[1] = gps_full_msg_.velocity[1];
    gps_msg_.sat_num = gps_full_msg_.sat_num;

    gps_pub_.publish(&gps_msg_);
  }
};

#endif
