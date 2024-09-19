/*
******************************************************************************
* File Name          : altitude_estimate.h
* Description        : attitude estimate interface
******************************************************************************
*/

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#ifndef __ALTITUDE_ESTIMATE_H
#define __ALTITUDE_ESTIMATE_H

#include "config.h"
/*  #include "arm_math.h" */
#include <ros.h>
#include <spinal/Barometer.h>

/* sensors */
////////////////////////////////////////
//TODO: should include the super class//
////////////////////////////////////////
#include "sensors/imu/drivers/mpu9250/imu_mpu9250.h"
#include "sensors/imu/drivers/icm20948/icm_20948.h"
#include "sensors/baro/baro_ms5611.h"


class AltitudeEstimate
{
public:
  AltitudeEstimate(): baro_pub_("baro", &baro_msg_)
  {}
  ~AltitudeEstimate(){}

  const static uint8_t READING_STAGE = 0;
  const static uint8_t PROCESS_STAGE = 1;
  const static uint8_t PUBLISH_STAGE = 2;

  void  init(IMU* imu, Baro* baro, ros::NodeHandle* nh)
  {
    nh_ = nh;
    nh_->advertise(baro_pub_);

    imu_ = imu;
    baro_= baro;

    stage_ = READING_STAGE;
  }

  void update()
  {
    /* do nothing if baro is not updated */
    if(!baro_->getUpdate()) return;

    /* reading stage */
    if(stage_ == READING_STAGE)
      {
        stage_ = PROCESS_STAGE;
        return;
      }

    /* process stage */
    if(stage_ == PROCESS_STAGE)
      {
        /* altitude estimation */
        //nav-ekf
        stage_ = PUBLISH_STAGE;
        return;
      }

    /* publish stage */
    if(stage_ == PUBLISH_STAGE)
      {
        /* send message */
        publish();

        stage_ = READING_STAGE;
        baro_->setUpdate(false);
      }
  }

private:
  ros::NodeHandle* nh_;
  ros::Publisher baro_pub_;
  spinal::Barometer baro_msg_;

  IMU* imu_;
  Baro* baro_;
  //nav-ekf

  //* stage: seperate 1) senser reading, 2) nav-ekf, 3) publish
  //* so the delay from sensor reading to publish is 3ms
  uint8_t stage_;

  void publish()
  {
    baro_msg_.stamp = nh_->now();
    baro_msg_.pressure = baro_->getPressure();
    baro_msg_.temperature = baro_->getTemperature();
    baro_msg_.altitude = baro_->getAltitude();
    baro_pub_.publish(&baro_msg_);
  }

};

#endif
