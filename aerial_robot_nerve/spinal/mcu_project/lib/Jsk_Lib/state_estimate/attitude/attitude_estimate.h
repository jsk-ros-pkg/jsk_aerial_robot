/*
******************************************************************************
* File Name          : attitude_estimate.h
* Description        : attitude estimate interface
******************************************************************************
*/

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#ifndef __ATTITUDE_ESTIMATE_H
#define __ATTITUDE_ESTIMATE_H

#ifndef SIMULATION
#include "config.h"
#include <math/AP_Math.h>

/* ros */
#include <ros.h>
#else
#include <ros/ros.h>
#endif

#include <spinal/Imu.h>
#include <spinal/DesireCoord.h>
#include <geometry_msgs/Vector3Stamped.h>

/* sensors */
#ifdef SIMULATION
#include <tf/LinearMath/Matrix3x3.h>
#else
#include "sensors/imu/drivers/mpu9250/imu_mpu9250.h"
#include "sensors/imu/drivers/icm20948/icm_20948.h"
#include "sensors/gps/gps_ublox.h"
#endif


/* estiamtor algorithm */
#include "state_estimate/attitude/complementary_ahrs.h"
//#include "state_estimate/attitude/madgwick_ahrs.h"

#include <vector>

#define COMPLEMENTARY 1
#define MADWICK 2
//#define MAHONY 3

/* please change the algorithm type according to your application */
#define ESTIMATE_TYPE COMPLEMENTARY

class AttitudeEstimate
{
public:
  ~AttitudeEstimate(){}
#ifdef SIMULATION
  AttitudeEstimate(): acc_(0,0,9.8), mag_(1,0,0){}

  void init(ros::NodeHandle* nh)
  {
    nh_ = nh;
    imu_pub_ = nh_->advertise<spinal::Imu>("imu", 1);

    last_imu_pub_time_ = HAL_GetTick();
    last_attitude_pub_time_ = HAL_GetTick();
    use_ground_truth_ = false;

#if ESTIMATE_TYPE == COMPLEMENTARY
    estimator_ = new ComplementaryAHRS();
#elif ESTIMATE_TYPE == MADWICK
    estimator_ = new MadgwickAHRS();
#else
#error "no instance for estimator"
#endif

  }

  void update()
  {
    /* attitude estimation */
    estimator_->update(gyro_, acc_, mag_);

    /* send message to ros*/
    publish();
  }

  void setMag(float x, float y, float z) { mag_.x = x; mag_.y = y; mag_.z = z; }
  void setGyro(float x, float y, float z) { gyro_.x = x; gyro_.y = y; gyro_.z = z; }
  void setAcc(float x, float y, float z) { acc_.x = x; acc_.y = y; acc_.z = z; }

#else
  AttitudeEstimate():
    imu_pub_("imu", &imu_msg_),
    mag_declination_srv_("mag_declination", &AttitudeEstimate::magDeclinationCallback,this)
  {}

  void init(IMU* imu, GPS* gps, ros::NodeHandle* nh)
  {
    nh_ = nh;
    nh_->advertise(imu_pub_);
    nh_->advertiseService(mag_declination_srv_);

    imu_ = imu;
    gps_ = gps;

    last_imu_pub_time_ = HAL_GetTick();
    last_attitude_pub_time_ = HAL_GetTick();
    use_ground_truth_ = false;

#if ESTIMATE_TYPE == COMPLEMENTARY
    estimator_ = new ComplementaryAHRS();
#elif ESTIMATE_TYPE == MADWICK
    estimator_ = new MadgwickAHRS();
#else
#error "no instance for estimator"
#endif
  }

  void update()
  {
    if(gps_)
      {
        if(gps_->getMagValid() && !estimator_->getMagDecValid())
          {
            /* update magnetic declination by GPS receive data */
            estimator_->setMagDeclination(gps_->getMagDeclination());
          }
      }

    if(imu_->getUpdate())
      {
        /* attitude estimation */
        estimator_->update(imu_->getGyro(), imu_->getAcc(), imu_->getMag());

        /* send message to ros*/
        if(nh_->connected())  publish();

        /* reset update status of imu*/
        imu_->setUpdate(false);
      }
  }
#endif

  /* send message via ros protocol */
  void publish()
  {
    /* imu data (default: body/board frame */
    uint32_t now_time = HAL_GetTick();
    if( now_time - last_imu_pub_time_ >= IMU_PUB_INTERVAL)
      {
        last_imu_pub_time_ = now_time;
#ifdef SIMULATION
        imu_msg_.stamp = ros::Time::now();
#else
        imu_msg_.stamp = nh_->now();
#endif
        // sensor values
        for(int i = 0; i < 3 ; i ++)
          {
            imu_msg_.mag[i] = estimator_->getMag()[i];
            imu_msg_.acc[i] = estimator_->getAcc()[i];
            imu_msg_.gyro[i] = estimator_->getAngular()[i];
          }

        // quaternion
        ap::Quaternion q = estimator_->getQuaternion();
        imu_msg_.quaternion[0] = q[1]; // x
        imu_msg_.quaternion[1] = q[2]; // y
        imu_msg_.quaternion[2] = q[3]; // z
        imu_msg_.quaternion[3] = q[0]; // w


#ifdef SIMULATION
        imu_pub_.publish(imu_msg_);
#else
        imu_pub_.publish(&imu_msg_);
#endif
      }
  }

  EstimatorAlgorithm* getEstimator() {return estimator_;}

  const ap::Matrix3f getRotation()
  {
    if (!use_ground_truth_) return estimator_->getRotation();
    else return ground_truth_rot_;
  }

  const ap::Vector3f getAngular()
  {
    if (!use_ground_truth_) return estimator_->getAngular();
    else return ground_truth_ang_vel_;
  }

  inline void useGroundTruth(bool flag) { use_ground_truth_ = flag; } // for simulation
  void setGroundTruthStates(ap::Matrix3f rot, ap::Vector3f ang_vel)
  {
    ground_truth_rot_ = rot;
    ground_truth_ang_vel_ = ang_vel;
  }

  static const uint8_t IMU_PUB_INTERVAL = 5; //10-> 100Hz, 2 -> 500Hz

private:
  ros::NodeHandle* nh_;
  ros::Publisher imu_pub_;
  spinal::Imu imu_msg_;

  EstimatorAlgorithm* estimator_;
#ifndef SIMULATION
  IMU* imu_;
  GPS* gps_;

  /* mag declination */
  ros::ServiceServer<spinal::MagDeclination::Request, spinal::MagDeclination::Response, AttitudeEstimate> mag_declination_srv_;

  void magDeclinationCallback(const spinal::MagDeclination::Request& req, spinal::MagDeclination::Response& res)
  {
    switch (req.command)
      {
      case spinal::MagDeclination::Request::GET_DECLINATION:
        {
          res.data = estimator_->getMagDeclination();
          res.success = true;
          break;
        }
      case spinal::MagDeclination::Request::SET_DECLINATION:
        {
          /* update the magnetic declination */
          estimator_->setMagDeclination(req.data);
          res.success = true;
          break;
        }
      default:
        {
          break;
        }
      }
  }

#else
  ap::Vector3f acc_, mag_, gyro_;
  uint32_t HAL_GetTick(){ return ros::Time::now().toSec() * 1000; }
#endif

  uint32_t last_imu_pub_time_, last_attitude_pub_time_;

  bool use_ground_truth_; // for simulation
  ap::Matrix3f ground_truth_rot_;
  ap::Vector3f ground_truth_ang_vel_;

};
#endif
