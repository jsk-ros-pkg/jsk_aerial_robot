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
  AttitudeEstimate()
  {
    tf_desired_coord_.setIdentity();
  }

  void init(ros::NodeHandle* nh)
  {
    nh_ = nh;
    imu_pub_ = nh_->advertise<spinal::Imu>("imu", 1);
    attitude_pub_ = nh_->advertise<geometry_msgs::Vector3Stamped>("attitude", 1),
    desire_coord_sub_ = nh_->subscribe("desire_coordinate", 1, &AttitudeEstimate::desireCoordCallback, this);

    last_imu_pub_time_ = HAL_GetTick();
    last_attitude_pub_time_ = HAL_GetTick();

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
    attitude_pub_("attitude", &attitude_msg_),
    desire_coord_sub_("desire_coordinate", &AttitudeEstimate::desireCoordCallback, this ),
    mag_declination_srv_("mag_declination", &AttitudeEstimate::magDeclinationCallback,this),
    imu_list_(1),
    imu_weights_(1,1)
  {}

  void init(IMU* imu, GPS* gps, ros::NodeHandle* nh)
  {
    nh_ = nh;
    nh_->advertise(imu_pub_);
    nh_->advertise(attitude_pub_);
    nh_->subscribe(desire_coord_sub_);
    nh_->advertiseService(mag_declination_srv_);

    imu_list_[0] = imu;
    gps_ = gps;

    last_imu_pub_time_ = HAL_GetTick();
    last_attitude_pub_time_ = HAL_GetTick();

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


    if(imu_list_[0]->getUpdate())
      {
        /* attitude estimation */
        Vector3f gyro, acc, mag;
        multiImuFusion(gyro, acc, mag);
        estimator_->update(gyro, acc, mag);

        /* send message to ros*/
        if(nh_->connected())  publish();

        /* reset update status of imu*/
        imu_list_[0]->setUpdate(false);
      }
  }

  void multiImuFusion(Vector3f& gyro, Vector3f& acc, Vector3f& mag )
  {
	  /* only gyro method */
	  Vector3f sum_gyro = Vector3f();
	  for(unsigned int i = 0; i < imu_list_.size(); i++)
	  {
		 sum_gyro += (imu_list_[i]->getGyro() * imu_weights_[i]);
	  }
	  gyro = sum_gyro;
	  acc = imu_list_[0]->getAcc();
	  mag = imu_list_[0]->getMag();
  }

  void addImu(IMU* imu, float weight)
  {
	  imu_list_.push_back(imu);
	  imu_weights_.push_back(weight);
  }

  void setImuWeight(uint8_t index, float weight)
  {
	  imu_weights_[index] = weight;
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
        for(int i = 0; i < 3 ; i ++)
          {
            imu_msg_.mag_data[i] = estimator_->getMag(Frame::BODY)[i];
            imu_msg_.acc_data[i] = estimator_->getAcc(Frame::BODY)[i];
            imu_msg_.gyro_data[i] = estimator_->getAngular(Frame::BODY)[i];
#if ESTIMATE_TYPE == COMPLEMENTARY
            imu_msg_.angles[i] = estimator_->getAttitude(Frame::BODY)[i]; // get the attitude at body frame3
#endif
          }
#ifdef SIMULATION
        imu_pub_.publish(imu_msg_);
#else
        imu_pub_.publish(&imu_msg_);
#endif
      }

    /* attitude data (default: cog/virtual frame) */
    if( now_time - last_attitude_pub_time_ >= ATTITUDE_PUB_INTERVAL)
      {
        last_attitude_pub_time_ = now_time;
#ifdef SIMULATION
        attitude_msg_.header.stamp = ros::Time::now();
#else
        attitude_msg_.header.stamp = nh_->now();
#endif

#if ESTIMATE_TYPE == COMPLEMENTARY
        /* get the attitude at virtual frame */
        attitude_msg_.vector.x = estimator_->getAttitude(Frame::VIRTUAL).x;
        attitude_msg_.vector.y = estimator_->getAttitude(Frame::VIRTUAL).y;
        attitude_msg_.vector.z = estimator_->getAttitude(Frame::VIRTUAL).z;
#endif
#ifdef SIMULATION
        attitude_pub_.publish(attitude_msg_);
#else
        attitude_pub_.publish(&attitude_msg_);
#endif
      }

  }

  EstimatorAlgorithm* getEstimator() {return estimator_;}

  /* receive message via ros protocal */
  inline const ap::Vector3f getAttitude(uint8_t frame)  { return estimator_->getAttitude(frame); }
  inline const ap::Vector3f getAngular(uint8_t frame) { return estimator_->getAngular(frame); }
  inline const ap::Vector3f getSmoothAngular(uint8_t frame) { return estimator_->getSmoothAngular(frame); }
  inline const ap::Matrix3f getDesiredCoord()  { return estimator_->getDesiredCoord(); }

  static const uint8_t IMU_PUB_INTERVAL = 5; //10-> 100Hz, 2 -> 500Hz
  static const uint8_t ATTITUDE_PUB_INTERVAL = 100; //100 -> 10Hz

private:
  ros::NodeHandle* nh_;
  ros::Publisher imu_pub_, attitude_pub_;

  spinal::Imu imu_msg_;
  geometry_msgs::Vector3Stamped attitude_msg_;

#ifdef SIMULATION
  ros::Subscriber desire_coord_sub_;
#else
  ros::Subscriber<spinal::DesireCoord, AttitudeEstimate> desire_coord_sub_;
#endif

  EstimatorAlgorithm* estimator_;
#ifndef SIMULATION
  std::vector< IMU* > imu_list_;
  GPS* gps_;
  std::vector< float > imu_weights_;

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
  tf::Matrix3x3 tf_desired_coord_;
  uint32_t HAL_GetTick(){ return ros::Time::now().toSec() * 1000; }
#endif

  uint32_t last_imu_pub_time_, last_attitude_pub_time_;

  void desireCoordCallback(const spinal::DesireCoord& coord_msg)
  {
    estimator_->coordinateUpdate(coord_msg.roll, coord_msg.pitch, coord_msg.yaw);
#ifdef SIMULATION
    /* bypass to tf::Matrix3x3 */
    tf_desired_coord_.setRPY(coord_msg.roll, coord_msg.pitch, coord_msg.yaw);
#endif
  }

#ifdef SIMULATION
public:
    inline const tf::Matrix3x3 getDesiredCoordTf()  { return tf_desired_coord_; }
#endif

};
#endif
