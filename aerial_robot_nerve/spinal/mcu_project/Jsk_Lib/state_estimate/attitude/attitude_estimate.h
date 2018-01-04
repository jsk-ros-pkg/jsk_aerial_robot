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

#include "stm32f7xx_hal.h"
#include "config.h"
/* #include "arm_math.h" */
#include <math/AP_Math.h>

/* ros */
#include <ros.h>
#include <std_srvs/Trigger.h>
#include <aerial_robot_msgs/Imu.h>
#include <aerial_robot_base/DesireCoord.h>
#include <geometry_msgs/Vector3Stamped.h>

/* sensors */
////////////////////////////////////////
//TODO: should include the super class//
////////////////////////////////////////
#include "sensors/imu/imu_mpu9250.h"

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
  AttitudeEstimate():
    imu_pub_("imu", &imu_msg_),
	attitude_pub_("attitude", &attitude_msg_),
    desire_coord_sub_("/desire_coordinate", &AttitudeEstimate::desireCoordCallback, this ),
	test_rosservice_("/test_rosservice_server", &AttitudeEstimate::testRosseriveCallback,this),
	imu_list_(1),
	imu_weights_(1,1),
	pub_smoothing_gyro_flag_(false)
  {}
  ~AttitudeEstimate(){}

  void init(IMU* imu, ros::NodeHandle* nh)
  {
    nh_ = nh;
    nh_->advertise(imu_pub_);
    nh_->advertise(attitude_pub_);
    nh_->subscribe< ros::Subscriber<aerial_robot_base::DesireCoord, AttitudeEstimate> >(desire_coord_sub_);

    /* rosserive server test */
    nh_->advertiseService< std_srvs::Trigger::Request, std_srvs::Trigger::Response, AttitudeEstimate> (test_rosservice_);

    imu_list_[0] = imu;

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
    if(imu_list_[0]->getUpdate())
      {
        /* attitude estimation */
        if(!imu_list_[0]->getCalibrated()) return;

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

  /* send message via ros protocol */
  void publish()
  {
	/* imu data (default: body/board frame */
	uint32_t now_time = HAL_GetTick();
    if( now_time - last_imu_pub_time_ >= IMU_PUB_INTERVAL)
      {
        last_imu_pub_time_ = now_time;
        imu_msg_.stamp = nh_->now();
        for(int i = 0; i < 3 ; i ++)
          {
        	if(pub_smoothing_gyro_flag_)
        	{
        		imu_msg_.gyro_data[i] = estimator_->getSmoothAngular(Frame::BODY)[i];
        	}
        	else
        		imu_msg_.gyro_data[i] = estimator_->getAngular(Frame::BODY)[i];
            imu_msg_.mag_data[i] = estimator_->getMag(Frame::BODY)[i];
            imu_msg_.acc_data[i] = estimator_->getAcc(Frame::BODY)[i];

#if ESTIMATE_TYPE == COMPLEMENTARY
            imu_msg_.angles[i] = estimator_->getAttitude(Frame::BODY)[i]; // get the attitude at body frame
#endif
          }
        imu_pub_.publish(&imu_msg_);
      }

    	/* attitude data (default: cog/virtual frame) */
      if( now_time - last_attitude_pub_time_ >= ATTITUDE_PUB_INTERVAL)
      {
        last_attitude_pub_time_ = now_time;
        attitude_msg_.header.stamp = nh_->now();

#if ESTIMATE_TYPE == COMPLEMENTARY
        /* get the attitude at virtual frame */
        attitude_msg_.vector.x = estimator_->getAttitude(Frame::VIRTUAL).x;
        attitude_msg_.vector.y = estimator_->getAttitude(Frame::VIRTUAL).y;
        attitude_msg_.vector.z = estimator_->getAttitude(Frame::VIRTUAL).z;
#endif
        attitude_pub_.publish(&attitude_msg_);
      }

  }

  EstimatorAlgorithm* getEstimator() {return estimator_;}

  /* receive message via ros protocal */
  inline const Vector3f getAttitude(uint8_t frame)  { return estimator_->getAttitude(frame); }
  inline const Vector3f getAngular(uint8_t frame) { return estimator_->getAngular(frame); }
  inline const Vector3f getSmoothAngular(uint8_t frame) { return estimator_->getSmoothAngular(frame); }
  inline void setGyroSmoothFlag(bool flag) { pub_smoothing_gyro_flag_ = flag; }

  static const uint8_t IMU_PUB_INTERVAL = 5; //10-> 100Hz, 2 -> 500Hz
  static const uint8_t ATTITUDE_PUB_INTERVAL = 100; //100 -> 10Hz

private:
  ros::NodeHandle* nh_;
  ros::Publisher imu_pub_, attitude_pub_;

  aerial_robot_msgs::Imu imu_msg_;
  geometry_msgs::Vector3Stamped attitude_msg_;
  bool pub_smoothing_gyro_flag_;

  ros::Subscriber<aerial_robot_base::DesireCoord, AttitudeEstimate> desire_coord_sub_;
  ros::ServiceServer<std_srvs::Trigger::Request, std_srvs::Trigger::Response, AttitudeEstimate> test_rosservice_; /* test rosserive server without class */

  EstimatorAlgorithm* estimator_;
  std::vector< IMU* > imu_list_;
  std::vector< float > imu_weights_;

  uint32_t last_imu_pub_time_, last_attitude_pub_time_;

  void desireCoordCallback(const aerial_robot_base::DesireCoord& coord_msg)
  {
    estimator_->coordinateUpdate(coord_msg.roll, coord_msg.pitch, coord_msg.yaw);
  }

  /* rosserive test */
  void testRosseriveCallback(const std_srvs::Trigger::Request & req, std_srvs::Trigger::Response & res)
  {
	  res.success = true;
	  res.message = "aotti~";
  }

};
#endif
