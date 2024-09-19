// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#pragma once

#include <aerial_robot_msgs/Acc.h>
#include <aerial_robot_estimation/sensor/base_plugin.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <spinal/Imu.h>


using namespace Eigen;
using namespace std;

namespace sensor_plugin
{
  class Imu :public sensor_plugin::SensorBase
  {
  public:
    virtual void initialize(ros::NodeHandle nh,
                            boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                            boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                            string sensor_name, int index) override;

    ~Imu() {}
    Imu();

    inline tf::Vector3 getAttitude(uint8_t frame)  { return euler_; }
    inline ros::Time getStamp(){return imu_stamp_;}

    inline void treatImuAsGroundTruth(bool flag) { treat_imu_as_ground_truth_ = flag; }

  protected:
    ros::Publisher  acc_pub_;
    ros::Publisher  imu_pub_;
    ros::Subscriber imu_sub_;

    /* rosparam */
    string imu_topic_name_;
    string imu_pub_topic_name_;

    int calib_count_;
    double acc_scale_, gyro_scale_, mag_scale_; /* the scale of sensor value */
    double level_acc_noise_sigma_, z_acc_noise_sigma_, level_acc_bias_noise_sigma_, z_acc_bias_noise_sigma_, angle_bias_noise_sigma_; /* sigma for kf */

    /* sensor internal */
    double sensor_dt_;

    /* imu */
    tf::Vector3 euler_; /* the euler angle of both body and cog frame */
    tf::Vector3 omega_; /* the omega both body and cog frame */
    tf::Vector3 mag_; /* the magnetometer both body and cog frame */
    /* acc */
    tf::Vector3 acc_b_; /* the acceleration in baselink frame */
    tf::Vector3 acc_l_; /* the acceleration in level frame as to baselink frame: previously is acc_i */
    tf::Vector3 acc_w_; /* the acceleration in world frame */
    tf::Vector3 acc_non_bias_w_; /* the acceleration without bias in world frame */
    /* acc bias */
    tf::Vector3 acc_bias_b_; /* the acceleration bias in baselink frame, only use z axis  */
    tf::Vector3 acc_bias_l_; /* the acceleration bias in level frame as to baselink frame: previously is acc_i */
    tf::Vector3 acc_bias_w_; /* the acceleration bias in world frame */
    bool treat_imu_as_ground_truth_; /* whether use imu value as ground truth */

    aerial_robot_msgs::States state_; /* for debug */

    double calib_time_;

    ros::Time imu_stamp_;

    virtual void ImuCallback(const spinal::ImuConstPtr& imu_msg);
    virtual void estimateProcess();
    void publishAccData();
    void publishRosImuData();
    void rosParamInit();
  };
};





