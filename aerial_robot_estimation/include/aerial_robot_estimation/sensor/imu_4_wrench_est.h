// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2021, JSK Lab
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

#include <aerial_robot_estimation/sensor/imu.h>
#include <geometry_msgs/AccelStamped.h>

using namespace Eigen;
using namespace std;

namespace sensor_plugin
{
class Imu4WrenchEst : public sensor_plugin::Imu
{
public:
  void initialize(ros::NodeHandle nh, boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                  boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator, string sensor_name,
                  int index) override;

  void setOmegaCogInCog(const tf::Vector3& omega_cog_in_cog)
  {
    boost::lock_guard<boost::mutex> lock(omega_mutex_);
    omega_cog_in_cog_ = omega_cog_in_cog;
  }

  void setVelCogInW(const tf::Vector3& vel_cog_in_w)
  {
    boost::lock_guard<boost::mutex> lock(vel_mutex_);
    vel_cog_in_w_ = vel_cog_in_w;
  }

  void setOmegaDotCogInCog(const tf::Vector3& omega_dot_cog_in_cog)
  {
    boost::lock_guard<boost::mutex> lock(omega_mutex_);
    omega_dot_cog_in_cog_ = omega_dot_cog_in_cog;
  }

  void setAccCogInCog(const tf::Vector3& acc_cog_in_cog)
  {
    boost::lock_guard<boost::mutex> lock(vel_mutex_);
    acc_cog_in_cog_ = acc_cog_in_cog;
  }

  tf::Vector3 getOmegaCogInCog()
  {
    boost::lock_guard<boost::mutex> lock(omega_mutex_);
    return omega_cog_in_cog_;
  }

  tf::Vector3 getVelCogInW()
  {
    boost::lock_guard<boost::mutex> lock(vel_mutex_);
    return vel_cog_in_w_;
  }

  tf::Vector3 getOmegaDotCogInCog()
  {
    boost::lock_guard<boost::mutex> lock(omega_mutex_);
    return omega_dot_cog_in_cog_;
  }

  tf::Vector3 getAccCogInCog()
  {
    boost::lock_guard<boost::mutex> lock(vel_mutex_);
    return acc_cog_in_cog_;
  }

protected:
  void ImuCallback(const spinal::ImuConstPtr& imu_msg) override;

  // semaphore
  boost::mutex omega_mutex_;
  boost::mutex vel_mutex_;

  // data
  tf::Vector3 vel_cog_in_w_;          // cog point, world frame
  tf::Vector3 acc_cog_in_cog_;        // cog point, cog frame, align with Imu Raw data

  tf::Vector3 omega_cog_in_cog_;      // cog point, cog frame
  tf::Vector3 omega_dot_cog_in_cog_;  // cog point, cog frame. Use dot means numerical derivative

  // publisher
  ros::Publisher pub_acc_;
};
};  // namespace sensor_plugin
