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

#include <aerial_robot_estimation/sensor/base_plugin.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <kalman_filter/kf_pos_vel_acc_plugin.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Empty.h>
#include <tf2_ros/static_transform_broadcaster.h>


namespace sensor_plugin
{
  enum {ONLY_POS_MODE = 0, ONLY_VEL_MODE = 1, POS_VEL_MODE = 2,};

  class VisualOdometry :public sensor_plugin::SensorBase
  {
  public:

    VisualOdometry();
    ~VisualOdometry(){}

    void initialize(ros::NodeHandle nh,
                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                    string sensor_name, int index) override;

    inline const bool odomPosMode()
    {
      if(fusion_mode_ != ONLY_VEL_MODE) return true;
      else return false;
    }

    bool reset();

    const bool rotValid() const { return rot_valid_; }
    const tf::Transform& getRawBaselinkTF() const { return baselink_tf_; }

  private:
    /* ros */
    ros::Subscriber vo_sub_;

    /* ros param */
    double throttle_rate_;
    double level_pos_noise_sigma_;
    double z_pos_noise_sigma_;
    double vel_noise_sigma_;
    double vel_outlier_thresh_;
    double downwards_vo_min_height_;
    double downwards_vo_max_height_;
    int fusion_mode_;
    bool vio_mode_; // visual + inertial mode
    bool z_vel_mode_;
    bool local_vel_mode_;
    bool outdoor_no_vel_time_sync_; // very special flag for stereo cam such as zed mini, which has tricky behavier in outdoor mode
    /* heuristic sepecial flag for fusion */
    bool outdoor_;
    bool z_no_delay_;
    bool rot_valid_; // the estimated orientatin by VO is whether valid or not.

    tf::Transform world_offset_tf_; // ^{w}H_{w_vo}: transform from true world frame to the vo/vio world frame
    tf::Transform baselink_tf_; // ^{w}H_{b}: transform from true world frame to the baselink frame, but is estimated by vo/vio
    tf::Vector3 raw_global_vel_;

    double reference_timestamp_;
    aerial_robot_msgs::States vo_state_;

    tf2_ros::StaticTransformBroadcaster static_broadcaster_; // publish the transfrom between the work and vo frame

    void rosParamInit();
    void estimateProcess();
    void voCallback(const nav_msgs::Odometry::ConstPtr & vo_msg);
  };
};



