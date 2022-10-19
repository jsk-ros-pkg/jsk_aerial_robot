// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, JSK Lab
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

#include <aerial_robot_control/flight_navigation.h>
#include <tiger/model/full_vectoring_robot_model.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Empty.h>

namespace aerial_robot_navigation
{
  namespace Tiger
  {
    class WalkNavigator : public BaseNavigator
    {
    public:
      WalkNavigator();
      ~WalkNavigator(){}

      void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                      boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                      boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator) override;

      void update() override;

      inline tf::Vector3 getTargetBaselinkPos() {return target_baselink_pos_;}
      inline tf::Vector3 getTargetBaselinkRpy() {return target_baselink_rpy_;}
      inline tf::Vector3 getTargetBaselinkVel() {return target_baselink_vel_;}
      inline sensor_msgs::JointState getTargetJointState() {return target_joint_state_;}
      inline std::vector<KDL::Rotation> getTargetLinkRots() {return target_link_rots_;}
      inline bool getLowerLegFlag() const { return lower_leg_flag_; }

    private:

      tf::Vector3 target_baselink_pos_;
      tf::Vector3 target_baselink_vel_;
      tf::Vector3 target_baselink_rpy_;

      std::vector<int> joint_index_map_;
      sensor_msgs::JointState target_joint_state_;
      std::vector<KDL::Frame> target_leg_ends_;
      std::vector<KDL::Rotation> target_link_rots_;

      int free_leg_id_; // start from 0: [0, leg_num -1]
      bool raise_leg_flag_;
      bool lower_leg_flag_;
      double raise_angle_;

      ros::Subscriber target_baselink_pos_sub_;
      ros::Subscriber target_baselink_delta_pos_sub_;
      ros::Subscriber raise_leg_sub_;
      ros::Subscriber lower_leg_sub_;
      ros::Publisher target_joint_angles_pub_;

      boost::shared_ptr<::Tiger::FullVectoringRobotModel> tiger_robot_model_;
      boost::shared_ptr<aerial_robot_model::RobotModel> robot_model_for_nav_;

      void halt() override;
      void reset() override;

      void rosParamInit() override;

      void targetBaselinkPosCallback(const geometry_msgs::Vector3StampedConstPtr& msg);
      void targetBaselinkDeltaPosCallback(const geometry_msgs::Vector3StampedConstPtr& msg);
      void raiseLegCallback(const std_msgs::UInt8ConstPtr& msg);
      void lowerLegCallback(const std_msgs::EmptyConstPtr& msg);

      // utils
      void setJointIndexMap();
      std::vector<double> getCurrentJointAngles();
    };
  };
};
