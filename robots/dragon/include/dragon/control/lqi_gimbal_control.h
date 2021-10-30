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

#include <hydrus/hydrus_lqi_controller.h>
#include <dragon/model/hydrus_like_robot_model.h>
#include <dragon/dragon_navigation.h>
#include <aerial_robot_msgs/ApplyWrench.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <spinal/RollPitchYawTerm.h>

namespace aerial_robot_control
{
  class DragonLQIGimbalController : public HydrusLQIController
  {
  public:
    DragonLQIGimbalController();
    ~DragonLQIGimbalController(){}

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                    boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                    double ctrl_loop_rate) override;

    bool update() override;
    void reset() override
    {
      HydrusLQIController::reset();
    }

  private:
    ros::Publisher gimbal_control_pub_;
    ros::Publisher gimbal_target_force_pub_;
    ros::Subscriber att_control_feedback_state_sub_;
    ros::Subscriber extra_vectoring_force_sub_;

    void gimbalControl();
    void controlCore() override;
    void rosParamInit() override;
    void sendCmd() override;
    void allocateYawTerm() override {} // do nothing

    void attControlFeedbackStateCallback(const spinal::RollPitchYawTermConstPtr& msg);
    void extraVectoringForceCallback(const std_msgs::Float32MultiArrayConstPtr& msg);

    boost::shared_ptr<Dragon::HydrusLikeRobotModel> dragon_robot_model_;
    Eigen::MatrixXd P_xy_;

    bool gimbal_vectoring_check_flag_;
    bool add_lqi_result_;
    std::vector<double> lqi_att_terms_;
    std::vector<double> target_gimbal_angles_;

    double gimbal_roll_pitch_control_rate_thresh_;
    double gimbal_roll_pitch_control_p_det_thresh_;


    /* external wrench */
    ros::Subscriber add_external_wrench_sub_, clear_external_wrench_sub_;
    void addExternalWrenchCallback(const aerial_robot_msgs::ApplyWrench::ConstPtr& msg);
    void clearExternalWrenchCallback(const std_msgs::String::ConstPtr& msg);

    /* extra vectoring force (i.e., for grasping) */
    Eigen::VectorXd extra_vectoring_force_;

  };
};
