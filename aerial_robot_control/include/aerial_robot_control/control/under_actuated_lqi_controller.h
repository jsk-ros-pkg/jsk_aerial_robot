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

#include <aerial_robot_control/control/under_actuated_controller.h>
#include <aerial_robot_control/control/utils/care.h>
#include <aerial_robot_control/LQIConfig.h>
#include <aerial_robot_msgs/FourAxisGain.h>
#include <dynamic_reconfigure/server.h>
#include <spinal/RollPitchYawTerms.h>
#include <spinal/PMatrixPseudoInverseWithInertia.h>
#include <ros/ros.h>
#include <thread>

namespace aerial_robot_control
{
  class UnderActuatedLQIController: public PoseLinearController
  {

  public:
    UnderActuatedLQIController();
    virtual ~UnderActuatedLQIController();

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                    boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                    double ctrl_loop_rate);

    void activate() override;

  protected:

    ros::Publisher flight_cmd_pub_; // for spinal
    ros::Publisher rpy_gain_pub_; // for spinal
    ros::Publisher four_axis_gain_pub_;
    ros::Publisher p_matrix_pseudo_inverse_inertia_pub_;

    bool verbose_;
    boost::shared_ptr<dynamic_reconfigure::Server<aerial_robot_control::LQIConfig> > lqi_server_;
    dynamic_reconfigure::Server<aerial_robot_control::LQIConfig>::CallbackType dynamic_reconf_func_lqi_;

    double target_roll_, target_pitch_;
    double candidate_yaw_term_;
    std::vector<float> target_base_thrust_;

    int lqi_mode_;
    bool clamp_gain_;
    Eigen::MatrixXd K_;

    Eigen::Vector3d lqi_roll_pitch_weight_, lqi_yaw_weight_, lqi_z_weight_;
    std::vector<double> r_; // matrix R

    std::vector<Eigen::Vector3d> pitch_gains_, roll_gains_, yaw_gains_, z_gains_;

    bool gyro_moment_compensation_;

    bool realtime_update_;
    std::thread gain_generator_thread_;

    //private functions
    virtual bool checkRobotModel();
    void resetGain() { K_ = Eigen::MatrixXd(); }

    virtual void rosParamInit();
    virtual void controlCore() override;

    virtual bool optimalGain();
    virtual void clampGain();
    virtual void publishGain();

    virtual void sendCmd() override;
    virtual void sendFourAxisCommand();

    virtual void allocateYawTerm();
    void cfgLQICallback(aerial_robot_control::LQIConfig &config, uint32_t level); //dynamic reconfigure

    void sendRotationalInertiaComp();

    void gainGeneratorFunc();
  };
};
