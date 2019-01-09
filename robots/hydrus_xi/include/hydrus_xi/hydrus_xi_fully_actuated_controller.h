// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, JSK Lab
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

#include <ros/ros.h>
#include <spinal/TorqueAllocationMatrixInv.h>
#include <spinal/SetAttitudeGains.h>
#include <spinal/FourAxisCommand.h>
#include <spinal/FlightConfigCmd.h>
#include <aerial_robot_msgs/WrenchAllocationMatrix.h>
#include <aerial_robot_msgs/FlatnessPid.h>
#include <aerial_robot_model/eigen_utils.h>
#include <aerial_robot_base/control/flight_control.h>
#include <aerial_robot_model/transformable_aerial_robot_model_ros.h>
#include <hydrus_xi/hydrus_xi_robot_model.h>
#include <hydrus_xi/FullyActuatedControllerGainsConfig.h>
#include <dynamic_reconfigure/server.h>
#include <tf_conversions/tf_eigen.h>
#include <cmath>
#include <angles/angles.h>

using boost::algorithm::clamp;

namespace control_plugin
{
  enum {
    GAIN_FLAG = 0,
    Z_P_GAIN = 1,
    Z_I_GAIN = 2,
    Z_D_GAIN = 3,
    XY_P_GAIN = 4,
    XY_I_GAIN = 5,
    XY_D_GAIN = 6,
    YAW_P_GAIN = 7,
    YAW_I_GAIN = 8,
    YAW_D_GAIN = 9,
    RP_P_GAIN = 10,
    RP_I_GAIN = 11,
    RP_D_GAIN = 12
  };

  class HydrusXiFullyActuatedController: public control_plugin::ControlBase, public aerial_robot_model::RobotModelRos
  {
  public:
    HydrusXiFullyActuatedController();
    virtual ~HydrusXiFullyActuatedController() = default;

    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                     BasicEstimator* estimator, Navigator* navigator,
                     double ctrl_loop_rate) override;
    bool update() override;
    void reset() override;
    void stateError();
    void pidUpdate();
    void sendCmd();

  private:
    HydrusXiRobotModel& getRobotModel() const { return static_cast<HydrusXiRobotModel&>(aerial_robot_model::RobotModelRos::getRobotModel()); }

    //state
    tf::Vector3 state_pos_;
    tf::Vector3 state_vel_;
    tf::Vector3 target_pos_;
    tf::Vector3 target_vel_;
    tf::Vector3 target_acc_;
    tf::Vector3 pos_err_;
    tf::Vector3 vel_err_;
    double state_yaw_;
    double state_yaw_vel_;
    double target_yaw_;
    double target_yaw_vel_;
    double yaw_err_;

    //pid
    //xy
    tf::Vector3 xy_gains_; //0:p 1:i 2:d
    tf::Vector3 xy_i_term_;
    double xy_limit_;
    tf::Vector3 xy_terms_limits_; //0:p 1:i 2:d
    tf::Vector3 xy_offset_;
    bool start_rp_integration_;
    double start_rp_integration_height_;

    //yaw
    tf::Vector3 yaw_gains_; //0:p 1:i 2:d
    double yaw_i_term_;
    double yaw_limit_;
    tf::Vector3 yaw_terms_limits_; //0:p 1:i 2:d
    double yaw_err_thresh_;
    bool need_yaw_d_control_;

    //altitude
    tf::Vector3 alt_gains_; //0:p 1:i 2:d
    double alt_takeoff_i_gain_;
    double alt_i_term_;
    double alt_limit_;
    tf::Vector3 alt_terms_limits_; //0:p 1:i 2:d
    double alt_err_thresh_;
    double alt_offset_;
    double alt_landing_const_i_ctrl_thresh_;

    //roll pitch (for spinal)
    double roll_pitch_limit_;
    tf::Vector3 roll_pitch_gains_;
    tf::Vector3 roll_pitch_terms_limits_;

    tf::Vector3 target_linear_acc_;
    double target_yaw_acc_;
    std::vector<float> calcForceVector(const Eigen::MatrixXd& wrench_allocation_matrix_inv);
    std::vector<double> target_throttle_;

    ros::Publisher flight_cmd_pub_;
    ros::Publisher pid_pub_;
    ros::Publisher torque_allocation_matrix_inv_pub_; //for spinal
    ros::Time torque_allocation_matrix_inv_pub_stamp_;
    ros::Publisher wrench_allocation_matrix_pub_; //for debug
    ros::Publisher wrench_allocation_matrix_inv_pub_; //for debug
    ros::Time wrench_allocation_matrix_pub_stamp_;
    ros::ServiceClient set_attitude_gains_client_; //for spinal

    double torque_allocation_matrix_inv_pub_interval_;
    double wrench_allocation_matrix_pub_interval_;
    bool verbose_;

    void setAttitudeGains();

    void rosParamInit();

    dynamic_reconfigure::Server<hydrus_xi::FullyActuatedControllerGainsConfig>::CallbackType dynamic_reconf_func_;
    dynamic_reconfigure::Server<hydrus_xi::FullyActuatedControllerGainsConfig> server_;
    void controllerGainsCfgCallback(hydrus_xi::FullyActuatedControllerGainsConfig &config, uint32_t level);

    tf::Vector3 clampV(tf::Vector3 input, double min, double max)
    {
      return tf::Vector3(clamp(input.x(), min, max),
                         clamp(input.y(), min, max),
                         clamp(input.z(), min, max));
    }
  };
} //namespace control_plugin

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(control_plugin::HydrusXiFullyActuatedController, control_plugin::ControlBase);
