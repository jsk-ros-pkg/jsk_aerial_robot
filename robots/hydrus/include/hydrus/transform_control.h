// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, JSK Lab
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


#ifndef TRANSFORM_CONTROL_H
#define TRANSFORM_CONTROL_H

/* ros */
#include <ros/ros.h>

#include <spinal/RollPitchYawTerms.h>
#include <aerial_robot_msgs/FourAxisGain.h>
#include <sensor_msgs/JointState.h>
#include <aerial_robot_model/AddExtraModule.h>
#include <hydrus/TargetPose.h>
#include <spinal/DesireCoord.h>
#include <spinal/PMatrixPseudoInverseWithInertia.h>
#include <std_msgs/UInt8.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf_conversions/tf_kdl.h>
#include <tf_conversions/tf_eigen.h>

/* robot model */
#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/treefksolverpos_recursive.hpp>
#include <kdl/treejnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>

/* for eigen cumputation */
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

/* kinematics */
#include <hydrus/hydrus_robot_model.h>
#include <aerial_robot_model/transformable_aerial_robot_model_ros.h>

/* util */
#include <thread>
#include <mutex>
#include <string>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <memory>

/* for dynamic reconfigure */
#include <dynamic_reconfigure/server.h>
#include <hydrus/LQIConfig.h>
#define LQI_GAIN_FLAG 0
#define LQI_RP_P_GAIN 1
#define LQI_RP_I_GAIN 2
#define LQI_RP_D_GAIN 3
#define LQI_Y_P_GAIN 4
#define LQI_Y_I_GAIN 5
#define LQI_Y_D_GAIN 6
#define LQI_Z_P_GAIN 7
#define LQI_Z_I_GAIN 8
#define LQI_Z_D_GAIN 9


class TransformController : public aerial_robot_model::RobotModelRos {
public:
  TransformController(ros::NodeHandle nh, ros::NodeHandle nh_private);
  virtual ~TransformController();

protected:
  HydrusRobotModel& getRobotModel() const
  {
    return static_cast<HydrusRobotModel&>(RobotModelRos::getRobotModel());
  }

private:
  ros::NodeHandle nh_, nh_private_;

  bool control_verbose_;
  bool debug_verbose_;
  bool kinematic_verbose_;
  bool verbose_;

  double stability_margin_;
  double m_f_rate_; //moment / force rate
  std::string baselink_;
  std::string thrust_link_;
  double stability_margin_thre_;

  void param2controller();
  bool hamiltonMatrixSolver();

  /* ros param init */
  void initParam();

  /* publisher */
  ros::Publisher rpy_gain_pub_;
  ros::Publisher four_axis_gain_pub_;
  ros::Publisher p_matrix_pseudo_inverse_inertia_pub_;

  std::thread control_thread_;
  std::mutex mutex_;
  double control_rate_;
  bool gyro_moment_compensation_;

  Eigen::MatrixXd K_;
  //Q: 8/12:r,r_d, p, p_d, y, y_d, z. z_d, r_i, p_i, y_i, z_i
  //   6/9:r,r_d, p, p_d, z. z_d, r_i, p_i, z_i
  Eigen::VectorXd q_diagonal_;
  double q_roll_,q_roll_d_,q_pitch_,q_pitch_d_,q_yaw_,strong_q_yaw_, q_yaw_d_,q_z_,q_z_d_;
  double q_roll_i_,q_pitch_i_,q_yaw_i_,q_z_i_;
  bool a_dash_eigen_calc_flag_;
  std::vector<double> r_; // matrix R

  virtual void control();
  /* LQI parameter calculation */
  void lqi();

  //dynamic reconfigure
  dynamic_reconfigure::Server<hydrus::LQIConfig> lqi_server_;
  dynamic_reconfigure::Server<hydrus::LQIConfig>::CallbackType dynamic_reconf_func_lqi_;
  void cfgLQICallback(hydrus::LQIConfig &config, uint32_t level);
};


#endif
