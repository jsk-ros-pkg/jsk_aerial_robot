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

#pragma once

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

/* for eigen cumputation */
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

/* kinematics */
#include <aerial_robot_model/transformable_aerial_robot_model.h>

/* util */
#include <thread>
#include <mutex>
#include <string>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <memory>


class HydrusRobotModel : public aerial_robot_model::RobotModel {
public:
  HydrusRobotModel(std::string baselink, std::string thrust_link, double stability_margin_thre, double p_det_thre, double f_max, double f_min, double m_f_rate, bool only_three_axis_mode = false, bool verbose = false);
  virtual ~HydrusRobotModel() = default;

  bool stabilityMarginCheck(bool verbose = false);
  //virtual bool overlapCheck(bool verbose = false) {return true;}
  bool modelling(bool verbose = false, bool control_verbose = false); //lagrange method

  /* static & stability */
  double getStabilityMargin() const
  {
    return stability_margin_;
  }
  Eigen::MatrixXd getP() const
  {
    return P_;
  }
  double getPdeterminant() const
  {
    return p_det_;
  }
  Eigen::VectorXd getOptimalHoveringThrust() const
  {
    return optimal_hovering_f_;
  }

  //////////////////////////////////////////////////////////////////////////////////////
  /* control */
  static constexpr uint8_t LQI_THREE_AXIS_MODE = 3;
  static constexpr uint8_t LQI_FOUR_AXIS_MODE = 4;

  uint8_t getLqiMode() const
  {
    return lqi_mode_;
  }
  void setLqiMode(uint8_t lqi_mode)
  {
    lqi_mode_ = lqi_mode;
  }
  void param2controller();
  bool hamiltonMatrixSolver(uint8_t lqi_mode);

private:
  double stability_margin_thre_;
  double p_det_thre_;
  double f_max_, f_min_;
  double m_f_rate_; //moment / force rate
  bool only_three_axis_mode_;

  double p_det_;
  double stability_margin_;
  int lqi_mode_;
  Eigen::VectorXd optimal_hovering_f_;
  Eigen::MatrixXd P_;
  Eigen::MatrixXd P_orig_pseudo_inverse_; // for compensation of cross term in the rotional dynamics
};
