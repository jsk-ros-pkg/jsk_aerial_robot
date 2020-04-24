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

#include <aerial_robot_model/transformable_aerial_robot_model.h>
#include <aerial_robot_msgs/FourAxisGain.h>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/LU>
#include <iomanip>
#include <iostream>
#include <spinal/RollPitchYawTerms.h>
#include <spinal/PMatrixPseudoInverseWithInertia.h>
#include <ros/ros.h>
#include <tf_conversions/tf_kdl.h>
#include <tf_conversions/tf_eigen.h>

class HydrusRobotModel : public aerial_robot_model::RobotModel {
public:
  HydrusRobotModel(bool init_with_rosparam,
                   bool verbose = false,
                   double stability_margin_thre = 0,
                   double p_det_thre = 0,
                   bool only_three_axis_mode = false);
  virtual ~HydrusRobotModel() = default;

  //public attributes
  static constexpr uint8_t LQI_THREE_AXIS_MODE = 3;
  static constexpr uint8_t LQI_FOUR_AXIS_MODE = 4;

  //public functions
  virtual bool modelling(bool verbose = false, bool control_verbose = false); //lagrange method
  virtual bool overlapCheck(bool verbose = false) const {return true;}
  virtual bool stabilityMarginCheck(bool verbose = false);
  uint8_t getLqiMode() const { return lqi_mode_; }
  double getStabilityMargin() const { return stability_margin_; }
  Eigen::VectorXd getOptimalHoveringThrust() const { return optimal_hovering_f_; }
  Eigen::MatrixXd getQf() const { return Q_f_; }
  Eigen::MatrixXd getQtau() const { return Q_tau_; }
  Eigen::MatrixXd getP() const { return P_; }
  double getPdeterminant() const { return p_det_; }
  Eigen::MatrixXd getPOrigPseudoInverse() const { return P_orig_pseudo_inverse_; }
  bool hamiltonMatrixSolver(uint8_t lqi_mode);
  void setLqiMode(uint8_t lqi_mode) { lqi_mode_ = lqi_mode; }

  inline const double getPDetThresh() const {return p_det_thre_;}
  inline const double getStabilityMaginThresh() const {return stability_margin_thre_;}

protected:

  //private attributes
  int lqi_mode_;
  bool only_three_axis_mode_;
  Eigen::VectorXd optimal_hovering_f_;
  Eigen::MatrixXd Q_tau_, Q_f_;
  Eigen::MatrixXd P_;
  double p_det_;
  double p_det_thre_;
  Eigen::MatrixXd P_orig_pseudo_inverse_; // for compensation of cross term in the rotional dynamics
  double stability_margin_;
  double stability_margin_thre_;

  //private functions
  void getParamFromRos();
};
