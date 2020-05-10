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

class HydrusRobotModel : public aerial_robot_model::RobotModel {
public:
  HydrusRobotModel(bool init_with_rosparam,
                   bool verbose = false,
                   double wrench_margin_t_min_thre = 0,
                   double wrench_margin_roll_pitch_min_thre = 0,
                   double epsilon = 10,
                   int wrench_dof = 4,
                   double control_margin_thre = 0,
                   double wrench_mat_det_thre = 0);
  virtual ~HydrusRobotModel() = default;

  //public functions
  virtual void updateRobotModelImpl(const KDL::JntArray& joint_positions) override;
  virtual void updateJacobians(const KDL::JntArray& joint_positions, bool update_model = true) override;

  virtual void calcWrenchMatrixOnRoot() override;
  virtual void calcStaticThrust() override;

  virtual bool stabilityCheck(bool verbose = false) override;

  inline const uint8_t getWrenchDof() const { return wrench_dof_; }
  inline const double getControlMargin() const { return control_margin_; }
  inline const double getControlMarginThresh() const { return control_margin_thre_; }
  inline const double getWrenchMatDeterminant() const { return wrench_mat_det_; }
  inline const double getWrenchMatDetThresh() const {return wrench_mat_det_thre_;}
  const Eigen::MatrixXd& getWrenchMarginRollPitchJacobian() const {return wrench_margin_roll_pitch_min_jacobian_;}
  void setWrenchMarginRollPitchJacobian(const Eigen::MatrixXd wrench_margin_roll_pitch_min_jacobian) {wrench_margin_roll_pitch_min_jacobian_ = wrench_margin_roll_pitch_min_jacobian;}

  inline void setWrenchDof(uint8_t dof) { wrench_dof_ = dof; }

  void calcWrenchMarginRollPitch();
  void calcWrenchMarginRollPitchJacobian();

  inline const double& getWrenchMarginRollPitchMin()  {return wrench_margin_roll_pitch_min_;}
  inline const double& getWrenchMarginRollPitchMinThre()  {return wrench_margin_roll_pitch_min_thre_;}


protected:

  // private attributes
  int wrench_dof_;
  // following variables will be replaced by wrench_margin_t_min, wrench_margin_f_min in the furture
  double wrench_mat_det_;
  double wrench_mat_det_thre_;
  double control_margin_;
  double control_margin_thre_;

  Eigen::VectorXd approx_wrench_margin_roll_pitch_min_i_;
  Eigen::VectorXd wrench_margin_roll_pitch_min_i_;
  double wrench_margin_roll_pitch_min_;
  double wrench_margin_roll_pitch_min_thre_;

  Eigen::MatrixXd wrench_margin_roll_pitch_min_jacobian_;

  // private functions
  void getParamFromRos();

  // jacobian part
  virtual void thrustForceNumericalJacobian(const KDL::JntArray joint_positions, Eigen::MatrixXd analytical_result = Eigen::MatrixXd(), std::vector<int> joint_indices = std::vector<int>()) override;
  void wrenchMarginRollPitchNumericalJacobian(const KDL::JntArray joint_positions, Eigen::MatrixXd analytical_result = Eigen::MatrixXd(), std::vector<int> joint_indices = std::vector<int>());
};
