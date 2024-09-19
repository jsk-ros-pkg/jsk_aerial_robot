// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, JSK Lab
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

#include <hydrus/numerical_jacobians.h>
#include <dragon/model/hydrus_like_robot_model.h>

class DragonNumericalJacobian : public HydrusNumericalJacobian
{
public:
  DragonNumericalJacobian(ros::NodeHandle nh, ros::NodeHandle nhp, std::unique_ptr<aerial_robot_model::transformable::RobotModel> robot_model = std::make_unique<aerial_robot_model::transformable::RobotModel>(true));
  virtual ~DragonNumericalJacobian() = default;

  virtual bool checkJacobians() override;

  virtual bool checkRotorOverlapJacobian();
  virtual bool checkExternalWrenchCompensateThrustJacobian();
  virtual bool checkThrsutForceJacobian(std::vector<int> joint_indices = std::vector<int>()) override;

protected:

  bool check_rotor_overlap_;
  bool check_comp_thrust_;
  double rotor_overlap_diff_thre_;
  double comp_thrust_diff_thre_;

  Dragon::HydrusLikeRobotModel& getDragonRobotModel() const {return dynamic_cast<Dragon::HydrusLikeRobotModel&>(*robot_model_);}

  // numerical solution
  virtual const Eigen::MatrixXd thrustForceNumericalJacobian(std::vector<int> joint_indices) override;
  virtual const Eigen::MatrixXd jointTorqueNumericalJacobian(std::vector<int> joint_indices) override;
  virtual const Eigen::MatrixXd overlapNumericalJacobian();
  virtual const Eigen::MatrixXd compThrustNumericalJacobian();
};
