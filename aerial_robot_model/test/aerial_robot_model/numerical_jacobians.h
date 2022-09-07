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

#include <aerial_robot_model/transformable_aerial_robot_model.h>
#include <spinal/DesireCoord.h>

namespace aerial_robot_model  {

  class NumericalJacobian
  {
  public:
    NumericalJacobian(ros::NodeHandle nh, ros::NodeHandle nhp, std::unique_ptr<transformable::RobotModel> robot_model = std::make_unique<transformable::RobotModel>(true));
    virtual ~NumericalJacobian() = default;

    virtual bool checkJacobians();

    virtual bool checkThrsutForceJacobian(std::vector<int> joint_indices = std::vector<int>());
    virtual bool checkJointTorqueJacobian(std::vector<int> joint_indices = std::vector<int>());
    virtual bool checkCoGMomentumJacobian(std::vector<int> joint_indices = std::vector<int>());
    virtual bool checkFeasibleControlJacobian(std::vector<int> joint_indices = std::vector<int>());

    const bool getInitialized() {return initialized_;}

  protected:
    ros::Subscriber joint_state_sub_;
    ros::Subscriber desire_coordinate_sub_;
    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;
    std::unique_ptr<aerial_robot_model::transformable::RobotModel> robot_model_;

    bool rostest_;

    double delta_;

    bool initialized_;
    bool check_thrust_force_;
    bool check_joint_torque_;
    bool check_cog_motion_;
    bool check_feasible_control_;

    double thrust_force_diff_thre_;
    double joint_torque_diff_thre_;
    double cog_vel_diff_thre_;
    double l_momentum_diff_thre_;
    double feasible_control_force_diff_thre_;
    double feasible_control_torque_diff_thre_;

    aerial_robot_model::transformable::RobotModel& getRobotModel() const { return *robot_model_; }

    void jointStateCallback(const sensor_msgs::JointStateConstPtr& state);
    void desireCoordinateCallback(const spinal::DesireCoordConstPtr& msg);

    //  numerical solution
    virtual const Eigen::MatrixXd thrustForceNumericalJacobian(std::vector<int> joint_indices);
    virtual const Eigen::MatrixXd jointTorqueNumericalJacobian(std::vector<int> joint_indices);
    virtual const std::vector<Eigen::MatrixXd> cogMomentumNumericalJacobian(std::vector<int> joint_indices);
    virtual const std::vector<Eigen::MatrixXd> feasibleControlNumericalJacobian(std::vector<int> joint_indices);
  };
};
