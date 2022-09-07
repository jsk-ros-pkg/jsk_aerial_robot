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

#include <aerial_robot_model/aerial_robot_model.h>

namespace aerial_robot_model {
  namespace transformable {

    //Transformable Aerial Robot Model
    class RobotModel : public ::aerial_robot_model::RobotModel {
    public:
      RobotModel(bool init_with_rosparam = true, bool verbose = false, double fc_f_min_thre = 0, double fc_t_min_thre = 0, double epsilon = 10.0);
      virtual ~RobotModel() = default;

      virtual void updateJacobians();
      virtual void updateJacobians(const KDL::JntArray& joint_positions, bool update_model = true);

      // kinematics
      const std::vector<std::string>& getLinkJointNames() const { return link_joint_names_; }
      const std::vector<int>& getLinkJointIndices() const { return link_joint_indices_; }
      const std::vector<double>& getLinkJointLowerLimits() const { return link_joint_lower_limits_; }
      const std::vector<double>& getLinkJointUpperLimits() const { return link_joint_upper_limits_; }
      const double getLinkLength() const { return link_length_; }

      // statics (static thrust, joint torque)
      virtual void calcJointTorque(const bool update_jacobian = true);

      // stability
      const Eigen::VectorXd& getApproxFeasibleControlFDists() const {return approx_fc_f_dists_;}
      const Eigen::VectorXd& getApproxFeasibleControlTDists() const {return approx_fc_t_dists_;}

      // jacobian
      virtual void calcBasicKinematicsJacobian();
      virtual void calcCoGMomentumJacobian();
      const Eigen::MatrixXd& getCOGJacobian() const {return cog_jacobian_;}
      const std::vector<Eigen::MatrixXd>& getCOGCoordJacobians() const {return cog_coord_jacobians_;}
      const Eigen::MatrixXd& getLMomentumJacobian() const {return l_momentum_jacobian_;}
      const std::vector<Eigen::MatrixXd>& getPJacobians() const {return p_jacobians_;}
      const std::vector<Eigen::MatrixXd>& getThrustCoordJacobians() const {return thrust_coord_jacobians_;}
      const std::vector<Eigen::MatrixXd>& getUJacobians() const {return u_jacobians_;}
      virtual void calcLambdaJacobian();
      virtual void calcJointTorqueJacobian();
      const Eigen::VectorXd& getJointTorque() const {return joint_torque_;}
      const Eigen::MatrixXd& getJointTorqueJacobian() const {return joint_torque_jacobian_;}
      const Eigen::MatrixXd& getLambdaJacobian() const {return lambda_jacobian_;}
      virtual void calcFeasibleControlJacobian();
      const Eigen::MatrixXd& getFeasibleControlFDistsJacobian() const { return fc_f_dists_jacobian_; }
      const Eigen::MatrixXd& getFeasibleControlTDistsJacobian() const { return fc_t_dists_jacobian_; }

      // utils
      virtual Eigen::MatrixXd convertJacobian(const Eigen::MatrixXd& in);
      virtual Eigen::MatrixXd getJacobian(const KDL::JntArray& joint_positions, std::string segment_name, KDL::Vector offset = KDL::Vector::Zero());
      Eigen::MatrixXd getSecondDerivative(std::string ref_frame, int joint_i, KDL::Vector offset = KDL::Vector::Zero());
      Eigen::MatrixXd getSecondDerivativeRoot(std::string ref_frame, KDL::Vector offset = KDL::Vector::Zero());
      Eigen::VectorXd getHessian(std::string ref_frame, int joint_i, int joint_j, KDL::Vector offset = KDL::Vector::Zero());

    private:

      //private attributes

      // kinematics
      std::vector<std::string> link_joint_names_; // index in KDL::JntArray
      std::vector<int> link_joint_indices_; // index in KDL::JntArray
      std::vector<double> link_joint_lower_limits_, link_joint_upper_limits_;
      double link_length_;

      // statics
      Eigen::VectorXd joint_torque_;

      // stability
      Eigen::VectorXd approx_fc_f_dists_;
      Eigen::VectorXd approx_fc_t_dists_;

      // jacobians
      std::vector<Eigen::MatrixXd> u_jacobians_; //thrust direction vector index:rotor
      std::vector<Eigen::MatrixXd> p_jacobians_; //thrust position index:rotor
      Eigen::MatrixXd cog_jacobian_; //cog jacobian
      Eigen::MatrixXd l_momentum_jacobian_; //angular_momemtum jacobian
      std::vector<Eigen::MatrixXd> cog_coord_jacobians_;
      Eigen::MatrixXd joint_torque_jacobian_; // joint torque
      Eigen::MatrixXd lambda_jacobian_; //thrust force
      std::vector<Eigen::MatrixXd> thrust_coord_jacobians_;
      Eigen::MatrixXd fc_f_dists_jacobian_;
      Eigen::MatrixXd fc_t_dists_jacobian_;

      //private functions
      void resolveLinkLength();

    protected:

      void setJointTorque(const Eigen::VectorXd joint_torque) {joint_torque_ = joint_torque;}

      // Jacobians
      void setCOGCoordJacobians(const std::vector<Eigen::MatrixXd> cog_coord_jacobians) {cog_coord_jacobians_ = cog_coord_jacobians;}
      void setCOGJacobian(const Eigen::MatrixXd cog_jacobian) {cog_jacobian_ = cog_jacobian;}
      void setJointTorqueJacobian(const Eigen::MatrixXd joint_torque_jacobian) {joint_torque_jacobian_ = joint_torque_jacobian;}
      void setLambdaJacobian(const Eigen::MatrixXd lambda_jacobian) {lambda_jacobian_ = lambda_jacobian;}
      void setLMomentumJacobian(const Eigen::MatrixXd l_momentum_jacobian) {l_momentum_jacobian_ = l_momentum_jacobian;}
      void setPJacobians(const std::vector<Eigen::MatrixXd> p_jacobians) {p_jacobians_ = p_jacobians;}
      void setThrustTCoordJacobians(const std::vector<Eigen::MatrixXd> thrust_coord_jacobians) {thrust_coord_jacobians_ = thrust_coord_jacobians;}
      void setUJacobians(const std::vector<Eigen::MatrixXd> u_jacobians) {u_jacobians_ = u_jacobians;}
    };
  } // namespace transformable
} //namespace aerial_robot_model
