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

#include <hydrus/hydrus_robot_model.h>
#include <aerial_robot_model/eigen_utils.h>
#include <algorithm>
#include <cmath>
#include <time.h>

class HydrusXiRobotModel : public HydrusRobotModel {
public:
  HydrusXiRobotModel(bool init_with_rosparam,
                     bool verbose = false,
                     std::string baselink = std::string(""),
                     std::string thrust_link = std::string(""),
                     double stability_margin_thre = 0,
                     double p_det_thre = 0,
                     double f_max = 0,
                     double f_min = 0,
                     double m_f_rate = 0,
                     bool only_three_axis_mode = false);
  virtual ~HydrusXiRobotModel() = default;

  bool modelling(bool verbose = false, bool control_verbose = false) override;
  Eigen::MatrixXd calcWrenchAllocationMatrix();
  double getMFRate() {return m_f_rate_;}

  Eigen::MatrixXd getJacobian(const KDL::JntArray& joint_positions, std::string segment_name);
  inline Eigen::MatrixXd convertJacobian(const Eigen::MatrixXd& in);
  Eigen::MatrixXd getCOGJacobian() const {return cog_jacobian_;}
  std::vector<Eigen::MatrixXd> getUJacobian() const {return u_jacobian_;}
  std::vector<Eigen::MatrixXd> getVJacobian() const {return v_jacobian_;}
  std::vector<Eigen::MatrixXd> getPJacobian() const {return p_jacobian_;}
  std::vector<Eigen::MatrixXd> getQJacobian() const {return q_jacobian_;}
  Eigen::MatrixXd getLambdaJacobian() const {return lambda_jacobian_;}
  void updateJacobians(const sensor_msgs::JointState& joint_state);
  Eigen::MatrixXd calcQMatrix();
  Eigen::MatrixXd calcQMatrix(const std::vector<Eigen::Vector3d>& u, const std::vector<Eigen::Vector3d>& p, const std::map<int, int>& sigma);
  Eigen::MatrixXd calcStaticThrust();
  Eigen::MatrixXd calcStaticThrust(const Eigen::MatrixXd& q_inv);
  double calcUTripleProduct(int i, int j, int k);
  double calcVTripleProduct(int i, int j, int k);
  double calcTripleProduct(const Eigen::Vector3d& ui, const Eigen::Vector3d& uj, const Eigen::Vector3d& uk);
  Eigen::VectorXd getUTripleProductJacobian(int i, int j, int k) const { return u_triple_product_jacobian_.at(i).at(j).at(k); }
  Eigen::VectorXd getVTripleProductJacobian(int i, int j, int k) const { return v_triple_product_jacobian_.at(i).at(j).at(k); }
  Eigen::VectorXd calcFmin();
  Eigen::VectorXd calcTmin();
  Eigen::MatrixXd getFMinJacobian() const { return f_min_jacobian_; }
  Eigen::MatrixXd getTMinJacobian() const { return t_min_jacobian_; }
  Eigen::VectorXd getJointTorque() const {return joint_torque_;}
  Eigen::MatrixXd getJointTorqueJacobian() const {return joint_torque_jacobian_;}
  Eigen::VectorXd calcJointTorque(const sensor_msgs::JointState& joint_state);
  Eigen::VectorXd calcJointTorque(const KDL::JntArray& joint_positions, const std::vector<Eigen::Vector3d>& u, const std::map<int, int> sigma, const Eigen::VectorXd& static_thrust);
  std::vector<Eigen::Vector3d> calcV();

  Eigen::VectorXd calcJointTorque(const sensor_msgs::JointState& joint_state, Eigen::VectorXd static_thrust);
  Eigen::VectorXd calcJointTorque(const sensor_msgs::JointState& joint_state, Eigen::VectorXd static_thrust, std::vector<Eigen::Vector3d> u);

  KDL::JntArray convertMsgToKDL(const sensor_msgs::JointState& joint_state) {
    return jointMsgToKdl(joint_state);
  }

  Eigen::VectorXd getSecondDerivative(std::string segment_name, int joint_i, int joint_j);

private:
  int joint_num_;
  Eigen::VectorXd gravity_;
  Eigen::VectorXd gravity_3d_;
  double epsilon_;
  std::vector<Eigen::MatrixXd> u_jacobian_; //thrust direction vector index:rotor
  std::vector<Eigen::MatrixXd> v_jacobian_; //thrust torque direction vector index:rotor
  std::vector<Eigen::MatrixXd> p_jacobian_; //thrust position index:rotor
  std::vector<Eigen::MatrixXd> q_jacobian_; //allocation matrix index:rotor
  Eigen::MatrixXd cog_jacobian_; //cog jacobian
  Eigen::MatrixXd lambda_jacobian_; //thrust force
  std::vector<std::vector<std::vector<Eigen::VectorXd> > > u_triple_product_jacobian_;
  std::vector<std::vector<std::vector<Eigen::VectorXd> > > v_triple_product_jacobian_;
  Eigen::MatrixXd f_min_jacobian_; //min force
  Eigen::MatrixXd t_min_jacobian_; //min torque
  Eigen::VectorXd joint_torque_;
  Eigen::MatrixXd joint_torque_jacobian_;
  std::vector<std::string> joint_names_;
  std::map<std::string, int> joint_hierachy_;

  std::map<std::string, std::vector<std::string> > joint_segment_map_;
  void makeJointThrustMap();
  void calcCOGJacobian();
  void makeJointSegmentMap();
  void jointSegmentSetupRecursive(const KDL::TreeElement& tree_element, std::vector<std::string> current_joints);

  Eigen::Matrix3d skew(const Eigen::Vector3d& vec);
  double reluApprox(double x);
  double sigmoid(double x);
  double absApprox(double x);
  double tanh(double x);
};
