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
#include <eigen_conversions/eigen_msg.h>
#include <kdl_conversions/kdl_msg.h>

class DragonRobotModel : public HydrusRobotModel {
public:
  DragonRobotModel(bool init_with_rosparam,
                   bool verbose = false,
                   double wrench_margin_t_min_thre = 0,
                   double wrench_margin_roll_pitch_min_thre = 0,
                   double epsilon = 10,
                   double control_margin_thre = 0,
                   double wrench_mat_det_thre = 0,
                   double edf_radius = 0,
                   double edf_max_tilt = 0);
  virtual ~DragonRobotModel() = default;

  //public functions
  const double getEdfRadius() const {return edf_radius_;}
  const double getEdfMaxTilt() const {return edf_max_tilt_;}
  const std::vector<std::string>& getEdfNames() const { return edf_names_; }
  template <class T> std::vector<T> getEdfsOriginFromCog() const;
  std::vector<double> getGimbalNominalAngles() const { return gimbal_nominal_angles_; }
  template <class T> T getGimbalProcessedJoint() const;
  template <class T> std::vector<T> getLinksRotationFromCog() const;
  std::vector<int> getClosestRotorIndices();
  const double getClosestRotorDist() const {return min_dist_;}
  const Eigen::MatrixXd& getRotorOverlapJacobian() const { return rotor_overlap_jacobian_;}
  void updateRobotModelImpl(const KDL::JntArray& joint_positions) override;
  bool stabilityCheck(bool verbose = false) override;
  bool overlapCheck(bool verbose = false);

  Eigen::MatrixXd getJacobian(const KDL::JntArray& joint_positions, std::string segment_name, KDL::Vector offset = KDL::Vector::Zero()) override;
  void updateJacobians(const KDL::JntArray& joint_positions, bool update_model = true) override;
  const Eigen::MatrixXd& getCompThrustJacobian() const {return  comp_thrust_jacobian_; }
  void setCompThrustJacobian(const Eigen::MatrixXd comp_thrust_jacobian) { comp_thrust_jacobian_ = comp_thrust_jacobian; }

  // static wrench
  bool addExternalStaticWrench(const std::string wrench_name, const std::string reference_frame, const geometry_msgs::Point offset, const geometry_msgs::Wrench wrench);
  bool addExternalStaticWrench(const std::string wrench_name, const std::string reference_frame, const KDL::Vector offset, const KDL::Wrench wrench);
  bool addExternalStaticWrench(const std::string wrench_name, const std::string reference_frame, const KDL::Vector offset, const Eigen::VectorXd wrench);

  bool removeExternalStaticWrench(const std::string wrench_name);
  void resetExternalStaticWrench();
  void calcExternalWrenchCompThrust();

private:
  //private attributes
  double edf_max_tilt_;
  double edf_radius_;

  std::vector<double> gimbal_nominal_angles_;
  KDL::JntArray gimbal_processed_joint_;
  std::vector<KDL::Rotation> links_rotation_from_cog_;

  Eigen::MatrixXd gimbal_jacobian_;
  Eigen::MatrixXd rotor_overlap_jacobian_;

  // overlap
  std::vector<KDL::Vector> edfs_origin_from_cog_;
  std::vector<std::string> edf_names_;
  int rotor_i_, rotor_j_;
  double min_dist_;

  // external static wrench
  struct ExternalWrench
  {
    std::string frame;
    KDL::Vector offset;
    Eigen::VectorXd wrench;
  };
  std::map<std::string, ExternalWrench> external_wrench_map_;
  Eigen::VectorXd wrench_comp_thrust_;
  Eigen::VectorXd vectoring_thrust_;
  Eigen::MatrixXd vectoring_q_mat_;
  Eigen::MatrixXd comp_thrust_jacobian_;


  //private functions
  void getParamFromRos();

  void thrustForceNumericalJacobian(const KDL::JntArray joint_positions, Eigen::MatrixXd analytical_result = Eigen::MatrixXd(), std::vector<int> joint_indices = std::vector<int>()) override;
  void jointTorqueNumericalJacobian(const KDL::JntArray joint_positions, Eigen::MatrixXd analytical_result = Eigen::MatrixXd(), std::vector<int> joint_indices = std::vector<int>()) override;
  void overlapNumericalJacobian(const KDL::JntArray joint_positions, Eigen::MatrixXd analytical_result = Eigen::MatrixXd());

  void compThrustNumericalJacobian(const KDL::JntArray joint_positions, Eigen::MatrixXd analytical_result = Eigen::MatrixXd());

protected:
  void calcBasicKinematicsJacobian() override;
  void calcCoGMomentumJacobian() override;
  void calcRotorOverlapJacobian();
  void calcCompThrustJacobian();
  void addCompThrustToLambdaJacobian();
  void addCompThrustToJointTorqueJacobian();

};

template<> inline KDL::JntArray DragonRobotModel::getGimbalProcessedJoint() const
{
  return gimbal_processed_joint_;
}

template<> inline sensor_msgs::JointState DragonRobotModel::getGimbalProcessedJoint() const
{
  return kdlJointToMsg(gimbal_processed_joint_);
}

template<> inline std::vector<KDL::Rotation> DragonRobotModel::getLinksRotationFromCog() const
{
  return links_rotation_from_cog_;
}

template<> inline std::vector<Eigen::Matrix3d> DragonRobotModel::getLinksRotationFromCog() const
{
  return aerial_robot_model::kdlToEigen(links_rotation_from_cog_);
}

template<> inline std::vector<KDL::Vector> DragonRobotModel::getEdfsOriginFromCog() const
{
  return edfs_origin_from_cog_;
}

template<> inline std::vector<geometry_msgs::PointStamped> DragonRobotModel::getEdfsOriginFromCog() const
{
  return aerial_robot_model::kdlToMsg(edfs_origin_from_cog_);
}

template<> inline std::vector<Eigen::Vector3d> DragonRobotModel::getEdfsOriginFromCog() const
{
  return aerial_robot_model::kdlToEigen(edfs_origin_from_cog_);
}

template<> inline std::vector<tf2::Vector3> DragonRobotModel::getEdfsOriginFromCog() const
{
  return aerial_robot_model::kdlToTf2(edfs_origin_from_cog_);
}
