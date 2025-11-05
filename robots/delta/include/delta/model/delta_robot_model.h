// -*- mode: c++ -*-

#pragma once

#include <ros/ros.h>

#include <aerial_robot_model/model/transformable_aerial_robot_model.h>
#include <chrono>
#include <geometry_msgs/PoseArray.h>
#include <numeric>
#include <queue>
#include <std_msgs/Float32.h>

using namespace aerial_robot_model;

class DeltaRobotModel : public aerial_robot_model::transformable::RobotModel
{
public:
  DeltaRobotModel(bool init_with_rosparam = true, bool verbose = false, double fc_f_min_thre = 0,
                  double fc_t_min_thre = 0, double epsilon = 10);
  ~DeltaRobotModel()
  {
  }

  template <class T>
  std::vector<T> getLinksRotationFromCog();

  Eigen::MatrixXd getFullWrenchAllocationMatrixFromCog();
  const std::vector<double>& getCurrentJointAngles()
  {
    return current_joint_angles_;
  }
  const std::vector<double>& getCurrentGimbalAngles()
  {
    return current_gimbal_angles_;
  }

private:
  std::mutex links_rotation_mutex_;
  std::mutex current_joint_angles_mutex_;
  std::mutex current_gimbal_angles_mutex_;

  std::vector<KDL::Rotation> links_rotation_from_cog_;
  std::vector<KDL::Rotation> links_rotation_from_control_frame_;
  std::vector<KDL::Vector> rotors_origin_from_control_frame_;
  std::vector<KDL::Vector> rotors_normal_from_control_frame_;
  std::vector<KDL::Frame> links_center_frame_from_cog_;
  std::vector<double> current_joint_angles_;
  std::vector<double> current_gimbal_angles_;

protected:
  void updateRobotModelImpl(const KDL::JntArray& joint_positions) override;
};

template <>
inline std::vector<KDL::Rotation> DeltaRobotModel::getLinksRotationFromCog()
{
  std::lock_guard<std::mutex> lock(links_rotation_mutex_);
  return links_rotation_from_cog_;
}

template <>
inline std::vector<Eigen::Matrix3d> DeltaRobotModel::getLinksRotationFromCog()
{
  return aerial_robot_model::kdlToEigen(getLinksRotationFromCog<KDL::Rotation>());
}
