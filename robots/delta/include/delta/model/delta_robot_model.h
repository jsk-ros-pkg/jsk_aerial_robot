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

  Eigen::MatrixXd calcWrenchMatrixOnCoG();
  Eigen::MatrixXd getFullWrenchAllocationMatrixFromCog();
  // const std::vector<double>& getCurrentJointAngles()
  // {
  //   return current_joint_angles_;
  // }
  const std::vector<double>& getCurrentGimbalAngles()
  {
    return current_gimbal_angles_;
  }

  int getRotorOnRigidFrameNum()
  {
    return rotor_on_rigid_frame_num_;
  }
  int getRotorOnSoftFrameNum()
  {
    return rotor_on_soft_frame_num_;
  }

private:
  ros::Subscriber rotor5_pose_sub_;
  ros::Subscriber body_pose_sub_;

  // mocap of rotor5
  KDL::Frame rotor5_pose_from_world_;
  KDL::Frame body_pose_from_world_;
  ros::Time rotor5_pose_update_time_;
  ros::Time body_pose_update_time_;
  Eigen::Vector3d prev_rotor5_origin = Eigen::Vector3d(0,0,0);
  Eigen::Vector3d prev_rotor5_normal = Eigen::Vector3d(0,0,0);

  std::mutex links_rotation_mutex_;
  std::mutex current_joint_angles_mutex_;
  std::mutex current_gimbal_angles_mutex_;

  std::vector<KDL::Rotation> links_rotation_from_cog_;
  std::vector<double> current_joint_angles_;
  std::vector<double> current_gimbal_angles_;

  int rotor_on_rigid_frame_num_;
  int rotor_on_soft_frame_num_;

  Eigen::MatrixXd getQMatForRotorsOnSoftFrame();
  void Rotor5MocapCallback(const geometry_msgs::PoseStamped& msg);
  void BodyMocapCallback(const geometry_msgs::PoseStamped& msg);

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
