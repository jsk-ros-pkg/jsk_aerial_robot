// -*- mode: c++ -*-

#pragma once

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseArray.h>
#include <aerial_robot_model/model/transformable_aerial_robot_model.h>

using namespace aerial_robot_model;

class RollingRobotModel : public aerial_robot_model::transformable::RobotModel {
public:
  RollingRobotModel(bool init_with_rosparam = true,
                    bool verbose = false,
                    double fc_f_min_thre = 0,
                    double fc_t_min_thre = 0,
                    double epsilon = 10);
  virtual ~RollingRobotModel() = default;

  void calcRobotModelFromFrame(std::string frame_name);
  template <class T> T getContactPoint();
  template <class T> std::vector<T> getLinksRotationFromCog();
  template <class T> T getCenterPoint();
  template <class T> T getCogToTargetFrame();
  template <class T> T getTargetToCogFrame();
  void setCircleRadius(double radius) {circle_radius_ = radius;}

  void setTargetFrameName(std::string frame_name);
  std::string getTargetFrameName() {return target_frame_name_;}
  KDL::Frame getTargetFrame();
  template <class T> std::vector<T> getLinksRotationFromTargetFrame();
  template <class T> std::vector<T> getRotorsOriginFromTargetFrame();
  template <class T> std::vector<T> getRotorsNormalFromTargetFrame();
  Eigen::MatrixXd getFullWrenchAllocationMatrixFromControlFrame();
  // Eigen::MatrixXd getWrenchAllocationMatrixFromTargetFrame() {return wrench_allocation_matrix_from_target_frame_;}
  template <class T> T getInertiaFromTargetFrame();
  // Eigen::Vector3d getGravityTorqueFromTargetFrame() {return gravity_torque_from_target_frame_;}

  Eigen::VectorXd calcFeasibleControlFDists(std::vector<Eigen::Vector3d>&u);
  Eigen::VectorXd calcFeasibleControlTDists(std::vector<Eigen::Vector3d>&v);

private:
  ros::Publisher feasible_control_force_pub_, feasible_control_torque_pub_;
  ros::Publisher feasible_control_force_radius_pub_, feasible_control_torque_radius_pub_;
  ros::Publisher rotor_origin_pub_;
  ros::Publisher rotor_normal_pub_;
  std::vector<KDL::Rotation> links_rotation_from_cog_;
  // std::vector<KDL::Vector> rotors_x_axis_from_cog_;
  // std::vector<KDL::Vector> rotors_y_axis_from_cog_;
  std::string thrust_link_;
  std::mutex links_rotation_mutex_;
  std::mutex links_rotation_target_frame_mutex_;
  std::mutex contact_point_mutex_;
  std::mutex center_point_mutex_;
  double circle_radius_;
  KDL::Frame center_point_;
  KDL::Frame contact_point_;

  /* robot model from target frame */
  std::map<std::string, KDL::Frame*> additional_frame_;
  std::string target_frame_name_;
  KDL::RigidBodyInertia link_inertia_; // from root

  // std::vector<KDL::Rotation> links_rotation_from_target_frame_;
  // KDL::RotationalInertia link_inertia_from_target_frame_;
  // std::vector<KDL::Vector> rotors_origin_from_target_frame_;
  // std::vector<KDL::Vector> rotors_normal_from_target_frame_;
  // Eigen::Vector3d gravity_torque_from_target_frame_;
  // KDL::Frame cog_to_target_frame_;
  // KDL::Frame target_to_cog_frame_;
  // Eigen::MatrixXd wrench_allocation_matrix_from_target_frame_;

protected:
  void updateRobotModelImpl(const KDL::JntArray& joint_positions) override;
};

template<> inline std::vector<KDL::Rotation> RollingRobotModel::getLinksRotationFromCog()
{
  std::lock_guard<std::mutex> lock(links_rotation_mutex_);
  return links_rotation_from_cog_;
}

template<> inline std::vector<Eigen::Matrix3d> RollingRobotModel::getLinksRotationFromCog()
{
  return aerial_robot_model::kdlToEigen(getLinksRotationFromCog<KDL::Rotation>());
}

// template<> inline std::vector<KDL::Rotation> RollingRobotModel::getLinksRotationFromTargetFrame()
// {
//   std::lock_guard<std::mutex> lock(links_rotation_target_frame_mutex_);
//   return links_rotation_from_target_frame_;
// }

// template<> inline std::vector<Eigen::Matrix3d> RollingRobotModel::getLinksRotationFromTargetFrame()
// {
//   return aerial_robot_model::kdlToEigen(getLinksRotationFromTargetFrame<KDL::Rotation>());
// }

// template<> inline std::vector<KDL::Vector> RollingRobotModel::getRotorsOriginFromTargetFrame()
// {
//   return rotors_origin_from_target_frame_;
// }

// template<> inline std::vector<Eigen::Vector3d> RollingRobotModel::getRotorsOriginFromTargetFrame()
// {
//   return aerial_robot_model::kdlToEigen(RollingRobotModel::getRotorsOriginFromTargetFrame<KDL::Vector>());
// }

// template<> inline std::vector<KDL::Vector> RollingRobotModel::getRotorsNormalFromTargetFrame()
// {
//   return rotors_normal_from_target_frame_;
// }

// template<> inline std::vector<Eigen::Vector3d> RollingRobotModel::getRotorsNormalFromTargetFrame()
// {
//   return aerial_robot_model::kdlToEigen(RollingRobotModel::getRotorsNormalFromTargetFrame<KDL::Vector>());
// }

template<> inline KDL::Frame RollingRobotModel::getContactPoint()
{
  std::lock_guard<std::mutex> lock(contact_point_mutex_);
  return contact_point_;
}

template<> inline geometry_msgs::TransformStamped RollingRobotModel::getContactPoint()
{
  return aerial_robot_model::kdlToMsg(RollingRobotModel::getContactPoint<KDL::Frame>());
}

template<> inline KDL::Frame RollingRobotModel::getCenterPoint()
{
  std::lock_guard<std::mutex> lock(center_point_mutex_);
  return center_point_;
}

template<> inline geometry_msgs::TransformStamped RollingRobotModel::getCenterPoint()
{
  return aerial_robot_model::kdlToMsg(RollingRobotModel::getCenterPoint<KDL::Frame>());
}

// template<> inline KDL::Frame RollingRobotModel::getCogToTargetFrame()
// {
//   return cog_to_target_frame_;
// }

// template<> inline geometry_msgs::TransformStamped RollingRobotModel::getCogToTargetFrame()
// {
//   return aerial_robot_model::kdlToMsg(RollingRobotModel::getCogToTargetFrame<KDL::Frame>());
// }

// template<> inline KDL::Frame RollingRobotModel::getTargetToCogFrame()
// {
//   return target_to_cog_frame_;
// }

// template<> inline geometry_msgs::TransformStamped RollingRobotModel::getTargetToCogFrame()
// {
//   return aerial_robot_model::kdlToMsg(RollingRobotModel::getTargetToCogFrame<KDL::Frame>());
// }

template<> inline KDL::RotationalInertia RollingRobotModel::getInertiaFromTargetFrame()
{
  KDL::Frame frame = getTargetFrame();
  return (frame.Inverse() * link_inertia_).getRotationalInertia();
}

template<> inline Eigen::Matrix3d RollingRobotModel::getInertiaFromTargetFrame()
{
  return aerial_robot_model::kdlToEigen(RollingRobotModel::getInertiaFromTargetFrame<KDL::RotationalInertia>());
}
