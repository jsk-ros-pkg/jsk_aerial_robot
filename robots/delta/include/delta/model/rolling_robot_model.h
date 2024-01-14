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
  void setCircleRadius(double radius) {circle_radius_ = radius;}

  void setTargetFrameName(std::string frame_name);
  std::string getTargetFrameName() {return target_frame_name_;}
  KDL::Frame getTargetFrame();
  Eigen::MatrixXd getFullWrenchAllocationMatrixFromControlFrame();
  Eigen::MatrixXd getFullWrenchAllocationMatrixFromControlFrame(std::string frame_name);
  template <class T> T getInertiaFromTargetFrame();

  Eigen::VectorXd calcFeasibleControlFDists(std::vector<Eigen::Vector3d>&u);
  Eigen::VectorXd calcFeasibleControlTDists(std::vector<Eigen::Vector3d>&v);

private:
  ros::Publisher feasible_control_force_pub_, feasible_control_torque_pub_;
  ros::Publisher feasible_control_force_radius_pub_, feasible_control_torque_radius_pub_;
  ros::Publisher rotor_origin_pub_;
  ros::Publisher rotor_normal_pub_;

  std::mutex links_rotation_mutex_;
  std::mutex contact_point_mutex_;
  std::mutex center_point_mutex_;

  KDL::Frame center_point_;
  KDL::Frame contact_point_;

  std::vector<KDL::Rotation> links_rotation_from_cog_;
  std::string thrust_link_;
  double circle_radius_;
  KDL::RigidBodyInertia link_inertia_; // from root
  std::map<std::string, KDL::Frame*> additional_frame_;
  std::string target_frame_name_;

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

template<> inline KDL::RotationalInertia RollingRobotModel::getInertiaFromTargetFrame()
{
  KDL::Frame frame = getTargetFrame();
  return (frame.Inverse() * link_inertia_).getRotationalInertia();
}

template<> inline Eigen::Matrix3d RollingRobotModel::getInertiaFromTargetFrame()
{
  return aerial_robot_model::kdlToEigen(RollingRobotModel::getInertiaFromTargetFrame<KDL::RotationalInertia>());
}
