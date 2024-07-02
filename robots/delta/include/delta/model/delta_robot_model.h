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
  template <class T> std::vector<T> getLinksPositionFromCog();
  template <class T> std::vector<T> getLinksRotationFromCog();
  std::vector<KDL::Frame> getLinksCenterFrameFromCog()
  {
    std::lock_guard<std::mutex> lock(links_center_frame_mutex_);
    return links_center_frame_from_cog_;
  }
  void setCircleRadius(double radius) {circle_radius_ = radius;}
  void setTargetFrame(KDL::Frame frame){target_frame_ = frame;};
  void setTargetFrame(std::string frame_name);
  KDL::Frame getTargetFrame(){return target_frame_;}
  std::string getTargetFrameName() {return target_frame_name_;}
  Eigen::MatrixXd getFullWrenchAllocationMatrixFromControlFrame();
  Eigen::MatrixXd getFullWrenchAllocationMatrixFromControlFrame(std::string frame_name);
  template <class T> T getInertiaFromTargetFrame();

private:
  ros::Publisher feasible_control_force_pub_, feasible_control_torque_pub_;
  ros::Publisher feasible_control_force_radius_pub_, feasible_control_torque_radius_pub_;
  ros::Publisher rotor_origin_pub_;
  ros::Publisher rotor_normal_pub_;

  std::mutex links_position_mutex_;
  std::mutex links_rotation_mutex_;
  std::mutex links_center_frame_mutex_;
  std::mutex contact_point_mutex_;

  KDL::Frame contact_point_;

  std::string thrust_link_;
  double circle_radius_;
  KDL::RigidBodyInertia link_inertia_; // from root
  KDL::Frame target_frame_;
  std::string target_frame_name_;
  std::map<std::string, KDL::Frame*> additional_frame_;

  std::vector<KDL::Vector> links_position_from_cog_;
  std::vector<KDL::Rotation> links_rotation_from_cog_;
  std::vector<KDL::Frame> links_center_frame_from_cog_;

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

template<> inline std::vector<KDL::Vector> RollingRobotModel::getLinksPositionFromCog()
{
  std::lock_guard<std::mutex> lock(links_position_mutex_);
  return links_position_from_cog_;
}

template<> inline std::vector<Eigen::Vector3d> RollingRobotModel::getLinksPositionFromCog()
{
  return aerial_robot_model::kdlToEigen(getLinksPositionFromCog<KDL::Vector>());
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

template<> inline KDL::RotationalInertia RollingRobotModel::getInertiaFromTargetFrame()
{
  KDL::Frame frame = getTargetFrame();
  return (frame.Inverse() * link_inertia_).getRotationalInertia();
}

template<> inline Eigen::Matrix3d RollingRobotModel::getInertiaFromTargetFrame()
{
  return aerial_robot_model::kdlToEigen(RollingRobotModel::getInertiaFromTargetFrame<KDL::RotationalInertia>());
}
