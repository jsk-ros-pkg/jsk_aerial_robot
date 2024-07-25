// -*- mode: c++ -*-

#pragma once

#include <ros/ros.h>
#include <boost/thread/thread.hpp>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseArray.h>
#include <aerial_robot_model/model/transformable_aerial_robot_model.h>
#include <numeric>

using namespace aerial_robot_model;

class RollingRobotModel : public aerial_robot_model::transformable::RobotModel {
public:
  RollingRobotModel(bool init_with_rosparam = true,
                    bool verbose = false,
                    double fc_f_min_thre = 0,
                    double fc_t_min_thre = 0,
                    double epsilon = 10);
  ~RollingRobotModel()
  {
    contact_point_calc_thread_.interrupt();
    contact_point_calc_thread_.join();
  }

  void calcRobotModelFromFrame(std::string frame_name);
  template <class T> T getContactPoint();
  template <class T> std::vector<T> getLinksRotationFromCog();
  template <class T> std::vector<T> getLinksRotationFromControlFrame();
  template <class T> std::vector<T> getRotorsOriginFromControlFrame();
  template <class T> std::vector<T> getRotorsNormalFromControlFrame();
  std::vector<KDL::Frame> getLinksCenterFrameFromCog()
  {
    std::lock_guard<std::mutex> lock(links_center_frame_mutex_);
    return links_center_frame_from_cog_;
  }
  void setCircleRadius(double radius) {circle_radius_ = radius;}
  void setControlFrame(KDL::Frame frame){control_frame_ = frame;};
  void setControlFrame(std::string frame_name);
  KDL::Frame getControlFrame(){return control_frame_;}
  std::string getControlFrameName() {return control_frame_name_;}
  Eigen::MatrixXd getFullWrenchAllocationMatrixFromControlFrame();
  Eigen::MatrixXd getFullWrenchAllocationMatrixFromControlFrame(std::string frame_name);
  Eigen::MatrixXd getPlannedWrenchAllocationMatrixFromControlFrame();
  std::vector<double> getCurrentGimbalAngles() {return current_gimbal_angles_;}
  void setGimbalPlanningFlag(int index, int flag) {gimbal_planning_flag_.at(index) = flag;}
  std::vector<int> getGimbalPlanningFlag() {return gimbal_planning_flag_;}
  void setGimbalPlanningAngle(int index, double angle) {gimbal_planning_angle_.at(index) = angle;}
  std::vector<double> getGimbalPlanningAngle() {return gimbal_planning_angle_;}
  template <class T> T getInertiaFromControlFrame();

private:
  ros::Publisher feasible_control_force_pub_, feasible_control_torque_pub_;
  ros::Publisher feasible_control_force_radius_pub_, feasible_control_torque_radius_pub_;
  ros::Publisher rotor_origin_pub_;
  ros::Publisher rotor_normal_pub_;

  boost::thread contact_point_calc_thread_;

  std::mutex links_rotation_mutex_;
  std::mutex links_center_frame_mutex_;
  std::mutex rotors_normal_control_frame_mutex_;
  std::mutex rotors_origin_control_frame_mutex_;
  std::mutex contact_point_mutex_;
  std::mutex current_gimbal_angles_mutex_;

  KDL::Frame contact_point_;

  std::string thrust_link_;
  double circle_radius_;
  KDL::RigidBodyInertia link_inertia_; // from root
  KDL::Frame control_frame_;
  std::string control_frame_name_;
  std::map<std::string, KDL::Frame*> additional_frame_;

  std::vector<KDL::Rotation> links_rotation_from_cog_;
  std::vector<KDL::Rotation> links_rotation_from_control_frame_;
  std::vector<KDL::Vector> rotors_origin_from_control_frame_;
  std::vector<KDL::Vector> rotors_normal_from_control_frame_;
  std::vector<KDL::Frame> links_center_frame_from_cog_;
  std::vector<double> current_gimbal_angles_;

  /* gimbal planning */
  std::vector<int> gimbal_planning_flag_;
  std::vector<double> gimbal_planning_angle_;

  void calcContactPoint();

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

template<> inline std::vector<KDL::Rotation> RollingRobotModel::getLinksRotationFromControlFrame()
{
  std::lock_guard<std::mutex> lock(links_rotation_mutex_);
  return links_rotation_from_control_frame_;
}

template<> inline std::vector<Eigen::Matrix3d> RollingRobotModel::getLinksRotationFromControlFrame()
{
  return aerial_robot_model::kdlToEigen(getLinksRotationFromControlFrame<KDL::Rotation>());
}

template<> inline std::vector<KDL::Vector> RollingRobotModel::getRotorsNormalFromControlFrame()
{
  std::lock_guard<std::mutex> lock(rotors_normal_control_frame_mutex_);
  return rotors_normal_from_control_frame_;
}

template<> inline std::vector<Eigen::Vector3d> RollingRobotModel::getRotorsNormalFromControlFrame()
{
  return aerial_robot_model::kdlToEigen(RollingRobotModel::getRotorsNormalFromControlFrame<KDL::Vector>());
}

template<> inline std::vector<KDL::Vector> RollingRobotModel::getRotorsOriginFromControlFrame()
{
  std::lock_guard<std::mutex> lock(rotors_origin_control_frame_mutex_);
  return rotors_origin_from_control_frame_;
}

template<> inline std::vector<Eigen::Vector3d> RollingRobotModel::getRotorsOriginFromControlFrame()
{
  return aerial_robot_model::kdlToEigen(RollingRobotModel::getRotorsOriginFromControlFrame<KDL::Vector>());
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

template<> inline KDL::RotationalInertia RollingRobotModel::getInertiaFromControlFrame()
{
  KDL::Frame frame = getControlFrame();
  return (frame.Inverse() * link_inertia_).getRotationalInertia();
}

template<> inline Eigen::Matrix3d RollingRobotModel::getInertiaFromControlFrame()
{
  return aerial_robot_model::kdlToEigen(RollingRobotModel::getInertiaFromControlFrame<KDL::RotationalInertia>());
}
