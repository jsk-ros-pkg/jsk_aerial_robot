// -*- mode: c++ -*-

#pragma once

#include <hydrus_xi/hydrus_xi_fully_actuated_robot_model.h>

using namespace aerial_robot_model;

class RollingRobotModel : public HydrusXiFullyActuatedRobotModel {
public:
  RollingRobotModel(bool init_with_rosparam = true,
                    bool verbose = false,
                    double fc_t_min_thre = 0,
                    double epsilon = 10);
  virtual ~RollingRobotModel() = default;

  std::vector<double> getGimbalNominalAngles(){return gimbal_nominal_angles_;}
  template <class T> T getContactPoint();
  template <class T> std::vector<T> getRotorsCoordFromCog();
  template <class T> std::vector<T> getLinksRotationFromCog();
  template <class T> std::vector<T> getRotorsOriginFromContactPoint();
  template <class T> std::vector<T> getRotorsNormalFromContactPoint();
  template <class T> T getInertiaContactPoint();
  void setCircleRadius(double radius) {circle_radius_ = radius;}

private:
  std::vector<KDL::Rotation> rotors_coord_rotation_from_cog_;
  std::vector<KDL::Rotation> links_rotation_from_cog_;
  std::vector<KDL::Vector> rotors_origin_from_contact_point_;
  std::vector<KDL::Vector> rotors_normal_from_contact_point_;
  KDL::Frame contact_point_;
  std::vector<double> gimbal_nominal_angles_;
  std::mutex rotors_coord_rotation_mutex_;
  std::mutex links_rotation_mutex_;
  std::string thrust_link_;
  double circle_radius_;
  KDL::RotationalInertia link_inertia_contact_point_;

protected:
  void updateRobotModelImpl(const KDL::JntArray& joint_positions) override;
};


template<> inline std::vector<KDL::Rotation> RollingRobotModel::getRotorsCoordFromCog()
{
  std::lock_guard<std::mutex> lock(rotors_coord_rotation_mutex_);
  return rotors_coord_rotation_from_cog_;
}

template<> inline std::vector<Eigen::Matrix3d> RollingRobotModel::getRotorsCoordFromCog()
{
  return aerial_robot_model::kdlToEigen(getRotorsCoordFromCog<KDL::Rotation>());
}

template<> inline std::vector<KDL::Rotation> RollingRobotModel::getLinksRotationFromCog()
{
  std::lock_guard<std::mutex> lock(links_rotation_mutex_);
  return links_rotation_from_cog_;
}

template<> inline std::vector<Eigen::Matrix3d> RollingRobotModel::getLinksRotationFromCog()
{
  return aerial_robot_model::kdlToEigen(getLinksRotationFromCog<KDL::Rotation>());
}

template<> inline std::vector<KDL::Vector> RollingRobotModel::getRotorsOriginFromContactPoint()
{
  return rotors_origin_from_contact_point_;
}

template<> inline std::vector<Eigen::Vector3d> RollingRobotModel::getRotorsOriginFromContactPoint()
{
  return aerial_robot_model::kdlToEigen(RollingRobotModel::getRotorsOriginFromContactPoint<KDL::Vector>());
}

template<> inline std::vector<KDL::Vector> RollingRobotModel::getRotorsNormalFromContactPoint()
{
  return rotors_normal_from_contact_point_;
}

template<> inline std::vector<Eigen::Vector3d> RollingRobotModel::getRotorsNormalFromContactPoint()
{
  return aerial_robot_model::kdlToEigen(RollingRobotModel::getRotorsNormalFromContactPoint<KDL::Vector>());
}

template<> inline KDL::Frame RollingRobotModel::getContactPoint()
{
  return contact_point_;
}

template<> inline geometry_msgs::TransformStamped RollingRobotModel::getContactPoint()
{
  return aerial_robot_model::kdlToMsg(RollingRobotModel::getContactPoint<KDL::Frame>());
}

template<> inline KDL::RotationalInertia RollingRobotModel::getInertiaContactPoint()
{
  return link_inertia_contact_point_;
}

template<> inline Eigen::Matrix3d RollingRobotModel::getInertiaContactPoint()
{
  return aerial_robot_model::kdlToEigen(RollingRobotModel::getInertiaContactPoint<KDL::RotationalInertia>());
}

