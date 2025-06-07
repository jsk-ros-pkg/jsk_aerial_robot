// -*- mode: c++ -*-

#pragma once

#include <aerial_robot_model/model/transformable_aerial_robot_model.h>

using namespace aerial_robot_model;

class GimbalrotorRobotModel : public transformable::RobotModel{
public:
  GimbalrotorRobotModel(bool init_with_rosparam = true,
                    bool verbose = false,
                    double fc_t_min_thre = 0,
                    double epsilon = 10);
  virtual ~GimbalrotorRobotModel() = default;

  template <class T> std::vector<T> getLinksRotationFromCog();
  template <class T> std::vector<T> getThrustCoordRot();

private:
  void updateRobotModelImpl(const KDL::JntArray& joint_positions) override;

  KDL::JntArray gimbal_processed_joint_;
  std::vector<KDL::Rotation> links_rotation_from_cog_;
  std::vector<KDL::Rotation> thrust_coords_rot_;
  std::mutex links_rotation_mutex_;
  std::mutex thrust_rotation_mutex_;

};

template<> inline std::vector<KDL::Rotation> GimbalrotorRobotModel::getLinksRotationFromCog()
{
  std::lock_guard<std::mutex> lock(links_rotation_mutex_);
  return links_rotation_from_cog_;
}

template<> inline std::vector<KDL::Rotation> GimbalrotorRobotModel::getThrustCoordRot()
{
  std::lock_guard<std::mutex> lock(thrust_rotation_mutex_);
  return thrust_coords_rot_;
}
