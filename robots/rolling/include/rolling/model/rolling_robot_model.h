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
  template <class T> std::vector<T> getRotorsCoordFromCog();
  template <class T> std::vector<T> getLinksRotationFromCog();

private:
  std::vector<KDL::Rotation> rotors_coord_rotation_from_cog_;
  std::vector<KDL::Rotation> links_rotation_from_cog_;
  std::vector<double> gimbal_nominal_angles_;
  std::mutex rotors_coord_rotation_mutex_;
  std::mutex links_rotation_mutex_;

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
