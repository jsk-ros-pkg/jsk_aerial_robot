// -*- mode: c++ -*-

#pragma once

#include <aerial_robot_model/transformable_aerial_robot_model.h>

using namespace aerial_robot_model;

class BirotorRobotModel : public aerial_robot_model::RobotModel {
public:
  BirotorRobotModel(bool init_with_rosparam = true,
                    bool verbose = false,
                    double fc_t_min_thre = 0,
                    double epsilon = 10);
  virtual ~BirotorRobotModel() = default;

  template <class T> std::vector<T> getRotorsCoordRot();

private:
  void updateRobotModelImpl(const KDL::JntArray& joint_positions) override;

  std::vector<KDL::Rotation> rotors_coord_rot_;
  std::mutex rotor_rotation_mutex_;
};


template<> inline std::vector<KDL::Rotation> BirotorRobotModel::getRotorsCoordRot()
{
  std::lock_guard<std::mutex> lock(rotor_rotation_mutex_);
  return rotors_coord_rot_;
}

template<> inline std::vector<Eigen::Matrix3d> BirotorRobotModel::getRotorsCoordRot()
{
  return aerial_robot_model::kdlToEigen(getRotorsCoordRot<KDL::Rotation>());
}
