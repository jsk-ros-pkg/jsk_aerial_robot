// -*- mode: c++ -*-

#pragma once

#include <aerial_robot_model/transformable_aerial_robot_model.h>

using namespace aerial_robot_model;

class TrirotorRobotModel : public aerial_robot_model::RobotModel{
public:
  TrirotorRobotModel(bool init_with_rosparam = true,
                    bool verbose = false,
                    double fc_t_min_thre = 0,
                    double epsilon = 10);
  virtual ~TrirotorRobotModel() = default;

  std::vector<Eigen::MatrixXd> getRotorMasks() const { return rotor_masks_;};
  template <class T> std::vector<T> getLinksRotationFromCog();

private:
  void updateRobotModelImpl(const KDL::JntArray& joint_positions) override;
  void calcThrustMask();

  std::vector<Eigen::MatrixXd> rotor_masks_;
  KDL::JntArray gimbal_processed_joint_;
  std::vector<KDL::Rotation> links_rotation_from_cog_;
  std::mutex links_rotation_mutex_;

};

template<> inline std::vector<KDL::Rotation> TrirotorRobotModel::getLinksRotationFromCog()
{
  std::lock_guard<std::mutex> lock(links_rotation_mutex_);
  return links_rotation_from_cog_;
}
