// -*- mode: c++ -*-

#pragma once

#include <aerial_robot_model/model/transformable_aerial_robot_model.h>

using namespace aerial_robot_model;

class BeetleOmniRobotModel : public aerial_robot_model::RobotModel
{
public:
  BeetleOmniRobotModel(bool init_with_rosparam = true, bool verbose = false, double fc_t_min_thre = 0,
                       double epsilon = 10);
  ~BeetleOmniRobotModel() override = default;

protected:
  void updateRobotModelImpl(const KDL::JntArray& joint_positions) override;
};
