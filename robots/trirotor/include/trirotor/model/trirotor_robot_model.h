// -*- mode: c++ -*-

#pragma once

#include <aerial_robot_model/transformable_aerial_robot_model.h>

using namespace aerial_robot_model;

class TrirotorRobotModel : public aerial_robot_model::RobotModel {
public:
  TrirotorRobotModel(bool init_with_rosparam = true,
                    bool verbose = false,
                    double fc_t_min_thre = 0,
                    double epsilon = 10);
  virtual ~TrirotorRobotModel() = default;
};
