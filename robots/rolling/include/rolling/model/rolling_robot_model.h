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
};
