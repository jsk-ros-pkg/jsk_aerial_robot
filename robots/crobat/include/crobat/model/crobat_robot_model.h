// -*- mode: c++ -*-

#pragma once

#include <gimbalrotor/model/gimbalrotor_robot_model.h>

class CrobatRobotModel : public GimbalrotorRobotModel{
public:
  CrobatRobotModel(bool init_with_rosparam = true,
                    bool verbose = false,
                    double fc_t_min_thre = 0,
                    double epsilon = 10);
  ~CrobatRobotModel() = default;
};
