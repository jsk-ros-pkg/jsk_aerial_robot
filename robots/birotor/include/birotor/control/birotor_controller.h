// -*- mode: c++ -*-

#pragma once

#include <aerial_robot_control/control/under_actuated_controller.h>
namespace aerial_robot_control
{
  class BirotorController: public UnderActuatedController
  {
  public:
    BirotorController(){}
    virtual ~BirotorController() = default;
  };
};
