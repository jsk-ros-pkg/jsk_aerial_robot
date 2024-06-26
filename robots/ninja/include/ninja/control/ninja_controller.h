// -*- mode: c++ -*-

#pragma once
#include <beetle/control/beetle_controller.h>

namespace aerial_robot_control
{
  class NinjaController: public BeetleController
  {
  public:
    NinjaController();
    ~NinjaController() = default;
  };
};
