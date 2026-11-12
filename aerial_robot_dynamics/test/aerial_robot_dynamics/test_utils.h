#pragma once

#include <aerial_robot_dynamics/robot_model.h>

namespace aerial_robot_dynamics
{
PinocchioRobotModel& getTestRobotModel();

bool testVerbose();

int testTrials();
}  // namespace aerial_robot_dynamics
