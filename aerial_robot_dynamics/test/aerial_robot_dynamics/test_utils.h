#pragma once

#include <aerial_robot_dynamics/robot_model.h>

namespace aerial_robot_dynamics
{
// The robot model is built once from the ROS parameters and shared by every
// test file.  A test for a new algorithm therefore only needs its own TEST()
// and one line in test/CMakeLists.txt.
PinocchioRobotModel& getTestRobotModel();

// ~verbose: dump the compared quantities to the console
bool testVerbose();

// ~trials: how often a test redraws its random inputs
int testTrials();
}  // namespace aerial_robot_dynamics
