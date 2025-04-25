#pragma once

#include <aerial_robot_dynamics/robot_model.h>

namespace aerial_robot_dynamics
{
class PinocchioRobotModelTest
{
public:
  PinocchioRobotModelTest()
  {
    robot_model_ = std::make_shared<PinocchioRobotModel>();
  }
  ~PinocchioRobotModelTest() = default;

  bool forwardDynamicsTest(bool verbose = false);
  bool forwardDynamicsDerivativesTest(bool verbose = false);
  bool inverseDynamicsTest(bool verbose = false);
  bool inverseDynamicsDerivativesTest(bool verbose = false);
  bool computeTauExtByThrustDerivativeQDerivativesTest(bool verbose = false);

private:
  std::shared_ptr<PinocchioRobotModel> robot_model_;
};
}  // namespace aerial_robot_dynamics
