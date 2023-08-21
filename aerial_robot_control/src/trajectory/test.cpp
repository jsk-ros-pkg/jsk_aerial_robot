#include <ros/ros.h>
#include "aerial_robot_control/trajectory/trajectory_reference/polynomial_trajectory.hpp"


int main (int argc, char **argv)
{
  ros::init (argc, argv, "aeria_robot_base");
  agi::QuadState start_state;
  agi::QuadState end_state;

  std::shared_ptr test_ptr = std::make_shared<agi::MinSnapTrajectory>(start_state, end_state);

  ros::waitForShutdown();
  return 0;
}




