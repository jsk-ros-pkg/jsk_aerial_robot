#include <delta/navigation/delta_navigation.h>

using namespace aerial_robot_model;
using namespace aerial_robot_navigation;

std::string RollingNavigator::indexToGroundNavigationModeString(int index)
{
  switch(index)
    {
    case aerial_robot_navigation::FLYING_STATE:
      return "FLYING_STATE";
      break;
    case aerial_robot_navigation::STANDING_STATE:
      return "STANDING_STATE";
      break;
    case aerial_robot_navigation::ROLLING_STATE:
      return "ROLLING_STATE";
      break;
    case aerial_robot_navigation::DOWN_STATE:
      return "DOWN_STATE";
    default:
      return "NONE";
      break;
    }
}

std::string RollingNavigator::indexToGroundMotionModeString(int index)
{
  switch(index)
    {
    case aerial_robot_navigation::LOCOMOTION_MODE:
      return "LOCOMOTION_MODE";
      break;
    case aerial_robot_navigation::MANIPULATION_MODE:
      return "MANIPULATION_MODE";
      break;
    default:
      return "NONE";
      break;
    }
}

