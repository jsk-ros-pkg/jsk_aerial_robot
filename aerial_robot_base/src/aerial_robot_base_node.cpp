#include "aerial_robot_base/aerial_robot_base.h"

int main (int argc, char **argv)
{
  ros::init (argc, argv, "aeria_robot_base");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  AerialRobotBase*  aerialRobotBaseNode = new AerialRobotBase(nh, nh_private);
  ros::waitForShutdown();

  delete aerialRobotBaseNode;
  return 0;
}






