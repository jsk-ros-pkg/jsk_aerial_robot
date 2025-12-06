#include <ros/ros.h>
#include <hugmy/control/deformation_planning.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "deformation_planning");
  ros::NodeHandle nh("~");
  DeformationPlanning node(nh);
  node.spin();
  return 0;
}
