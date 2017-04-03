#include "aerial_robot_comm/aerial_robot_comm.h"

int main (int argc, char **argv)
{
  ros::init (argc, argv, "aerial_robot_comm");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  UavComm*  UavCommNode = new UavComm(nh, nh_private);
  ros::spin ();
  delete UavCommNode;
  return 0;
}
