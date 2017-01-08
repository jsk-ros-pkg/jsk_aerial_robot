#include "uav_comm/uav_comm.h"

int main (int argc, char **argv)
{
  ros::init (argc, argv, "uav_comm");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  UavComm*  UavCommNode = new UavComm(nh, nh_private);
  ros::spin ();
  delete UavCommNode;
  return 0;
}
