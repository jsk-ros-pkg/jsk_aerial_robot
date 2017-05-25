#include "aerial_robot_estimation/optical_flow.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lk_flow");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  OpticalFlow* optical_flow_node = new OpticalFlow(nh, nh_private);
  ros::spin();
  delete optical_flow_node;
  return 0;
}
