#include <aerial_robot_model/model/pinocchio_test.h>

int main(int argc, char ** argv)
{
  ros::init (argc, argv, "pinocchio_robot_model");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  PinocchioRobotModel* pinocchio_robot_model = new PinocchioRobotModel(nh, nh_private);
  ros::spin();

  delete pinocchio_robot_model;
  return 0;
}
