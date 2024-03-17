#include <aerial_robot_model/model/pinocchio_robot_model_ros.h>

int main(int argc, char ** argv)
{
  ros::init (argc, argv, "pinocchio_robot_model");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  aerial_robot_model::PinocchioRobotModelRos* pinocchio_robot_model_ros = new aerial_robot_model::PinocchioRobotModelRos(nh, nh_private);
  ros::spin();

  delete pinocchio_robot_model_ros;
  return 0;
}
