#include <aerial_robot_model/joint_state_publisher.h>

int main (int argc, char **argv)
{
  ros::init (argc, argv, "joint_state_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  JointStatePublisher*  jointStateNode = new JointStatePublisher(nh, nh_private);
  ros::spin ();
  delete jointStateNode;
  return 0;
}





