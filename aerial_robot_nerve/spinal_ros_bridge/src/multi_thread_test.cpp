#include <spinal_ros_bridge/multi_thread_test.h>

int main (int argc, char **argv)
{
  ros::init (argc, argv, "aeria_robot_base");
  ros::NodeHandle nh;
  MultiThreadTest*  multithreadTestNode = new MultiThreadTest(nh);

  delete multithreadTestNode;
  return 0;
}



