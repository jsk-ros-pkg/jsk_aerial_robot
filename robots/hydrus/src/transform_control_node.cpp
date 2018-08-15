#include <hydrus/transform_control.h>

int main (int argc, char **argv)
{
  ros::init (argc, argv, "transform_control");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  TransformController transformControllerNode(nh, nh_private);
  ros::spin();
  return 0;
}
