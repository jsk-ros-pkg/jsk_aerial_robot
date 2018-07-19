#include <dragon/transform_control.h>

int main (int argc, char **argv)
{
  ros::init (argc, argv, "dragon_control");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  DragonTransformController transformControllerNode(nh, nh_private);
  ros::spin();
  return 0;
}
