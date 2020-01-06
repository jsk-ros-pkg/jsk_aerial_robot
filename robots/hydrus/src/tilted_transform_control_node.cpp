#include <hydrus/tilted_transform_control.h>

int main(int argc, char **argv)
{
  ros::init (argc, argv, "hydrus_tilted_transform_control");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  TiltedTransformController TiltedTransformController(nh, nh_private);
  ros::spin();
  return 0;
}
