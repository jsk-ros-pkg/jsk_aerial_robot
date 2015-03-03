#include "hydra/transform_control.h"

int main (int argc, char **argv)
{
  ros::init (argc, argv, "hydra_transform_control");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  TransformController*  transformControllerNode = new TransformController(nh, nh_private);
  ros::spin ();
  delete transformControllerNode;
  return 0;
}
