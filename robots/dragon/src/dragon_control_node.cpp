#include <dragon/dragon_control.h>

int main (int argc, char **argv)
{
  ros::init (argc, argv, "dragon_control");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  DragonController*  dragonControllerNode = new DragonController(nh, nh_private);
  ros::spin ();
  delete dragonControllerNode;
  return 0;
}
