#include <di_control/di_gimbal_control.h>

int main (int argc, char **argv)
{
  ros::init (argc, argv, "di_gimbal_control");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  GimbalControl*  gimbalControlNode = new GimbalControl(nh, nh_private);
  ros::spin ();
  delete gimbalControlNode;
  return 0;
}

