#include <di_control/di_tilt_control.h>

int main (int argc, char **argv)
{
  ros::init (argc, argv, "di_tilt_control");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  TiltControl*  tiltControlNode = new TiltControl(nh, nh_private);
  ros::spin ();
  delete tiltControlNode;
  return 0;
}

