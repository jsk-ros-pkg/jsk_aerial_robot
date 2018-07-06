#include <hydrus_xi/hydrus_xi_lqi_controller.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hydrus_xi_lqi_controller");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  HydrusXiLqiController node(nh, nhp);
  ros::spin();
  return 0;
}
