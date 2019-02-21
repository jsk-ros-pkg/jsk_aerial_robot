#include <hydrus_xi/hydrus_xi_quad_vectoring_controller.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "hydrus_xi_quad_vectoring_controller");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~");

  QuadVectoringController node(nh, nhp);
  ros::spin();
  return 0;
}

