#include <hydrus/torsion_estimator.h>

int main (int argc, char **argv)
{
  ros::init (argc, argv, "torsion_estimator");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  TorsionEstimator torsionEstimatorNode(nh, nh_private);
  ros::spin();
  return 0;
}
