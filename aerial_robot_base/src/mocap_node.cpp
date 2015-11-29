#include <aerial_robot_base/sensor/mocap.h>

int main (int argc, char **argv)
{
  ros::init (argc, argv, "mocap_base");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  BasicEstimator* estimator = new BasicEstimator(nh, nh_private);
  MocapData*  mocapDataNode = new MocapData(nh, nh_private, estimator);
  ros::spin ();
  delete mocapDataNode;
  delete estimator;
  return 0;
}






