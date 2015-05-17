#include "jsk_quadcopter/quadcopter.h"

int main (int argc, char **argv)
{
  ros::init (argc, argv, "quadcopter");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  JskQuadcopter*  quadcopterNode = new JskQuadcopter(nh, nh_private);
  ros::spin ();
  delete quadcopterNode;
  return 0;
}






