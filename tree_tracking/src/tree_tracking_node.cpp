#include "tree_tracking/tree_tracking.h"

int main (int argc, char **argv)
{
  ros::init (argc, argv, "tree_detector");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  TreeTracking*  treeTrackingNode = new TreeTracking(nh, nh_private);
  ros::spin ();
  delete treeTrackingNode;
  return 0;
}
