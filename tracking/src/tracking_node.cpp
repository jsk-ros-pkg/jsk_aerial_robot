#include <tracking/tracking.h>

int main(int argc, char **argv)
{
  ros::init (argc, argv, "tracker");
  ros::NodeHandle node_handle;
  ros::NodeHandle node_handle_private("~");

  Tracking* tracker = new Tracking(node_handle, node_handle_private);

  ros::spin();

  delete tracker;
  return 0;
}
