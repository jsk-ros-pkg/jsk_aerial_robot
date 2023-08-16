#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>

ros::Publisher timer_pub;
ros::Time zero_time;

void timerCallback(const ros::TimerEvent & e)
{
  rosgraph_msgs::Clock msg;
  msg.clock.fromSec((ros::Time::now() - zero_time).toSec());
  timer_pub.publish(msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "timer");
  ros::NodeHandle nh;
  timer_pub = nh.advertise<rosgraph_msgs::Clock>("/clock", 10);
  ros::Timer timer = nh.createTimer(ros::Duration(0.00100), timerCallback);
  zero_time = ros::Time::now();
  ros::spin();
  return 0;
}
