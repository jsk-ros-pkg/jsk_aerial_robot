#ifndef JOINT_STATE_PUBLISHER_H
#define JOINT_STATE_PUBLISHER_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <string>

class JointStatePublisher
{
public:

JointStatePublisher (ros::NodeHandle nh, ros::NodeHandle nh_private);
~JointStatePublisher ();

void pubFunction(const ros::TimerEvent & e);

const static double degree = M_PI/180;

private :

ros::NodeHandle jointStateNodeHandle;
ros::NodeHandle jointStateNodeHandlePrivate;
ros::Timer  pubTimer;

ros::Publisher joint_pub;
tf::TransformBroadcaster broadcaster;
double pubLoopRate;

geometry_msgs::TransformStamped odom_trans;
sensor_msgs::JointState joint_state;
ros::Time startTime;

 std::string tf_prefix_;

 float delta_t_;
 double hydraMotion(double phase);

};



#endif
