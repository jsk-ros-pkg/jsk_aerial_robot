#include <ros/ros.h>
#include <aerial_robot_msgs/States.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <eigen_conversions/eigen_msg.h>
#include "aerial_robot_control/trajectory/trajectory_reference/polynomial_trajectory.hpp"


int main (int argc, char **argv)
{
  ros::init (argc, argv, "aeria_robot_base");
  agi::QuadState start_state;
  agi::QuadState end_state;

  double du = 2; // 2 [sec]
  start_state.setZero();
  start_state.v = agi::Vector<3>(-1,0,0);
  end_state.setZero();
  end_state.p = agi::Vector<3>(1,2,3);
  end_state.v = agi::Vector<3>(0,0,0);
  end_state.t = du;

  // mediate points: bug
  // agi::QuadState med_state;
  // med_state.setZero();
  // med_state.p = agi::Vector<3>(0.2,-1,1);
  // med_state.t = du/2;

  std::vector<agi::QuadState> states;
  states.push_back(start_state);
  // states.push_back(med_state); // bug
  states.push_back(end_state);

  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<nav_msgs::Path>("trajectory", 1);
  ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 1);
  ros::Publisher twist_pub = nh.advertise<geometry_msgs::TwistStamped>("twist", 1);
  ros::Publisher acc_pub = nh.advertise<geometry_msgs::Vector3Stamped>("acc", 1);
  ros::Duration(0.5).sleep();

  //std::shared_ptr test_ptr = std::make_shared<agi::MinSnapTrajectory>(start_state, end_state);
  std::shared_ptr test_ptr = std::make_shared<agi::MinSnapTrajectory>(states);


  double dt = 0.02;

  nav_msgs::Path msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "world";

  for (double t = 0; t <= du; t += dt) {
    geometry_msgs::PoseStamped pose_stamp;
    pose_stamp.header.stamp = msg.header.stamp + ros::Duration(t);
    pose_stamp.header.frame_id = msg.header.frame_id;

    agi::QuadState state = test_ptr->getState(t);

    tf::pointEigenToMsg(state.p, pose_stamp.pose.position);
    tf::quaternionEigenToMsg(state.q(), pose_stamp.pose.orientation);
    msg.poses.push_back(pose_stamp);

    // debug
    pose_pub.publish(pose_stamp);

    geometry_msgs::TwistStamped twist_stamp;
    twist_stamp.header.stamp = pose_stamp.header.stamp;
    twist_stamp.header.frame_id = pose_stamp.header.frame_id;
    tf::vectorEigenToMsg(state.v, twist_stamp.twist.linear);
    tf::vectorEigenToMsg(state.w, twist_stamp.twist.angular);
    twist_pub.publish(twist_stamp);

    geometry_msgs::Vector3Stamped acc_stamp;
    acc_stamp.header.stamp = pose_stamp.header.stamp;
    acc_stamp.header.frame_id = pose_stamp.header.frame_id;
    tf::vectorEigenToMsg(state.a, acc_stamp.vector);
    acc_pub.publish(acc_stamp);
  }

  pub.publish(msg);

  ros::waitForShutdown();
  return 0;
}




