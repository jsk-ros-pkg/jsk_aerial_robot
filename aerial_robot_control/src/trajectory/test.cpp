#include <ros/ros.h>
#include <aerial_robot_msgs/States.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <aerial_robot_control/trajectory/trajectory_reference/polynomial_trajectory.hpp>
#include <visualization_msgs/MarkerArray.h>

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

  // mediate points
  agi::QuadState intermediate1_state;
  intermediate1_state.setZero();
  intermediate1_state.p = agi::Vector<3>(0.5,-1,1);
  intermediate1_state.t = du/2;

  agi::QuadState intermediate2_state;
  intermediate2_state.setZero();
  intermediate2_state.p = agi::Vector<3>(1.0,0,2);
  intermediate2_state.t = du/4 * 3;

  std::vector<agi::QuadState> states;
  states.push_back(start_state);
  states.push_back(intermediate1_state);
  states.push_back(intermediate2_state);
  states.push_back(end_state);

  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<nav_msgs::Path>("trajectory", 1);
  ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("pose", 1);
  ros::Publisher twist_pub = nh.advertise<geometry_msgs::TwistStamped>("twist", 1);
  ros::Publisher acc_pub = nh.advertise<geometry_msgs::Vector3Stamped>("acc", 1);
  ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("marker", 1);
  ros::Duration(0.5).sleep();

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

    // ROS_INFO_STREAM("position: " << state.p.transpose());

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

  visualization_msgs::MarkerArray marker_array_msg;

  visualization_msgs::Marker marker_msg;
  marker_msg.header.stamp = msg.header.stamp;
  marker_msg.header.frame_id = msg.header.frame_id;
  marker_msg.id = 0;
  marker_msg.action = visualization_msgs::Marker::ADD;
  marker_msg.type = visualization_msgs::Marker::SPHERE;
  marker_msg.pose.position.x = start_state.p(0);
  marker_msg.pose.position.y = start_state.p(1);
  marker_msg.pose.position.z = start_state.p(2);
  marker_msg.pose.orientation.w = 1;
  double r = 0.2;
  marker_msg.scale.x = r;
  marker_msg.scale.y = r;
  marker_msg.scale.z = r;
  marker_msg.color.r = 1.0;
  marker_msg.color.a = 1.0;
  marker_array_msg.markers.push_back(marker_msg);

  marker_msg.id = 1;
  marker_msg.pose.position.x = end_state.p(0);
  marker_msg.pose.position.y = end_state.p(1);
  marker_msg.pose.position.z = end_state.p(2);
  marker_array_msg.markers.push_back(marker_msg);

  marker_msg.id = 2;
  marker_msg.pose.position.x = intermediate1_state.p(0);
  marker_msg.pose.position.y = intermediate1_state.p(1);
  marker_msg.pose.position.z = intermediate1_state.p(2);
  marker_array_msg.markers.push_back(marker_msg);

  marker_msg.id = 3;
  marker_msg.pose.position.x = intermediate2_state.p(0);
  marker_msg.pose.position.y = intermediate2_state.p(1);
  marker_msg.pose.position.z = intermediate2_state.p(2);
  marker_array_msg.markers.push_back(marker_msg);

  marker_pub.publish(marker_array_msg);

  ros::waitForShutdown();
  return 0;
}




