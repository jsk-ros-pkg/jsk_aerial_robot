#ifndef TREE_TRACKING_H_
#define TREE_TRACKING_H_

/* ros */
#include <ros/ros.h>

/* ros msg/srv */
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Empty.h>

/* tree database */
#include "tree_database.h" //
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

using namespace std;

class TreeTracking
{
public:
  TreeTracking(ros::NodeHandle nh, ros::NodeHandle nhp);
  ~TreeTracking(){}

private:
  ros::NodeHandle nh_, nhp_;

  ros::Subscriber sub_laser_scan_;
  ros::Subscriber sub_odom_, sub_erase_tree_;
  ros::Publisher pub_visualization_marker_;
  string laser_scan_topic_name_, odom_topic_name_;
  string tree_global_location_topic_name_;
  string visualization_marker_topic_name_, tree_database_erase_topic_name_;
  string laser_scan_frame_name_;

  TreeDataBase tree_db_;
  vector<TreeHandlePtr> target_trees_;

  tf::Vector3 uav_odom_;
  float uav_roll_, uav_pitch_, uav_yaw_;
  tf::TransformListener listener;
  tf::StampedTransform transform;

  double tree_scan_angle_thre_;
  double tree_circle_regulation_thre_;
  double tree_radius_max_, tree_radius_min_;

  double urg_yaw_offset_;
  // void visionDetectionCallback(const geometry_msgs::Vector3StampedConstPtr& vision_detection_msg);
  void uavOdomCallback(const nav_msgs::OdometryConstPtr& uav_msg);
  void laserScanCallback(const sensor_msgs::LaserScanConstPtr& scan_msg);
  void eraseTreeDBCallback(const std_msgs::Empty& msg);
  
};

#endif
