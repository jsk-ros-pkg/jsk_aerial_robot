#ifndef PERCHING_H
#define PERCHING_H

#include <ros/ros.h>
#include <math.h>
#include <random>

#include <std_msgs/Empty.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <jsk_recognition_msgs/RectArray.h>
#include <aerial_robot_msgs/FlightNav.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <tf/transform_datatypes.h>

class ApproachingHuman
{
public:
  ApproachingHuman();
  void spin();  // use if you prefer spin loop instead of timer

private:
  // ===== Callbacks =====
  void cameraInfoCb(const sensor_msgs::CameraInfo::ConstPtr& msg);
  void rectCb(const jsk_recognition_msgs::RectArray::ConstPtr& msg);
  void flightStateCb(const std_msgs::UInt8::ConstPtr& msg);
  void depthCb(const sensor_msgs::Image::ConstPtr& msg);
  void odomCb(const nav_msgs::Odometry::ConstPtr& msg);
  void timerCb(const ros::TimerEvent& ev);

  // ===== Sub-routines =====
  void flightRotateState();
  void findMaxRect();
  void posCalc();
  void rotateYaw();
  void relativePos();
  void pdControl();

private:
  // ===== ROS =====
  ros::NodeHandle nh_;
  ros::Subscriber sub_flight_state_;
  ros::Subscriber sub_camera_info_;
  ros::Subscriber sub_rect_;
  ros::Subscriber sub_depth_;
  ros::Subscriber sub_odom_;

  ros::Publisher pub_target_pos_;
  ros::Publisher pub_reach_human_;
  ros::Publisher pub_move_;
  ros::Publisher pub_land_;

  ros::Timer timer_;

  // ===== Messages / states =====
  std_msgs::UInt8 flight_state_msg_;
  bool flight_state_flag_{false};
  bool approaching_flag_{false};

  int n_{0};
  int rotate_cnt_{0};
  int land_cnt_{0};
  bool rotate_flag_{true};
  double offset_yaw_{2.0 * M_PI};

  sensor_msgs::CameraInfo camera_info_;
  int camera_height_{720};
  int camera_width_{1280};
  double camera_param_{0.3};

  jsk_recognition_msgs::RectArray rects_;
  size_t max_index_{0};
  jsk_recognition_msgs::Rect max_rect_;
  geometry_msgs::Vector3 max_rect_pos_;        // center (px) relative to image center
  geometry_msgs::Vector3 max_rect_pixel_pos_;  // center pixel (px)
  double max_rect_area_{0.0};
  double max_thresh_area_{370000.0};

  nav_msgs::Odometry::ConstPtr odom_raw_;
  geometry_msgs::Vector3 euler_;
  double height_{0.0};

  double Kp_{0.3};
  double Kd_{0.01};

  cv::Mat depth_img_; // 16UC1 (mm) assumed
  bool depth_ready_{false};

  double depth_{0.0};
  double prev_depth_{0.0};
  bool depth_check_start_{false};
  double depth_thresh_max_{10000.0};
  double depth_thresh_min_{0.0};
  double min_depth_{10.0};
  double raw_depth_{0.0};
  int fail_safe_stop_cnt_{0};

  bool reach_to_human_flag_{false};

  aerial_robot_msgs::FlightNav move_msg_;
  geometry_msgs::Vector3 target_pos_;

  // RNG for neighborhood sampling
  std::mt19937 rng_;
};

int main_approaching_human(int argc, char** argv);

#endif
