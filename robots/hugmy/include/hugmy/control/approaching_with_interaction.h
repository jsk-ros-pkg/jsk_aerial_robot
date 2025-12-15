#ifndef PERCHING_H
#define PERCHING_H

#include <ros/ros.h>
#include <math.h>
#include <random>
#include <algorithm>
#include <limits>
#include <cmath>
#include <string>
#include <vector>

#include <std_msgs/Empty.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <jsk_recognition_msgs/RectArray.h>
#include <jsk_recognition_msgs/HumanSkeletonArray.h>
#include <jsk_recognition_msgs/HumanSkeleton.h>
#include <jsk_recognition_msgs/Segment.h>

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
  void humanSkeletonCb(const jsk_recognition_msgs::HumanSkeletonArray::ConstPtr& msg);
  void timerCb(const ros::TimerEvent& ev);
  

  // ===== Sub-routines =====
  void flightRotateState();
  void findMaxRect();
  void findBones();
  void posCalc();
  void rotateYaw();
  void relativePos();
  void pdControl();
  void updateGazeAndExpressionWhileApproaching();
  void handleValidDepth();
  void handleInvalidDepth();
  void handleRectDetected();
  void handleReachedHuman();
  void stopWithIdleExpression();
  void handleNoRectDetected();
  void updateLandCounter();
  

private:
  // ===== ROS =====
  ros::NodeHandle nh_;
  ros::Subscriber sub_flight_state_;
  ros::Subscriber sub_camera_info_;
  ros::Subscriber sub_rect_;
  ros::Subscriber sub_depth_;
  ros::Subscriber sub_odom_;
  ros::Subscriber sub_human_skeleton_;

  ros::Publisher pub_target_pos_;
  ros::Publisher pub_reach_human_;
  ros::Publisher pub_move_;
  ros::Publisher pub_land_;

  //face expression
  ros::Publisher pub_vad_;
  ros::Publisher pub_gaze_;


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

  //face expression
  std_msgs::Float32MultiArray vad_array_;
  std_msgs::Float32 gaze_x_;
  int dont_see_cnt_ = 0;
  bool face_first_change_flag_{true};

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


   std::vector<std::vector<std::string>> bone_names_;
  std::vector<std::vector<jsk_recognition_msgs::Segment>> bones_;
  std::vector<std::string> target_shoulder_bones_ = {"left_shoulder", "right_shoulder"};
  std::vector<std::string> target_wrist_bones_ = {"left_wrist", "right_wrist"};

  geometry_msgs::Point left_wrist_bone_;
  geometry_msgs::Point right_wrist_bone_;
  geometry_msgs::Point shoulder_bone_;
  bool handup_flag_;
  bool left_wrist_find_ = false;
  bool right_wrist_find_ = false;
  bool both_handup_flag_ = false;
  bool both_wrist_find_ = false;
  bool shoulder_find_ = false;
  double shoulder_dist_ = 0.0;
  int shoulder_u_ = -1;
  int shoulder_v_ = -1;

  ros::Time wrist_detect_time_;
  ros::Time shoulder_detect_time_;


  double depth_trend_prev_ = std::numeric_limits<double>::quiet_NaN();

  std::mt19937 rng_;
};

int main_approaching_human(int argc, char** argv);

#endif
