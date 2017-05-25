// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/


#ifndef OPTICAL_FLOW_H_
#define OPTICAL_FLOW_H_

/* ros */
#include <ros/ros.h>

/* ros msg/srv */
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Range.h>

/* tf */
#include <tf/transform_broadcaster.h>

/* opencv */
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/video.hpp"
#include "opencv2/gpu/gpu.hpp"
#include <cv_bridge/cv_bridge.h>

class OpticalFlow
{
public:
  OpticalFlow(ros::NodeHandle nh, ros::NodeHandle nhp);
  ~OpticalFlow(){}

private:
  /* ros */
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  
  /* subscriber */
  ros::Subscriber downward_camera_image_sub_;
  ros::Subscriber downward_camera_info_sub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber odometry_sub_;
  ros::Subscriber sonar_sub_;

  /* publisher */
  ros::Publisher camera_vel_pub_;
  ros::Publisher optical_flow_image_pub_;
  
  /* topic name */
  std::string downward_camera_image_topic_name_;
  std::string downward_camera_info_topic_name_;
  std::string imu_topic_name_;
  std::string odometry_topic_name_;
  std::string camera_vel_topic_name_;
  std::string optical_flow_image_topic_name_;
  std::string sonar_topic_name_;

  /* callback function */
  void downwardCameraImageCallback(const sensor_msgs::ImageConstPtr& msg);
  void downwardCameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);
  void imuCallback(const sensor_msgs::ImuConstPtr& msg);
  void odometryCallback(const nav_msgs::OdometryConstPtr& msg);
  void sonarCallback(const sensor_msgs::RangeConstPtr& msg);

  /* member variable */
  bool verbose_;
  bool camera_info_update_;
  bool imu_update_;
  bool image_stamp_update_;
  bool sonar_update_;
  bool use_sonar_;
  double camera_f_, camera_cx_, camera_cy_;
  double sonar_, sonar_vel_, sonar_offset_;
  tf::Vector3 ang_vel_;
  cv::Mat prev_gray_img_;
  std::vector<cv::Point2f> points_[2];
  int max_count_;
  ros::Time prev_stamp_;
  ros::Time sonar_prev_stamp_;
  tf::Matrix3x3 camera_rotation_mat_, camera_rotation_mat_inv_;
  double camera_roll_, camera_pitch_, camera_yaw_;
};

#endif
