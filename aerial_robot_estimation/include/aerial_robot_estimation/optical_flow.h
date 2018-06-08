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


#ifndef AERIAL_ROBOT_ESTIMATION_OPTICAL_FLOW_H_
#define AERIAL_ROBOT_ESTIMATION_OPTICAL_FLOW_H_

/* ros */
#include <ros/ros.h>
#include <nodelet/nodelet.h>

/* ros msg/srv */
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Range.h>

/* tf */
#include <tf/transform_listener.h>

/* opencv */
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/video/video.hpp"
#include "opencv2/calib3d.hpp"
#if USE_GPU
#include "opencv2/gpu/gpu.hpp"
#endif
#include <cv_bridge/cv_bridge.h>

/* ransac */
#include <mrpt/math/ransac.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/stock_objects.h>

/* for eigen cumputation */
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <tf_conversions/tf_eigen.h>

/* message filter */
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

/* thread */
#include <thread>

using namespace mrpt;
using namespace mrpt::utils;
using namespace mrpt::math;
using namespace mrpt::poses;


namespace aerial_robot_estimation
{
  class OpticalFlow : public nodelet::Nodelet
  {
  public:
    OpticalFlow():
      z_pos_(1.0), z_vel_(0),
      camera_info_update_(false),
      odom_update_(false),
      prev_image_ready_(false)
    {}

    ~OpticalFlow(){}
    virtual void onInit();

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Imu> SyncPolicy;

  private:
    /* ros */
    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;

    /* subscriber */
    //ros::Subscriber downward_camera_image_sub_;
    ros::Subscriber downward_camera_info_sub_;
    //ros::Subscriber imu_sub_;
    ros::Subscriber odometry_sub_;
    ros::Subscriber sonar_sub_;

    boost::shared_ptr< message_filters::Synchronizer<SyncPolicy> > sync_;
    message_filters::Subscriber<sensor_msgs::Image> sub_image_;
    message_filters::Subscriber<sensor_msgs::Imu> sub_imu_;

    /* publisher */
    ros::Publisher camera_vel_pub_;
    ros::Publisher baselink_vel_pub_;
    ros::Publisher optical_flow_image_pub_;

    /* topic name */
    std::string downward_camera_image_topic_name_;
    std::string downward_camera_info_topic_name_;
    std::string odometry_topic_name_;
    std::string imu_topic_name_;
    std::string camera_vel_topic_name_;
    std::string optical_flow_image_topic_name_;

    /* callback function */
    void downwardCameraImageCallback(const sensor_msgs::ImageConstPtr& msg);
    void downwardCameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg);
    void odometryCallback(const nav_msgs::OdometryConstPtr& msg);
    void imuCallback(const sensor_msgs::ImuConstPtr& msg);
    void opticalFlowCallback(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::ImuConstPtr& imu_msg);

    void motionEstimation();

    void ransac(const CMatrixDouble& all_data, double& t_x, double& t_y, double& t_z, vector_size_t& best_inliers);

    /* member variable */
    bool debug_;
    bool camera_info_update_;
    bool odom_update_;
    bool prev_image_ready_;

    ros::Time prev_image_stamp_, curr_image_stamp_;
    std::vector<cv::Point2f> prev_raw_feature_points_;
    cv::Mat prev_gray_img_, curr_gray_img_;

    double camera_f_, camera_cx_, camera_cy_;
    cv::Mat camera_d_, camera_mat_;

    tf::StampedTransform tf_fc2camera_;
    tf::Matrix3x3 uav_rotation_mat_;
    double z_pos_, z_vel_, alt_offset_;
    tf::Vector3 ang_vel_;

    int max_count_;

    /* param for feature extration and matching */
    double image_crop_scale_;
    int feature_min_dist_;
    int optical_flow_win_size_;
    double epipolar_thresh_;
    double inlier_rate_;

    double delta_rot_thresh_;
#if USE_GPU
    cv::gpu::GpuMat d_frame0Gray, d_frame1Gray;
#endif

    /* ransac */
    math::RANSAC ransac_;
  };

} //namespace aerial_robot_estimation


#endif
