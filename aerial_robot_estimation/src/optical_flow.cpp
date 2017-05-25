#include "aerial_robot_estimation/optical_flow.h"

namespace {
  void download(const cv::gpu::GpuMat& d_mat, std::vector<cv::Point2f>& vec)
  {
    vec.resize(d_mat.cols);
    cv::Mat mat(1, d_mat.cols, CV_32FC2, (void*)&vec[0]);
    d_mat.download(mat);
  }
  
  void download(const cv::gpu::GpuMat& d_mat, std::vector<uchar>& vec)
  {
    vec.resize(d_mat.cols);
    cv::Mat mat(1, d_mat.cols, CV_8UC1, (void*)&vec[0]);
    d_mat.download(mat);
  }
}

OpticalFlow::OpticalFlow(ros::NodeHandle nh, ros::NodeHandle nhp):
nh_(nh), nhp_(nhp), camera_info_update_(false), imu_update_(false), image_stamp_update_(false), sonar_(-1.0)
{
  /* ros param */
  //nhp_.param("downward_camera_image_topic_name", downward_camera_image_topic_name_, std::string("/downward_cam/camera/image"));
  nhp_.param("downward_camera_image_topic_name", downward_camera_image_topic_name_, std::string("/zed/left/image_rect_color"));
  
  //nhp_.param("downward_camera_info_topic_name", downward_camera_info_topic_name_, std::string("/downward_cam/camera/camera_info"));
  nhp_.param("downward_camera_info_topic_name", downward_camera_info_topic_name_, std::string("/zed/left/camera_info"));
 
  //nhp_.param("imu_topic_name", imu_topic_name_, std::string("/raw_imu"));
  nhp_.param("imu_topic_name", imu_topic_name_, std::string("/imu"));
  //nhp_.param("odometry_topic_name", odometry_topic_name_, std::string("/ground_truth/state"));
  nhp_.param("odometry_topic_name", odometry_topic_name_, std::string("/uav/state"));

  nhp_.param("camera_vel_topic_name", camera_vel_topic_name_, std::string("camera_velocity"));
  nhp_.param("optical_flow_image_topic_name", optical_flow_image_topic_name_, std::string("optical_flow_image"));
  nhp_.param("sonar_topic_name", sonar_topic_name_, std::string("sonar_height"));

  nhp_.param("verbose", verbose_, true);
  nhp_.param("max_count", max_count_, 100);
  nhp_.param("use_sonar", use_sonar_, false);
  nhp_.param("sonar_offset", sonar_offset_, 0.0);

  //imu(base)->camera
  nhp_.param("camera_roll_offset", camera_roll_, M_PI);
  nhp_.param("camera_pitch_offset", camera_pitch_, 0.0);
  nhp_.param("camera_yaw_offset", camera_yaw_, -M_PI / 2);
  nhp_.param("camera_x_offset", camera_x_, 0.0);
  nhp_.param("camera_y_offset", camera_y_, 0.0);
  nhp_.param("camera_z_offset", camera_z_, 0.0);

  /* subscriber */
  downward_camera_image_sub_ = nh_.subscribe(downward_camera_image_topic_name_, 1, &OpticalFlow::downwardCameraImageCallback, this);
  downward_camera_info_sub_ = nh_.subscribe(downward_camera_info_topic_name_, 1, &OpticalFlow::downwardCameraInfoCallback, this);
  imu_sub_ = nh_.subscribe(imu_topic_name_, 1, &OpticalFlow::imuCallback, this);
  if (use_sonar_)
    sonar_sub_ = nh_.subscribe(sonar_topic_name_, 1, &OpticalFlow::sonarCallback, this);
  else
    odometry_sub_ = nh_.subscribe(odometry_topic_name_, 1, &OpticalFlow::odometryCallback, this);  

  /* publisher */
  camera_vel_pub_ = nh_.advertise<geometry_msgs::Vector3Stamped>(camera_vel_topic_name_, 10);
  optical_flow_image_pub_ = nh_.advertise<sensor_msgs::Image>(optical_flow_image_topic_name_, 1);
  
  camera_rotation_mat_.setRPY(camera_roll_, camera_pitch_, camera_yaw_); //imu->camera
  camera_rotation_mat_inv_ = camera_rotation_mat_.inverse(); //camera->imu
}

void OpticalFlow::downwardCameraImageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  if (!camera_info_update_ || !imu_update_ || !image_stamp_update_) {
    prev_stamp_ = msg->header.stamp;
    image_stamp_update_ = true;
    return;
  }
  tf::Vector3 ang_vel = camera_rotation_mat_inv_ * ang_vel_;
  cv::Mat src_img = cv_bridge::toCvCopy(msg, msg->encoding)->image;
  cv::Mat gray_img;
  if (src_img.channels() > 1) {
    cv::cvtColor(src_img, gray_img, cv::COLOR_BGR2GRAY);
  } else {
    src_img.copyTo(gray_img);
  }

  if(prev_gray_img_.empty()) gray_img.copyTo(prev_gray_img_);
  std::vector<cv::Point2f> points[2];

  //calc features
#if USE_GPU
  cv::gpu::GoodFeaturesToTrackDetector_GPU detector(max_count_, 0.01, 10, 5, false, 0.04);
  cv::gpu::GpuMat d_frame0Gray(prev_gray_img_);
  cv::gpu::GpuMat d_prevPts;
  detector(d_frame0Gray, d_prevPts);
#else
  cv::TermCriteria termcrit(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 20, 0.03);
  cv::Size subPixWinSize(10,10), winSize(31,31);
  cv::goodFeaturesToTrack(gray_img, points[0], max_count_, 0.01, 10, cv::Mat(), 3, 0, 0.04);
  cv::cornerSubPix(gray_img, points[0], subPixWinSize, cv::Size(-1,-1), termcrit);
#endif

  double time = (msg->header.stamp - prev_stamp_).toSec();
  std::vector<uchar> status;

  //calc optical flow
#if USE_GPU
  cv::gpu::PyrLKOpticalFlow d_pyrLK;

  d_pyrLK.winSize.width = 31;
  d_pyrLK.winSize.height = 31;
  d_pyrLK.maxLevel = 3;
  d_pyrLK.iters = 30;

  cv::gpu::GpuMat d_frame1Gray(gray_img);
  cv::gpu::GpuMat d_nextPts, d_status;
  
  d_pyrLK.sparse(d_frame0Gray, d_frame1Gray, d_prevPts, d_nextPts, d_status);
 
  points[0].resize(d_prevPts.cols);
  download(d_prevPts, points[0]);
  
  points[1].resize(d_nextPts.cols);
  download(d_nextPts, points[1]);
  
  status.resize(d_status.cols);
  download(d_status, status);
#else
  std::vector<float> err;
  if (!points[0].empty()) 
    cv::calcOpticalFlowPyrLK(prev_gray_img_, gray_img, points[0], points[1], status, err, winSize, 3, termcrit, 0, 0.001);
#endif
  tf::Vector3 camera_vel;
  double camera_x_vel = 0.0, camera_y_vel = 0.0;
  size_t valid_point = 0;
  for(size_t i = 0; i < points[0].size(); i++) {
    if(!status[i]) continue;

    valid_point++;
    double x = points[1][i].x - camera_cx_, prev_x = points[0][i].x - camera_cx_;
    double y = points[1][i].y - camera_cy_, prev_y = points[0][i].y - camera_cy_;
    
    camera_x_vel += sonar_vel_ * x / camera_f_ + (-(x - prev_x) / time - ang_vel.y() * camera_f_ + ang_vel.z() * y + (ang_vel.x() * x * y - ang_vel.y() * x * x) / camera_f_) * sonar_ / camera_f_;
    camera_y_vel += sonar_vel_ * y / camera_f_ + (-(y - prev_y) / time + ang_vel.x() * camera_f_ - ang_vel.z() * x + (ang_vel.x() * y * y - ang_vel.y() * x * y) / camera_f_) * sonar_ / camera_f_;
    if (verbose_) {
      cv::circle(src_img, points[0][i], 3, cv::Scalar(0,255,0), -1, 8);
      cv::line(src_img, points[1][i], points[0][i], cv::Scalar(0,255,0), 1, 8, 0);
    }
  }
  
  if (valid_point != 0) camera_vel.setValue(camera_x_vel / valid_point, camera_y_vel / valid_point, sonar_vel_);

  camera_vel = camera_rotation_mat_ * camera_vel;

  if (verbose_) 
    optical_flow_image_pub_.publish(cv_bridge::CvImage(msg->header, msg->encoding, src_img).toImageMsg());
  cv::swap(prev_gray_img_, gray_img);
  
  geometry_msgs::Vector3Stamped camera_vel_msg;
  camera_vel_msg.header = msg->header;
  camera_vel_msg.vector.x = camera_vel.x();
  camera_vel_msg.vector.y = camera_vel.y();
  camera_vel_msg.vector.z = camera_vel.z();
  camera_vel_pub_.publish(camera_vel_msg);
  
  prev_stamp_ = msg->header.stamp;
}

    
void OpticalFlow::downwardCameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
{
  if (camera_info_update_) return;
  camera_f_ = msg->K[0];
  camera_cx_ = msg->K[2];
  camera_cy_ = msg->K[5];
  if (msg->K[0] > 0) {
    camera_info_update_ = true;
  }
}

void OpticalFlow::imuCallback(const sensor_msgs::ImuConstPtr& msg)
{
  tf::vector3MsgToTF(msg->angular_velocity, ang_vel_);
  imu_update_ = true;
}

void OpticalFlow::odometryCallback(const nav_msgs::OdometryConstPtr& msg)
{
  tf::Quaternion uav_q(msg->pose.pose.orientation.x,
		       msg->pose.pose.orientation.y,
		       msg->pose.pose.orientation.z,
		       msg->pose.pose.orientation.w);
  tf::Matrix3x3 uav_rotation_mat_(uav_q);
  double r, p, y;
  uav_rotation_mat_.getRPY(r, p, y);
  sensor_msgs::Range sonar_msg;
  sonar_msg.header = msg->header;
  sonar_msg.range = msg->pose.pose.position.z;
  sensor_msgs::RangeConstPtr msg_ptr(&sonar_msg);
  sonarCallback(msg_ptr);
}

void OpticalFlow::sonarCallback(const sensor_msgs::RangeConstPtr& msg)
{
  double sonar_val = msg->range - sonar_offset_;
  if (sonar_ != -1.0) {
    sonar_vel_ = (sonar_val - prev_sonar_) / (msg->header.stamp - sonar_prev_stamp_).toSec();
    sonar_update_ = true;  
  }
  sonar_ = sonar_val;
  sonar_prev_stamp_ = msg->header.stamp;
  prev_sonar_ = msg->range;
}
