#include <hugmy/control/perching.h>

ApproachingHuman::ApproachingHuman()
: nh_(), rng_(0)
{
  sub_flight_state_ = nh_.subscribe<std_msgs::UInt8>("/quadrotor/flight_state", 1, &ApproachingHuman::flightStateCb, this);
  sub_camera_info_ = nh_.subscribe<sensor_msgs::CameraInfo>("/camera/color/camera_info", 1, &ApproachingHuman::cameraInfoCb, this);
  sub_rect_ = nh_.subscribe<jsk_recognition_msgs::RectArray>("/human", 1, &ApproachingHuman::rectCb, this);
  sub_depth_ = nh_.subscribe<sensor_msgs::Image>("/camera/depth/image_rect_raw", 1, &ApproachingHuman::depthCb, this);
  sub_odom_ = nh_.subscribe<nav_msgs::Odometry>("/quadrotor/uav/cog/odom", 1, &ApproachingHuman::odomCb, this);

  pub_target_pos_ = nh_.advertise<geometry_msgs::Vector3>("/3D_pos_1", 1);
  pub_reach_human_ = nh_.advertise<std_msgs::Bool>("/reach_flag", 1);
  pub_move_ = nh_.advertise<aerial_robot_msgs::FlightNav>("/quadrotor/uav/nav", 1);
  pub_land_ = nh_.advertise<std_msgs::Empty>("/quadrotor/teleop_command/land", 1);

  move_msg_.control_frame = 1;
  move_msg_.target = 1;
  move_msg_.pos_xy_nav_mode = 1;
  move_msg_.yaw_nav_mode = 2;

  //face exression
  pub_vad_ = nh_.advertise<std_msgs::Float32MultiArray>("/vad", 1);
  pub_gaze_ = nh_.advertise<std_msgs::Float32>("/look_at", 1);

  timer_ = nh_.createTimer(ros::Duration(0.05), &ApproachingHuman::timerCb, this); // 20Hz

  ROS_INFO("ApproachingHuman node constructed");
}

// ===== Callbacks =====
void ApproachingHuman::cameraInfoCb(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
  camera_info_ = *msg;
  camera_height_ = camera_info_.height;
  camera_width_  = camera_info_.width;
}

void ApproachingHuman::rectCb(const jsk_recognition_msgs::RectArray::ConstPtr& msg)
{
  rects_ = *msg;
  n_ = static_cast<int>(rects_.rects.size());
}

void ApproachingHuman::flightStateCb(const std_msgs::UInt8::ConstPtr& msg)
{
  flight_state_msg_ = *msg;
  // demo/test: force true (as in python "####test")
  //test
  flight_state_flag_ = true;
  //demo
//   if (flight_state_msg_.data == 5) {
//     flight_state_flag_ = true;
//   } else {
//     flight_state_flag_ = false;
//   }
  if (rotate_cnt_ >= 20) {
    rotate_flag_ = true;
    rotate_cnt_ = 0;
  }
}

void ApproachingHuman::depthCb(const sensor_msgs::Image::ConstPtr& msg)
{
  try {
    cv_bridge::CvImageConstPtr cvp = cv_bridge::toCvShare(msg);
    if (cvp->image.type() == CV_16UC1) {
      depth_img_ = cvp->image.clone();
      depth_ready_ = true;
    } else if (cvp->image.type() == CV_32FC1) {
      // Convert meters->mm to mimic python logic that expects millimeters
      cv::Mat mm;
      cvp->image.convertTo(mm, CV_32FC1, 1000.0); // meters to mm
      mm.convertTo(depth_img_, CV_16UC1);  // clamp/convert
      depth_ready_ = true;
    } else {
      ROS_WARN_THROTTLE(1.0, "Unexpected depth encoding: %d", cvp->image.type());
      depth_ready_ = false;
    }
  } catch (const std::exception& e) {
    ROS_ERROR_THROTTLE(1.0, "depthCb cv_bridge error: %s", e.what());
    depth_ready_ = false;
  }
}

void ApproachingHuman::odomCb(const nav_msgs::Odometry::ConstPtr& msg)
{
  odom_raw_ = msg;
  const auto& p = msg->pose.pose;
  height_ = p.position.z;

  tf::Quaternion q(p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w);
  double roll, pitch, yaw;
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
  euler_.x = roll;
  euler_.y = pitch;
  euler_.z = yaw;
}

void ApproachingHuman::flightRotateState()
{
  //flight_state_flag sets True in test mode
  flight_state_flag_ = true;

  if (rotate_cnt_ >= 20) {
    rotate_flag_ = true;
    rotate_cnt_ = 0;
  }
}

void ApproachingHuman::findMaxRect()
{
  double max_area = 0.0;
  size_t idx = 0;
  for (size_t i = 0; i < rects_.rects.size(); ++i) {
    const auto& r = rects_.rects[i];
    double area = static_cast<double>(r.width) * static_cast<double>(r.height);
    if (area >= max_area) {
      max_area = area;
      idx = i;
    }
  }
  max_index_ = idx;
  if (!rects_.rects.empty()) {
    max_rect_ = rects_.rects[max_index_];
  }
  max_rect_area_ = max_area;
}

void ApproachingHuman::posCalc()
{
  // center relative to image center (pixels)
  max_rect_pos_.x = static_cast<double>(max_rect_.x) + (static_cast<double>(max_rect_.width) / 2.0) - (static_cast<double>(camera_width_) / 2.0);
  max_rect_pos_.y = - (static_cast<double>(max_rect_.y) + (static_cast<double>(max_rect_.height) / 2.0)) + (static_cast<double>(camera_height_) / 2.0);
  ROS_INFO("max_rect_pos.x: %.3f", max_rect_pos_.x);
}

void ApproachingHuman::rotateYaw()
{
  // yaw command based on horizontal pixel offset
  double yaw = max_rect_pos_.x * (M_PI / static_cast<double>(camera_width_)) * camera_param_;
  move_msg_.target_yaw = -yaw + euler_.z;  // keep relative to current yaw
  target_pos_.z = move_msg_.target_yaw;

  ROS_INFO("rotate_yaw: %.3f", move_msg_.target_yaw);
  pub_move_.publish(move_msg_);
  rotate_flag_ = false;
}

void ApproachingHuman::relativePos()
{
  if (!depth_ready_ || depth_img_.empty()) {
    ROS_WARN_THROTTLE(1.0, "Depth not ready");
    return;
  }
  // center pixel of rect
  int cx = static_cast<int>(max_rect_.x + max_rect_.width / 2);
  int cy = static_cast<int>(max_rect_.y + max_rect_.height / 2);
  cx = std::max(0, std::min(cx, depth_img_.cols - 1));
  cy = std::max(0, std::min(cy, depth_img_.rows - 1));

  max_rect_pixel_pos_.x = cx;
  max_rect_pixel_pos_.y = cy;

  // depth sampling around the center (±10 px), accept within ±200mm of center
  int pixel_cnt = 1;
  uint16_t depth_center = depth_img_.at<uint16_t>(cy, cx);
  int depth_sum = depth_center;

  std::uniform_int_distribution<int> dist(-10, 10);
  for (int i = 0; i < 10; ++i) {
    int px = std::max(0, std::min(cx + dist(rng_), depth_img_.cols - 1));
    int py = std::max(0, std::min(cy + dist(rng_), depth_img_.rows - 1));
    uint16_t d = depth_img_.at<uint16_t>(py, px);
    if (std::abs(static_cast<int>(d) - static_cast<int>(depth_center)) <= 200) {
      depth_sum += d;
      pixel_cnt++;
    }
  }

  raw_depth_ = static_cast<double>(depth_sum) / static_cast<double>(pixel_cnt); // [mm]
  ROS_INFO("Raw depth: %.1f", raw_depth_);
  target_pos_.y = raw_depth_;

  std_msgs::Empty empty_msg;

  if (raw_depth_ > 0.0) {
    fail_safe_stop_cnt_ = 0;
    depth_check_start_ = true;
  } else {
    fail_safe_stop_cnt_++;
    if (fail_safe_stop_cnt_ > 20) {
      ROS_ERROR("fail safe land!");
      pub_land_.publish(empty_msg);
    }
  }

  if (depth_check_start_) {
    if (raw_depth_ > depth_thresh_min_ && raw_depth_ <= depth_thresh_max_) {
      depth_ = raw_depth_ / 1000.0 - 0.5; // meters, minus 0.5 m offset
      prev_depth_ = depth_;
      depth_thresh_max_ = raw_depth_ + 500.0;
      depth_thresh_min_ = std::max(raw_depth_ - 500.0, -100.0);

      if (min_depth_ >= depth_) {
        if (std::abs(prev_depth_ - depth_) < 0.15) {
          min_depth_ = depth_;
        }
      }
    } else {
      ROS_WARN("Invalid depth value: %.1f. Using previous depth: %.3f", raw_depth_, prev_depth_);
      depth_ = prev_depth_;
    }
  }

  ROS_INFO("depth: %.3f", depth_);
}

void ApproachingHuman::pdControl()
{
  double output = depth_ * Kp_ + (depth_ - prev_depth_) * Kd_;
  target_pos_.x = output;
  move_msg_.target_vel_x = output;
  ROS_INFO("vel: %.3f", output);
}

void ApproachingHuman::timerCb(const ros::TimerEvent&)
{
  flightRotateState();
  std_msgs::Bool msg;
  msg.data = reach_to_human_flag_;
  pub_reach_human_.publish(msg);

  if (reach_to_human_flag_ == true) {
    ROS_INFO("shutdown");
    timer_.stop();
    return;
  }

  if (flight_state_flag_) {
    if (n_ >= 1 && !rects_.rects.empty()) {
      findMaxRect();
      if (rotate_flag_) {
        rotateYaw();
        ROS_INFO("rotate!");
      }
      relativePos();
      posCalc();
      ROS_INFO("%d", land_cnt_);
      pub_target_pos_.publish(target_pos_);
    //   pdControl();
    //   pub_move_.publish(move_msg_);
    //   ROS_INFO("go!");

      if (depth_ > 0.0) {
        // valid depth
        pdControl();
        pub_move_.publish(move_msg_);
        ROS_INFO("go!");
        land_cnt_ = 0;
        fail_safe_stop_cnt_ = 0;
        /////////////// face expression change the iris movement
        float w = static_cast<float>(camera_width_);
        float x_px = static_cast<float>(max_rect_pixel_pos_.x)  - w * 0.5 ;
        float x_norm = - (x_px / w )*2.0f;
        gaze_x_.data = std::clamp(x_norm, -1.0f, 1.0f);
        pub_gaze_.publish(gaze_x_);
        if (depth_ <= 0.7f){
          /////////////// face expression change v
          float v = (0.7f- static_cast<float>(depth_)) / 0.7f;
          float a = (0.7f- static_cast<float>(depth_)) / 0.7f;
          v = std::clamp(v, a, 1.0f);
          vad_array_.data = {v, 0.0f, 0.0f};
          pub_vad_.publish(vad_array_);
          if (v >= 0.5f){
            gaze_x_.data = 0.0;
            pub_gaze_.publish(gaze_x_);
          }
          face_first_change_flag_ == true;
        }else{
          if (face_first_change_flag_){
            vad_array_.data = {0.0f, 0.0f, 0.0f};
            pub_vad_.publish(vad_array_);
          }
          face_first_change_flag_ == false;
        }
      } else {
        // invalid/zero depth
        if (raw_depth_ == 0.0) {
          fail_safe_stop_cnt_++;
          if (fail_safe_stop_cnt_ > 10) {
            ROS_INFO("fail safe land!");
            std_msgs::Empty e;
            pub_land_.publish(e);
          }
        }
        move_msg_.target_vel_x = 0.0;
        pub_move_.publish(move_msg_);
        ROS_INFO("stop!");
        land_cnt_++;
      }

      if (land_cnt_ >= 10) {
        for (int i = 0; i < 10; ++i) {
          reach_to_human_flag_ = true;
          /////////////// face expression max v(smile)
          vad_array_.data = {1.0f, 0.8f, 0.0f};
          pub_vad_.publish(vad_array_);
          std_msgs::Bool msg2;
          msg2.data = reach_to_human_flag_;
          pub_reach_human_.publish(msg2);
        }
        land_cnt_ = 0;
      }
      rotate_cnt_++;
    } else {
      move_msg_.target_vel_x = 0.0;
      pub_move_.publish(move_msg_);
      ROS_INFO("stop! because the robot can't see people");
      /////////////// face expression disgust
      vad_array_.data = {-0.5f, -0.3f, -0.8f};
      pub_vad_.publish(vad_array_);
      face_first_change_flag_ == true;
      if (dont_see_cnt_ % 2 ==0){
        gaze_x_.data = -0.8;
      }else{
        gaze_x_.data = 0.8;
      }
      pub_gaze_.publish(gaze_x_);
      dont_see_cnt_++;
      if (min_depth_ > 2.0) {
        move_msg_.target_yaw = euler_.z + 0.01;
        pub_move_.publish(move_msg_);
        ROS_INFO("yaw rotate to see people: %.3f", move_msg_.target_yaw);
        ros::Duration(1.0).sleep();
      }
    }
  }
}

void ApproachingHuman::spin()
{
  ros::spin();
}

// ===== main wrapper (optional) =====
int main_approaching_human(int argc, char** argv)
{
  ros::init(argc, argv, "Approaching_human");
  ApproachingHuman node;
  ros::spin();
  return 0;
}

// If you prefer a standalone main in this file:
int main(int argc, char** argv) { return main_approaching_human(argc, argv); }
