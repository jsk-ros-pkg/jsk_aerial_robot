#include <hugmy/control/approaching_with_interaction.h>

ApproachingHuman::ApproachingHuman()
: nh_(), rng_(0)
{
  sub_flight_state_ = nh_.subscribe<std_msgs::UInt8>("/quadrotor/flight_state", 1, &ApproachingHuman::flightStateCb, this);
  sub_camera_info_ = nh_.subscribe<sensor_msgs::CameraInfo>("/camera/color/camera_info", 1, &ApproachingHuman::cameraInfoCb, this);
  sub_rect_ = nh_.subscribe<jsk_recognition_msgs::RectArray>("/human", 1, &ApproachingHuman::rectCb, this);
  sub_depth_ = nh_.subscribe<sensor_msgs::Image>("/camera/depth/image_rect_raw", 1, &ApproachingHuman::depthCb, this);
  sub_odom_ = nh_.subscribe<nav_msgs::Odometry>("/quadrotor/uav/cog/odom", 1, &ApproachingHuman::odomCb, this);
  sub_human_skeleton_ = nh_.subscribe<jsk_recognition_msgs::HumanSkeletonArray>("/edgetpu_human_pose_estimator/output/skeletons", 1, &ApproachingHuman::humanSkeletonCb, this);

  pub_target_pos_ = nh_.advertise<geometry_msgs::Vector3>("/3D_pos_1", 1);
  pub_reach_human_ = nh_.advertise<std_msgs::Bool>("/reach_flag", 1);
  pub_move_ = nh_.advertise<aerial_robot_msgs::FlightNav>("/quadrotor/uav/nav", 1);
  pub_land_ = nh_.advertise<std_msgs::Empty>("/quadrotor/teleop_command/land", 1);

  move_msg_.control_frame = 1;
  move_msg_.target = 1;
  move_msg_.pos_xy_nav_mode = 1;
  move_msg_.yaw_nav_mode = 2;

  //face expression
  pub_vad_ = nh_.advertise<std_msgs::Float32MultiArray>("/vad", 1);
  pub_gaze_ = nh_.advertise<std_msgs::Float32>("/look_at", 1);

  timer_ = nh_.createTimer(ros::Duration(0.05), &ApproachingHuman::timerCb, this); // 20Hz

  ROS_INFO("ApproachingHuman node constructed");
}

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

void ApproachingHuman::humanSkeletonCb(
    const jsk_recognition_msgs::HumanSkeletonArrayConstPtr& msg)
{
  bone_names_.clear();
  bones_.clear();

  // for (size_t i = 0; i < msg->skeletons.size(); ++i) {
  //   const auto& skel = msg->skeletons[i];
  //   bone_names_.push_back(skel.bone_names);
  //   bones_.push_back(skel.bones);
  // }
  if (msg->skeletons.empty()) {
    return;
  }

  size_t idx = max_index_;
  if (idx >= msg->skeletons.size()) {
    idx = msg->skeletons.size() - 1;
  }
  
  const auto& skel = msg->skeletons[idx];
  bone_names_.push_back(skel.bone_names);
  bones_.push_back(skel.bones);
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

// void ApproachingHuman::findBones(){
//   if (bone_names_.empty() || bones_.empty()) {
//     shoulder_find_ = false;
//     wrist_find_    = false;
//     return;
//   }

//   auto trim = [](const std::string& s) -> std::string {
//                 const auto first = s.find_first_not_of(" \t");
//                 if (first == std::string::npos) return std::string();
//                 const auto last = s.find_last_not_of(" \t");
//                 return s.substr(first, last - first + 1);
//               };

//   ros::Time now = ros::Time::now();

//   for (size_t i = 0 ; i < bone_names_.size(); ++i){
//     const auto& names = bone_names_[i];
//     const auto& bones = bones_[i];

//     if (names.size() != bones.size()) {
//       ROS_WARN("bone_names_[%zu].size() != bones_[%zu].size()", i, i);
//       continue;
//     }

//     for (size_t k = 0; k < names.size(); ++k) {
//       const std::string& bone_name = names[k];

//       size_t pos = bone_name.find("->");
//       if (pos == std::string::npos) {
//         continue;
//       }

//       std::string start_bone_name = trim(bone_name.substr(0, pos));
//       std::string end_bone_name   = trim(bone_name.substr(pos + 2));

//       ROS_INFO("%zu, bone %zu: start: %s, end: %s", i, k, start_bone_name.c_str(), end_bone_name.c_str());

//       const jsk_recognition_msgs::Segment& bone = bones[k];

//       for (size_t j = 0 ; j < target_shoulder_bones_.size(); ++j){
//         const std::string target = target_shoulder_bones_[j];
//         if (start_bone_name == target){
//           shoulder_bone_ = bone.start_point;
//           shoulder_find_ = true;
//           shoulder_detect_time_ = now;
//         }else if (end_bone_name == target){
//           shoulder_bone_ = bone.end_point;
//           shoulder_find_ = true;
//           shoulder_detect_time_ = now; 
//         }
//       }

//       for (size_t j = 0 ; j < target_wrist_bones_.size(); ++j){
//         const std::string target = target_wrist_bones_[j];
//         if (start_bone_name == target){
//           wrist_bone_ = bone.start_point;
//           wrist_find_ = true;
//           wrist_detect_time_ = now;
//         }else if (end_bone_name == target){
//           wrist_bone_ = bone.end_point;
//           wrist_find_ = true;
//           wrist_detect_time_ = now;
//         }
//       }
//     }
//   }

//   const double MAX_AGE = 5.0;
//   if (shoulder_find_) {
//     double shoulder_age = (now - shoulder_detect_time_).toSec();
//     if (shoulder_age > MAX_AGE) {
//       shoulder_find_ = false;
//     }
//   }

//   if (wrist_find_) {
//     double wrist_age = (now - wrist_detect_time_).toSec();
//     if (wrist_age > MAX_AGE) {
//       wrist_find_ = false;
//     }
//   }
//   if (shoulder_find_ && wrist_find_) {
//     handup_flag_ = (wrist_bone_.y >= shoulder_bone_.y);
//   }else{
//     handup_flag_ = false;
//   }
// }
//

void ApproachingHuman::findBones(){
  if (bone_names_.empty() || bones_.empty()) {
    shoulder_find_ = false;
    left_wrist_find_ = false;
    right_wrist_find_ = false;
    handup_flag_ = false;
    return;
  }

  auto trim = [](const std::string& s) -> std::string {
                const auto first = s.find_first_not_of(" \t");
                if (first == std::string::npos) return std::string();
                const auto last = s.find_last_not_of(" \t");
                return s.substr(first, last - first + 1);
              };

  ros::Time now = ros::Time::now();

  for (size_t i = 0 ; i < bone_names_.size(); ++i){
    const auto& names = bone_names_[i];
    const auto& bones = bones_[i];

    if (names.size() != bones.size()) {
      ROS_WARN("bone_names_[%zu].size() != bones_[%zu].size()", i, i);
      continue;
    }

    bool has_left_eye  = false;
    bool has_right_eye = false;
    bool has_left_shoulder  = false;
    bool has_right_shoulder = false;
    bool has_left_wrist  = false;
    bool has_right_wrist = false;
    shoulder_find_ = false;
    left_wrist_find_ = false;
    right_wrist_find_ = false;

    geometry_msgs::Point left_shoulder_pt;
    geometry_msgs::Point right_shoulder_pt;
    geometry_msgs::Point left_wrist_pt;
    geometry_msgs::Point right_wrist_pt;

    for (size_t k = 0; k < names.size(); ++k) {
      const std::string& bone_name = names[k];
      const jsk_recognition_msgs::Segment& bone = bones[k];

      size_t pos = bone_name.find("->");
      if (pos == std::string::npos) {
        continue;
      }

      std::string start_bone_name = trim(bone_name.substr(0, pos));
      std::string end_bone_name   = trim(bone_name.substr(pos + 2));

      ROS_DEBUG("%zu, bone %zu: start: %s, end: %s", i, k, start_bone_name.c_str(), end_bone_name.c_str());

      if (start_bone_name == "left eye" || end_bone_name == "left eye") {
        has_left_eye = true;
      }
      if (start_bone_name == "right eye" || end_bone_name == "right eye") {
        has_right_eye = true;
      }

      if (start_bone_name == "left shoulder") {
        left_shoulder_pt = bone.start_point;
        has_left_shoulder = true;
      } else if (end_bone_name == "left shoulder") {
        left_shoulder_pt = bone.end_point;
        has_left_shoulder = true;
      }

      if (start_bone_name == "right shoulder") {
        right_shoulder_pt = bone.start_point;
        has_right_shoulder = true;
      } else if (end_bone_name == "right shoulder") {
        right_shoulder_pt = bone.end_point;
        has_right_shoulder = true;
      }

      if (start_bone_name == "left wrist") {
	left_wrist_pt = bone.start_point;
	has_left_wrist = true;
      } else if (end_bone_name == "left wrist") {
	left_wrist_pt = bone.end_point;
	has_left_wrist = true;
      }
      if (start_bone_name == "right wrist") {
        right_wrist_pt = bone.start_point;
	has_right_wrist = true;
      } else if (end_bone_name == "right wrist") {
	right_wrist_pt = bone.end_point;
	has_right_wrist = true;
      }

      bool eye_ok  = has_left_eye && has_right_eye;
      bool shoulder_pair_ok = has_left_shoulder && has_right_shoulder;


      // for (size_t j = 0 ; j < target_wrist_bones_.size(); ++j){
      // 	const std::string target = target_wrist_bones_[j];
      // 	if (start_bone_name == target){
      // 	  wrist_bone_ = bone.start_point;
      // 	  wrist_find_ = true;
      // 	  wrist_detect_time_ = now;
      // 	}else if (end_bone_name == target){
      // 	  wrist_bone_ = bone.end_point;
      // 	  wrist_find_ = true;
      // 	  wrist_detect_time_ = now;
      // 	}
      // }
      
      if (!eye_ok && !shoulder_pair_ok) {
        ROS_DEBUG("Skeleton %zu rejected: eye/shoulder pair not found", i);
        continue;
      }
      if(eye_ok){
	if (shoulder_pair_ok) {
	  double dx = left_shoulder_pt.x - right_shoulder_pt.x;
	  double dy = left_shoulder_pt.y - right_shoulder_pt.y;
	  shoulder_dist_ = std::sqrt(dx*dx + dy*dy);
	  
	  double mx = 0.5 * (left_shoulder_pt.x + right_shoulder_pt.x);
	  double my = 0.5 * (left_shoulder_pt.y + right_shoulder_pt.y);
	  shoulder_u_ = static_cast<int>(std::round(mx));
	  shoulder_v_ = static_cast<int>(std::round(my));
	  
	  shoulder_bone_ = left_shoulder_pt;
	  shoulder_find_ = true;
	  shoulder_detect_time_ = now;

	  if (has_left_wrist){
	    left_wrist_bone_ = left_wrist_pt;
	    left_wrist_find_ = true;
	    wrist_detect_time_ = now;
	  }
	  if (has_right_wrist){
	    right_wrist_bone_ = right_wrist_pt;
	    right_wrist_find_ = true;
	    wrist_detect_time_ = now;
	  }
	  if (has_right_wrist && has_right_wrist){
	    both_wrist_find_ = true;
	  }
	}
      }
    }
  }

  // const double MAX_AGE = 1.0;
  // if (shoulder_find_) {
  //   double shoulder_age = (now - shoulder_detect_time_).toSec();
  //   if (shoulder_age > MAX_AGE) {
  //     shoulder_find_ = false;
  //   }
  // }
  // if (wrist_find_) {
  //   double wrist_age = (now - wrist_detect_time_).toSec();
  //   if (wrist_age > MAX_AGE) {
  //     wrist_find_ = false;
  //   }
  // }
  if (shoulder_find_ && right_wrist_find_) {    
    if (right_wrist_bone_.y <= shoulder_bone_.y){
      handup_flag_ = true;
      if (left_wrist_find_ && left_wrist_bone_.y <= shoulder_bone_.y){
	both_handup_flag_ = true;
      }else{
	both_handup_flag_ = false;
      }
    }else{
      handup_flag_ = false;
      both_handup_flag_ = false;
    }
  }else{
    handup_flag_ = false;
  }
  ROS_INFO("shoulder:%d, wrist:%d, handup: %d", shoulder_find_, both_wrist_find_, handup_flag_);
}


// void ApproachingHuman::posCalc()
// {
//   // center relative to image center (pixels)
//   max_rect_pos_.x = static_cast<double>(max_rect_.x) + (static_cast<double>(max_rect_.width) / 2.0) - (static_cast<double>(camera_width_) / 2.0);
//   max_rect_pos_.y = - (static_cast<double>(max_rect_.y) + (static_cast<double>(max_rect_.height) / 2.0)) + (static_cast<double>(camera_height_) / 2.0);
//   ROS_INFO("max_rect_pos.x: %.3f", max_rect_pos_.x);
// }

void ApproachingHuman::posCalc()
{
  max_rect_pos_.x = static_cast<double>(shoulder_u_) + (static_cast<double>(max_rect_.width) / 2.0) - (static_cast<double>(camera_width_) / 2.0);
  max_rect_pos_.y = - (static_cast<double>(shoulder_v_) + (static_cast<double>(max_rect_.height) / 2.0)) + (static_cast<double>(camera_height_) / 2.0);
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

  std_msgs::Empty empty_msg;

  if (raw_depth_ > 0.0) {
    fail_safe_stop_cnt_ = 0;
    depth_check_start_ = true;
  } else {
    fail_safe_stop_cnt_++;
    if (fail_safe_stop_cnt_ > 5) {
      move_msg_.target_vel_x = 0.0;
      pub_move_.publish(move_msg_);
      ROS_INFO("stop!");
      depth_check_start_ = false;
      if (fail_safe_stop_cnt_ > 20) {
	ROS_INFO("fail safe land!");
	std_msgs::Empty e;
	pub_land_.publish(e);
      }
    }
    return;
  }

  if (depth_check_start_) {
    double new_depth_m = raw_depth_ / 1000.0 - 0.5;
    if (std::isnan(depth_)) {
      depth_ = new_depth_m;
    } else {
      const double MAX_STEP = 0.5;
      double step = new_depth_m - depth_;
      if (std::fabs(step) > MAX_STEP) {
	double clipped = depth_ + std::copysign(MAX_STEP, step);
	ROS_WARN_THROTTLE(1.0,"Depth jump too large (old=%.3f, new=%.3f). Clipping to %.3f",depth_, new_depth_m, clipped);
	depth_ = clipped;
      } else {
	depth_ = new_depth_m;
	if (min_depth_ >= depth_) {
	  if (std::abs(prev_depth_ - depth_) < 0.5) {
	    min_depth_ = depth_;
	  }
	}
      }
    }
  target_pos_.y = depth_;
  ROS_INFO("depth: %.3f", depth_);
  }
}

void ApproachingHuman::pdControl()
{
  if (std::isnan(prev_depth_)) {
    prev_depth_ = depth_;
  }
  
  double output = depth_ * Kp_ + (depth_ - prev_depth_) * Kd_;
  target_pos_.x = output;
  move_msg_.target_vel_x = output;
  ROS_INFO("vel: %.3f", output);

  prev_depth_ = depth_;
}


void ApproachingHuman::updateGazeAndExpressionWhileApproaching()
{
  // face expression change the iris movement
  float w    = static_cast<float>(camera_width_);
  float x_px = static_cast<float>(max_rect_pixel_pos_.x) - w * 0.5f;
  float x_norm = -(x_px / w) * 2.0f;
  gaze_x_.data = std::clamp(x_norm, -1.0f, 1.0f);
  pub_gaze_.publish(gaze_x_);

  float v = 0.0f;
  float a = 0.0f;
  
  // face expression change v
  if (depth_ <= 0.5f) {
    v = (0.5f - static_cast<float>(depth_)) / 0.5f * 0.5f + 0.4f;
    a = (0.5f - static_cast<float>(depth_)) / 0.5f * 0.5f + 0.4f;
      if (handup_flag_) {
      // 手を上げているなら、もう少し嬉しそうに
	v = std::min(1.0f, v + 0.2f);
	a = std::min(1.0f, a + 0.1f);
      }

    v = std::clamp(v, a, 1.0f);

    vad_array_.data = {v, 0.0f, 0.0f};

    if (v >= 0.5f) {
      gaze_x_.data = 0.0f;
      pub_gaze_.publish(gaze_x_);
    }

    face_first_change_flag_ = true;
  } else {
    v = 0.0f;
    a = 0.0f;
    float d = 0.0f;
    if (face_first_change_flag_) {
      v = -0.3f;
      a = 0.6f;
      d = 0.0f;
    }else{
      v = 0.0f;
      a = 0.0f;
      d = 0.0f;
    }
    if (handup_flag_) {
      v = 0.8f;
      a = 0.5f;
      d = 0.0f;
    }
    vad_array_.data = {v, a, d};
    face_first_change_flag_ = false;
  }
}

// void ApproachingHuman::updateAltitudeCommand()
// {
//   if (shoulder_find_) {
//     double target_h = shoulder_bone_.y;
//     move_msg_.target_pos_z = target_h;
//     move_msg_.pos_z_nav_mode = 1;
//     return;
//   }

//   double desired_y = 50.0;
//   double err_y = desired_y - max_rect_pos_.y;

//   double k_z = 0.002;
//   double dz = k_z * err_y;

//   double target_h = height_ + dz;
//   move_msg_.target_pos_z = target_h;
//   move_msg_.pos_z_nav_mode = 1;
// }



void ApproachingHuman::handleValidDepth()
{
  //move forward
  double speed_scale = 1.0;
  if (handup_flag_) {
    speed_scale = 1.5;
  }
  
  pdControl();
  move_msg_.target_vel_x *= speed_scale;

  const double TREND_THRESH = 0.15;
  updateGazeAndExpressionWhileApproaching();
  if (both_handup_flag_){
    vad_array_.data = {-0.6f, -0.4f, -0.5f};  // V<0, Aも少しマイナス寄り
    move_msg_.target_vel_x = -0.2;
    ROS_INFO("Human is going away... sad & back off");
  }

  pub_vad_.publish(vad_array_);
  // if (!std::isnan(depth_trend_prev_)) {
  //   bool going_forward = (move_msg_.target_vel_x > 0.05);
  //   bool getting_farther = (depth_ > depth_trend_prev_ + TREND_THRESH);
    
  //   if (going_forward && getting_farther) {

      
      
  //   }
  // }

  // depth_trend_prev_ = depth_;
  
  pub_move_.publish(move_msg_);
  ROS_INFO("go! (handup: %s, vel_x=%.3f, depth=%.3f)", handup_flag_ ? "true" : "false", move_msg_.target_vel_x, depth_);

  fail_safe_stop_cnt_ = 0;
}

void ApproachingHuman::handleInvalidDepth()
{
  move_msg_.target_vel_x = 0.0;
  pub_move_.publish(move_msg_);
  ROS_INFO("stop!");
}

void ApproachingHuman::handleRectDetected()
{
  findMaxRect();

  if (rotate_flag_) {
    rotateYaw();
    ROS_INFO("rotate!");
  }

  relativePos();
  posCalc();
  ROS_INFO("%d", land_cnt_);
  pub_target_pos_.publish(target_pos_);

  if (depth_ > 0.0) {
    handleValidDepth();
  } else {
    handleInvalidDepth();
  }

  updateLandCounter();

  if (land_cnt_ >= 10) {
    handleReachedHuman();
  }

  rotate_cnt_++;
}

void ApproachingHuman::updateLandCounter()
{
  // if (std::isnan(last_depth_for_land_)) {
  //   land_cnt_ = 0;
  //   return;
  // }

  // double diff = last_depth_for_land_ - depth_;

  const double NEAR_THRESH   = 0.1;
  if (min_depth_ < NEAR_THRESH) {
    land_cnt_++;
  }
}

void ApproachingHuman::handleReachedHuman()
{
  for (int i = 0; i < 10; ++i) {
    reach_to_human_flag_ = true;

    // face expression max v(smile)
    vad_array_.data = {1.0f, 0.8f, 0.0f};
    pub_vad_.publish(vad_array_);

    std_msgs::Bool msg;
    msg.data = reach_to_human_flag_;
    pub_reach_human_.publish(msg);
  }
  land_cnt_ = 0;
}

void ApproachingHuman::stopWithIdleExpression()
{
  move_msg_.target_vel_x = 0.0;
  pub_move_.publish(move_msg_);
  vad_array_.data = {0.0f, -1.0f, 0.0f};
  pub_vad_.publish(vad_array_);
}

void ApproachingHuman::handleNoRectDetected()
{
  move_msg_.target_vel_x = 0.0;
  pub_move_.publish(move_msg_);
  ROS_INFO("stop! because the robot can't see people");

  // face expression disgust
  vad_array_.data = {-0.5f, -0.3f, -0.8f};
  pub_vad_.publish(vad_array_);
  face_first_change_flag_ = true;

  // gaze movement
  if (dont_see_cnt_ % 2 == 0) {
    gaze_x_.data = -0.8f;
  } else {
    gaze_x_.data = 0.8f;
  }
  pub_gaze_.publish(gaze_x_);
  dont_see_cnt_++;

  // ある程度離れているなら少し yaw を回して探す
  if (min_depth_ > 1.0) {
    move_msg_.target_yaw = euler_.z + 0.01;
    pub_move_.publish(move_msg_);
    ROS_INFO("yaw rotate to see people: %.3f", move_msg_.target_yaw);
    ros::Duration(1.0).sleep();
  }
}

void ApproachingHuman::timerCb(const ros::TimerEvent&)
{
  std_msgs::Bool reach_msg;
  reach_msg.data = reach_to_human_flag_;
  pub_reach_human_.publish(reach_msg);

  if (reach_to_human_flag_ == true) {
    /////////////test//////////////////
    std_msgs::Empty e;
    pub_land_.publish(e);
    ///////////////////////////////////
    ROS_INFO("shutdown");
    timer_.stop();
    return;
  }

  flightRotateState();
  findBones();

  flight_state_flag_ = true;
  if (!flight_state_flag_) {
    stopWithIdleExpression();
    return;
  }else{
    if (n_ >= 1 && !rects_.rects.empty()) {
      handleRectDetected();
    }else{
      handleNoRectDetected();
    }
  }
}

void ApproachingHuman::spin()
{
  ros::spin();
}

int main_approaching_human(int argc, char** argv)
{
  ros::init(argc, argv, "Approaching_human");
  ApproachingHuman node;
  ros::spin();
  return 0;
}

// If you prefer a standalone main in this file:
int main(int argc, char** argv) { return main_approaching_human(argc, argv); }
