#include <hugmy/control/haptics_controller.h>

HapticsController::HapticsController(ros::NodeHandle& nh){
    pwm_haptic_pub_ = nh.advertise<spinal::PwmTest>("/pwm_cmd/haptic", 1);
    emotion_pub_ = nh.advertise<std_msgs::Float32MultiArray>("/vad", 1);
    marker_pub_ = nh.advertise<visualization_msgs::Marker>("/target_marker", 1);
    alpha_pub_ = nh.advertise<spinal::PwmTest>("/motor_alpha", 1);
    thrust_pub_ = nh.advertise<spinal::Thrust>("/motor_haptics_thrust", 1);

    // odom_sub_ = nh.subscribe("/quadrotor/uav/cog/odom", 1, &HapticsController::odomCb, this);
    odom_sub_ = nh.subscribe("/Odometry", 1, &HapticsController::odomCb, this);
    imu_sub_ = nh.subscribe("/imu", 1, &HapticsController::imuCb, this);

    target_x_ = 0.0;
    target_y_ = 0.0;
    pos_flag_ = true;
    output_ = 0.0;
    motor_pwms_.assign(4,0.5f);
    ros::NodeHandle pnh("~");

    pnh.param("thrust_strength", thrust_strength_, 1.0);

    XmlRpc::XmlRpcValue wp_list;
    if (pnh.getParam("waypoints", wp_list)){
	if (wp_list.getType() != XmlRpc::XmlRpcValue::TypeArray){
	    ROS_ERROR("~waypoints must be an array");
	}else{
	  for (int i = 0; i < wp_list.size(); ++i){
	    if (wp_list[i].getType() != XmlRpc::XmlRpcValue::TypeArray || wp_list[i].size() != 2){
	      continue;
	    }
	    double x = static_cast<double>(wp_list[i][0]);
	    double y = static_cast<double>(wp_list[i][1]);	     
	    waypoints_.emplace_back(x, y);
	    }
	}
    }else{
      ROS_WARN("No ~waypoints parameter, auto mode will have no targets");
    }

    current_wp_idx_ = 0;
}

void HapticsController::publishHapticsPwm(const std::vector<uint8_t>& indices, const std::vector<float>& pwms) {
    spinal::PwmTest msg;
    msg.motor_index = indices;
    msg.pwms = pwms;
    pwm_haptic_pub_.publish(msg);
    last_published_pwm_ = msg;
}

void HapticsController::imuCb(const spinal::Imu::ConstPtr& msg){
    imu_ = *msg;
}

void HapticsController::odomCb(const nav_msgs::Odometry::ConstPtr& msg){
    const geometry_msgs::Pose parent_pose = msg->pose.pose;
  
    double qx = parent_pose.orientation.x;
    double qy = parent_pose.orientation.y;
    double qz = parent_pose.orientation.z;
    double qw = parent_pose.orientation.w;

    tf::Quaternion q(qx, qy, qz, qw);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    tf::Vector3 offset(0.4, 0.2, -0.3);
    tf::Vector3 offset_world = m * offset;

    pose_.position.x = parent_pose.position.x + offset_world.x();
    pose_.position.y = parent_pose.position.y + offset_world.y();
    pose_.position.z = parent_pose.position.z + offset_world.z();

    pose_.orientation = parent_pose.orientation;

    euler_.x = roll;
    euler_.y = pitch;
    euler_.z = yaw;
}

//thrust_strength_をわからなさに応じて変更できるようにする
double HapticsController::calThrustPower(double alpha) {
    thrust_ = std::min(5.0, base_thrust_ * thrust_strength_ * forward_gain_ * std::abs(alpha));
    ROS_ERROR("base_thrust: %.2f, thrust_strength: %.2f, forward_gain: %.2f, total_thrust: %.2f", base_thrust_,thrust_strength_, forward_gain_, thrust_);
    double pwm = -0.000679 * thrust_ * thrust_ + 0.044878 * thrust_ + 0.5;
    return std::min(pwm, 0.7);
}

void HapticsController::controlManual() {
    ROS_INFO("Manual mode start");

    if (joy_.axes.size() < 2) {
      ROS_WARN_THROTTLE(1.0, "Joy axes not received yet, skipping manual haptics.");
      return;
    }
    double x = -joy_.axes[0];
    double y = -joy_.axes[1];

    double deadzone = 0.05;
    x = (std::abs(x) > deadzone) ? x : 0.0;
    y = (std::abs(y) > deadzone) ? y : 0.0;

    Eigen::Vector2d target_vec(y,x);
    ROS_ERROR("Target force: (%.2f, %.2f)", target_vec.y(), target_vec.x());
    double target_norm = target_vec.norm();

    if (norm_mode_switch_ == 0){
      outputStrength(target_norm);
      motor_pwms_ = computeMotorPwmFixedTotal(target_vec, total_thrust_c_);
      int on_interval_default = 50;
      outputPulse(motor_pwms_, on_interval_default);
    }else{
      motor_pwms_ = computeMotorPwmFixedTotal(target_vec, total_thrust_c_);
      if (norm_mode_switch_ == 1){
        outputPulseLengthPattern(target_norm, motor_pwms_);
      }else if(norm_mode_switch_ == 2){
        outputPulsePattern(target_norm, motor_pwms_);
      }
    }
}

void HapticsController::controlAuto() {
    ROS_INFO("AUTO mode start");
    if (pos_flag_) {
      if (!waypoints_.empty()) {
	current_wp_idx_ = 0;
	target_x_ = waypoints_[0].x();
	target_y_ = waypoints_[0].y();
      } else {
	target_x_ = pose_.position.x + auto_target_x_;
        target_y_ = pose_.position.y + auto_target_y_;
      }
      
      last_pos_ = pose_.position;
      pos_flag_ = false;
      ROS_INFO("Current position: (%.2f, %.2f)", pose_.position.x, pose_.position.y);
      ROS_INFO("Target position : (%.2f, %.2f)", target_x_, target_y_);
    }
    
    Eigen::Vector2d cur_pos(pose_.position.x, pose_.position.y);
    Eigen::Vector2d tgt_pos(target_x_, target_y_);
    ROS_INFO("Current position: (%.2f, %.2f)", pose_.position.x, pose_.position.y);
    ROS_INFO("Target position : (%.2f, %.2f)", target_x_, target_y_);
    Eigen::Vector2d target_vec = tgt_pos - cur_pos;
    double target_norm = target_vec.norm();
    ROS_INFO("Target vector: (%.2f, %.2f), norm: %.2f", target_vec.x(), target_vec.y(), target_norm);
    
    min_target_norm_ = std::min(min_target_norm_, target_norm);

    if (min_target_norm_ < waypoint_reached_thresh_) {
      ROS_INFO("Waypoint %d reached (%.2f, %.2f).", current_wp_idx_, target_x_, target_y_);
      if (!waypoints_.empty() && current_wp_idx_ + 1 < (int)waypoints_.size()) {
	current_wp_idx_++;
	const Eigen::Vector2d& wp = waypoints_[current_wp_idx_];

	target_x_ = wp.x();
	target_y_ = wp.y();

	first_haptics_done_ = false;
	haptics_finished_flag_ = false;
	finished_cnt_ = 0;
	min_target_norm_ = std::numeric_limits<double>::infinity();
	ROS_INFO("Switching to waypoint %d: (%.2f, %.2f)", current_wp_idx_, target_x_, target_y_);
      } else {
	ROS_ERROR("Final waypoint reached, stopping motors.");
	vibratePwms();
	finished_cnt_ += 1;
	if (finished_cnt_ > 100){
	  haptics_finished_flag_ = true;
	  publishHapticsPwm({0,1,2,3}, {0.5, 0.5, 0.5, 0.5});
	}
	return;
      }
      tgt_pos << target_x_, target_y_;
      target_vec = tgt_pos - cur_pos;
      target_norm = target_vec.norm();
    }
    
    if (!first_haptics_done_) {
      if (norm_mode_switch_ == 0){
	outputStrength(target_norm);
	motor_pwms_ = computeMotorPwmFixedTotal(target_vec, total_thrust_c_);
	int on_interval_default = 50;
	outputPulse(motor_pwms_, on_interval_default);
      }else{
	motor_pwms_ = computeMotorPwmFixedTotal(target_vec, total_thrust_c_);
	if (norm_mode_switch_ == 1){
	  outputPulseLengthPattern(target_norm, motor_pwms_);
	}else if(norm_mode_switch_ == 2){
	  outputPulsePattern(target_norm, motor_pwms_);
	}
      }
      first_haptics_done_ = true;
    }

    if ((ros::Time::now() - last_check_time_).toSec() > 1.0) {  // 5s
      publishEmotion(target_vec,target_norm);
      approaching_target_flag_ = false;
      last_check_time_ = ros::Time::now();
    }
    
    //チェックはずっとやっていて，この状態がくるときは挙動を変えるようにする
    //もしtrue→true 出力しないまま
    // もしfalse→false　出力するまま
    // false→true　出力しなくなる
    // もしtrue→false　出力する
    //できればチェックの回数をもっと細かくして，最初の出力の挙動をもっとでかく
    // チェックはずっとしておく？　最初と，違うよーこっちだよーだけほしい
    // if ((ros::Time::now() - last_check_time_).toSec() > 5.0) {  // 5s
    //     isApproachingTarget(target_vec, target_norm);
    //     last_check_time_ = ros::Time::now();
    // }

    if (target_norm < 0.8 && haptics_finished_flag_ == false) {
        ROS_ERROR("target is close enough, stopping motors.");
        vibratePwms();
        finished_cnt_ += 1;
	emotion_msg_.data.resize(3);
	emotion_msg_.data[0] = 1.0;
	emotion_msg_.data[1] = 0.8;
	emotion_msg_.data[2] = 0.0;
	
	emotion_pub_.publish(emotion_msg_);
        if (finished_cnt_ > 100){
            haptics_finished_flag_ = true;
            publishHapticsPwm({0,1,2,3}, {0.5, 0.5, 0.5, 0.5});
        }
        return;
    }else{
        if (approaching_target_flag_) {
            // 目標近く → 出力しない
            ROS_INFO("Approaching target, not outputting haptics.");
            publishHapticsPwm({0,1,2,3}, {0.5, 0.5, 0.5, 0.5});
        } else {
            if (!isArmRaised()) {
                // 腕を下げている場合 → 振動パターン
                ROS_INFO("Arm is down, vibrating haptics.");
                vibratePwms();
            }else{
                // 腕を上げている場合
                // 目標遠い → 力覚提示を一定間隔で
                ROS_INFO("Arm is raised, outputting haptics PWM pattern.");
		if (norm_mode_switch_ == 0){
		  outputStrength(target_norm);
		  motor_pwms_ = computeMotorPwmFixedTotal(target_vec, total_thrust_c_);
		  int on_interval_default = 50;
		  outputPulse(motor_pwms_, on_interval_default);
		}else{
		  motor_pwms_ = computeMotorPwmFixedTotal(target_vec, total_thrust_c_);
		  if (norm_mode_switch_ == 1){
		    outputPulseLengthPattern(target_norm, motor_pwms_);
		  }else if(norm_mode_switch_ == 2){
		    outputPulsePattern(target_norm, motor_pwms_);
		  }
		}
            }
        }
    }
}

double HapticsController::computeDirectionGain(const Eigen::Vector2d& d_body)
{
    // 前後方向にどれだけ近いか
    double forwardness = std::abs(d_body.x());
    const double gain_min = 1.0;
    const double gain_max = 1.7;

    const double a = 3.85;
    double denom = std::exp(a) - 1.0;
    double t = 0.0;
    if (denom > 1e-9) {
      t = (std::exp(a * forwardness) - 1.0) / denom;
    }

    if (t < 0.0) t = 0.0;
    if (t > 1.0) t = 1.0;

    double g = gain_min + (gain_max - gain_min) * t;
    ROS_ERROR("gain: %.2f", g);
    return g;
}


void HapticsController::publishEmotion(const Eigen::Vector2d& target_vec, double target_norm)
{
  Eigen::Vector2d last_vec = target_vec - Eigen::Vector2d(last_pos_.x, last_pos_.y);
  Eigen::Vector2d delta_vec = Eigen::Vector2d(pose_.position.x, pose_.position.y) - Eigen::Vector2d(last_pos_.x, last_pos_.y);


  const double eps = 1e-6;

  if (last_vec.norm() > target_norm) {
    if (delta_vec.norm() < move_distance_threshold_) {
      a_ = 0.0;
    } else {
      a_ = 1.0;
    }

    if (last_vec.norm() > eps && delta_vec.norm() > eps) {
      v_ = last_vec.normalized().dot(delta_vec.normalized());
      v_ = std::max(-1.0, std::min(1.0, v_));
    } else {
      v_ = 0.0;
    }
  } else {
    double normalized = std::min(1.0, std::max(0.0,(target_norm - waypoint_reached_thresh_) / 3.0));
    double min_d = -1.0;
    double max_d = 1.0;
    d_ = min_d + normalized * (max_d - min_d); 
    d_ = std::max(-1.0, std::min(1.0, d_));
  }
  emotion_msg_.data.resize(3);
  emotion_msg_.data[0] = v_;
  emotion_msg_.data[1] = a_;
  emotion_msg_.data[2] = d_;

  emotion_pub_.publish(emotion_msg_);
  ROS_INFO("[HapticsController] emotion v=%.3f, a=%.3f, d=%.3f", v_, a_, d_);
}



std::vector<float> HapticsController::computeMotorPwm(const Eigen::Vector2d& target_vec){
    double cos_yaw = cos(euler_.z);
    double sin_yaw = sin(euler_.z);
    ROS_INFO("cos_yaw: %.2f, sin_yaw: %.2f", cos_yaw, sin_yaw);
    Eigen::Matrix<double, 2, 4> motor_dirs_base;
    motor_dirs_base <<  1, -1, -1,  1,
                    -1, -1,  1,  1;
    Eigen::Matrix2d R;
    R << cos_yaw, -sin_yaw,
         sin_yaw,  cos_yaw;
    Eigen::Matrix<double, 2, 4> motor_dirs = R * motor_dirs_base;
    // std::ostringstream oss;
    // oss << "motor_dirs:\n";
    // for (int row = 0; row < motor_dirs.rows(); ++row) {
    //     for (int col = 0; col < motor_dirs.cols(); ++col) {
    //         oss << motor_dirs(row, col) << "\t";
    //     }
    //     oss << "\n";
    // }
    // ROS_INFO_STREAM(oss.str());

    Eigen::Vector4d alpha =  Eigen::Vector4d::Zero();
    Eigen::Vector2d target_dir = target_vec.normalized();
    const int max_iter = 100;
    const double lr = 0.1;

    for (int i = 0; i < max_iter; ++i) {
        Eigen::Vector2d residual = motor_dirs * alpha - target_dir;
        Eigen::Vector4d gradient = motor_dirs.transpose() * residual;
        alpha -= lr * gradient;
        alpha = alpha.cwiseMax(0.0); // Ensure non-negative thrust
    }
    // alpha *= target_norm;
    
    for (size_t i = 0; i < 4; ++i) {
        motor_pwms_[i] = calThrustPower(alpha[i]);
    }

    //debug
    spinal::PwmTest alpha_msg;
    alpha_msg.motor_index = {0, 1, 2, 3};
    for (size_t i = 0; i < 4; ++i) {
        alpha_msg.pwms.push_back(static_cast<float>(alpha[i]));
    }
    alpha_pub_.publish(alpha_msg);

    return motor_pwms_;
}


Eigen::Vector4d HapticsController::computeAlphaFixedTotal(const Eigen::Vector2d& dir, double total_thrust_c)
{
    double cos_yaw = cos(euler_.z);
    double sin_yaw = sin(euler_.z);
    // ROS_INFO("cos_yaw: %.2f, sin_yaw: %.2f", cos_yaw, sin_yaw);
    Eigen::Vector4d alpha = Eigen::Vector4d::Zero();

    const double E = std::max(0.0, total_thrust_c);
    const double eps = 1e-9;

    double n = dir.norm();
    if (n < eps || E < eps) return alpha;
    Eigen::Vector2d d_world = dir / n;

    Eigen::Matrix<double,2,4> motor_base;
    // motor_base <<  1, -1, -1,  1,
    //      -1, -1,  1,  1;
    motor_base <<  -1, 1, 1, -1,
      1, 1,  -1,  -1;
    Eigen::Matrix2d R;
    R << cos_yaw, -sin_yaw,
         sin_yaw,  cos_yaw;
    Eigen::Matrix<double, 2, 4> M = R * motor_base;
    
    Eigen::Vector2d d_body = R.transpose() * d_world;

    const double kx = 1.7;
    const double ky = 1.0;
    Eigen::Vector2d d_body_scaled(kx * d_body.x(), ky * d_body.y());
    ROS_INFO("x: %.2f, y: %.2f", d_body.x(), d_body.y());
    ROS_INFO("x_scaled: %.2f, y: %.2f", d_body_scaled.x(), d_body_scaled.y());

    if (d_body_scaled.norm() > eps) {
        d_world = (R * d_body_scaled).normalized();
    } else {
        d_world = dir / n;
    }

    int best_idx = -1;
    double best_cos = -1.0;
    forward_gain_ = 1.0;
    for (int i = 0; i < 4; ++i) {
        Eigen::Vector2d ui = M.col(i).normalized();
        double c = ui.dot(d_world);
        if (c > best_cos) { best_cos = c; best_idx = i; }
    }

    bool found_pair = false;
    double best_err = 1e9;
    int bi=-1, bj=-1;
    Eigen::Vector2d bsol(0,0);

    for (int i = 0; i < 4; ++i) {
        for (int j = i+1; j < 4; ++j) {
            Eigen::Matrix2d P;
            P.col(0) = M.col(i);
            P.col(1) = M.col(j);
            double det = P.determinant();
            if (std::abs(det) < eps) continue;

            Eigen::Vector2d beta = P.inverse() * d_world;
            if (beta[0] < -1e-9 || beta[1] < -1e-9) continue;

            Eigen::Vector2d errv = P * beta - d_world;
            double err = errv.norm();
            if (err < best_err) {
                best_err = err; bi=i; bj=j; bsol = beta; found_pair = true;
            }
        }
    }

    if (found_pair) {
        forward_gain_ = computeDirectionGain(d_body);
        double norm_b = std::sqrt(std::max(0.0, bsol.squaredNorm()));
        if (norm_b < eps) return alpha;
        alpha[bi] = (bsol[0] / norm_b) * E;
        alpha[bj] = (bsol[1] / norm_b) * E;
        for (int k=0;k<4;++k) if (alpha[k] < 0 && alpha[k] > -1e-9) alpha[k] = 0.0;
        return alpha;
    }

    Eigen::Matrix2d MMt = M * M.transpose();
    Eigen::Vector2d x = MMt.ldlt().solve(d_world);
    alpha = M.transpose() * x;

    for (int k=0;k<4;++k) if (alpha[k] < 0 && alpha[k] > -1e-9) alpha[k] = 0.0;

    double na = std::sqrt(std::max(0.0, alpha.squaredNorm()));
    if (na >= eps) alpha *= (E / na); else alpha.setZero();

    return alpha;
}


std::vector<float> HapticsController::computeMotorPwmFixedTotal(const Eigen::Vector2d& target_vec, double total_thrust_c)
{
    Eigen::Vector4d alpha = computeAlphaFixedTotal(target_vec, total_thrust_c);
    spinal::Thrust thrust_msg;

    for (int i = 0; i < 4; ++i) if (alpha[i] < 0 && alpha[i] > -1e-6) alpha[i] = 0;

    for (int i = 0; i < 4; ++i) {
        motor_pwms_[i] = calThrustPower(alpha[i]);
        thrust_msg.thrust.push_back(static_cast<float>(thrust_));
    }

    // debug publish
    spinal::PwmTest alpha_msg;
    alpha_msg.motor_index = {0,1,2,3};
    for (int i = 0; i < 4; ++i) alpha_msg.pwms.push_back(static_cast<float>(alpha[i]));
    alpha_pub_.publish(alpha_msg);

    thrust_pub_.publish(thrust_msg);

    return motor_pwms_;
}



void HapticsController::vibratePwms(){
    if (vibrate_count_ > 5) {
        vibrate_toggle_ = !vibrate_toggle_;
        vibrate_count_ = 0;
    }
    if (vibrate_toggle_) {
        publishHapticsPwm({0,1,2,3}, {0.6, 0.6, 0.5, 0.5});
    } else {
        publishHapticsPwm({0,1,2,3}, {0.5, 0.5, 0.6, 0.6});
        //ros::Duration(0.5).sleep();
    }
    vibrate_count_ += 1;
    ROS_INFO("Vibration pattern completed.");
}
void HapticsController::outputPulsePattern(double target_norm, const std::vector<float>& motor_pwms){
    static const int rest_toggle_interval = 50;

    if (in_cooldown_) {
        double elapsed = (ros::Time::now() - cooldown_start_).toSec();
        if (elapsed < cooldown_duration_sec_) {
            publishHapticsPwm({0,1,2,3}, {0.5, 0.5, 0.5, 0.5});
            ROS_INFO_THROTTLE(1.0, "In cooldown (%.2f / %.2f sec)", elapsed, cooldown_duration_sec_);
            return;
        } else {
            in_cooldown_ = false;
            pulse_count_ = 0;
            rest_count_ = 0;
            rest_toggle_ = false;
            ROS_INFO("Cooldown finished, restarting pulse pattern.");
        }
    }


    if (pulse_count_ == 0) {
        if (target_norm < 1.5) {
            pulse_target_ = 1 * 2;
        } else if (target_norm < 2.0) {
            pulse_target_ = 2 * 2;
	} else if (target_norm < 2.5) {
            pulse_target_ = 3 * 2;
        } else {
            pulse_target_ = 4 * 2;
        }
    }

    rest_count_ += 1;
    if (rest_count_ > rest_toggle_interval) {
        rest_toggle_ = !rest_toggle_;
        rest_count_ = 0;
        pulse_count_ += 1;
    }

    if (pulse_count_ < pulse_target_) {
        if (rest_toggle_) {
            publishHapticsPwm({0,1,2,3}, {0.5, 0.5, 0.5, 0.5});
        } else {
            publishHapticsPwm({0,1,2,3}, motor_pwms_);
        }
        ROS_INFO("Pulse pattern %d/%d, rest_toggle: %s",
                 pulse_count_ + 1, pulse_target_, rest_toggle_ ? "ON" : "OFF");
    } else {
        pulse_count_ = 0;
        rest_count_ = 0;
        rest_toggle_ = false;

        in_cooldown_ = true;
        cooldown_start_ = ros::Time::now();

        ROS_ERROR("Pulse pattern completed, entering cooldown (%.2f sec).",
                  cooldown_duration_sec_);
    }
}

void HapticsController::outputPulseLengthPattern(double target_norm, const std::vector<float>& motor_pwms){
    static const int min_on_interval = 10;
    static const int max_on_interval = 110;

    double normalized = std::min(1.0, std::max(0.0, target_norm / 3.0));
    int on_interval = min_on_interval + normalized * (max_on_interval - min_on_interval);

    outputPulse(motor_pwms, on_interval);
}

void HapticsController::outputPulse(const std::vector<float>& motor_pwms, int on_interval){
    static const int base_interval = 50;
    static const int rest_toggle_interval = 50;

    rest_count_ += 1;
    if (rest_toggle_) {
      publishHapticsPwm({0,1,2,3}, motor_pwms_);
      if (rest_count_ > on_interval) {
        rest_toggle_ = false;
        rest_count_ = 0;
      }
    } else {
      publishHapticsPwm({0,1,2,3}, {0.5,0.5,0.5,0.5});
      if (rest_count_ > base_interval) {
        rest_toggle_ = true;
        rest_count_ = 0;
      }
    }
    ROS_INFO("Pulse pattern: ON for %d, OFF for %d" ,on_interval, base_interval);
}

void HapticsController::outputStrength(double target_norm)
{
    const double d_max = 5.0;
    double x = std::min(std::max(0.001, target_norm/d_max),1.0);
    const double r_min = 2.5;
    const double r_max = 4.0;
    double rating = r_min + (r_max - r_min) * x;

    const double a = 2.5086;
    const double b = 0.9222;

    double F_des = std::exp((rating - b) / a);
    
    double raw_strength = F_des / base_thrust_;

    thrust_strength_ = std::min(1.2, std::max(0.53, raw_strength));
}


void HapticsController::isApproachingTarget(const Eigen::Vector2d& target_vec, double target_norm) {
    Eigen::Vector2d last_vec_ = Eigen::Vector2d(target_x_, target_y_) - Eigen::Vector2d(last_pos_.x, last_pos_.y);
    Eigen::Vector2d delta_vec_ = Eigen::Vector2d(pose_.position.x, pose_.position.y) - Eigen::Vector2d(last_pos_.x, last_pos_.y);
    approaching_target_flag_ = false;
    // if (last_vec_.norm() > target_norm) {
    //     if (delta_vec_.norm() < move_distance_threshold_) {
    //         approaching_target_flag_ = false;  // 動いていない
    //         return;
    //     }
    //     double dot = last_vec_.normalized().dot(delta_vec_.normalized());
    //     if (dot > direction_threshold_) {
    //         last_pos_ = pose_.position;
    //         approaching_target_flag_ = true;  // 一定以上方向が合っている
    //     }else{
    //         approaching_target_flag_ = false;
    //     }
    // }else{
    //     approaching_target_flag_ = false;
    // }
}

void HapticsController::toggleSwitch() {
    rest_count_ += 1;
    if (rest_count_ > 50) {
        rest_toggle_ = !rest_toggle_;
        rest_count_ = 0;
    }
}

bool HapticsController::isArmRaised() {
    // 角度の閾値を設定
    double roll  = imu_.angles[0];
    double pitch = imu_.angles[1];
    ROS_INFO("Checking if arm is raised with roll: %.2f, pitch: %.2f", roll,pitch);
    return (std::abs(roll) < roll_threshold_) && (std::abs(pitch) < pitch_threshold_);
}

void HapticsController::stopAllMotors() {
    ROS_INFO("Stopping all motors.");
    publishHapticsPwm({0,1,2,3}, {0.5, 0.5, 0.5, 0.5});
}
