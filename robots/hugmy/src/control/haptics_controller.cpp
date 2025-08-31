#include <hugmy/control/haptics_controller.h>

HapticsController::HapticsController(ros::NodeHandle& nh){
    pwm_haptic_pub_ = nh.advertise<spinal::PwmTest>("/pwm_cmd/haptic", 1);
    marker_pub_ = nh.advertise<visualization_msgs::Marker>("/target_marker", 1);
    alpha_pub_ = nh.advertise<spinal::PwmTest>("/motor_alpha", 1);

    odom_sub_ = nh.subscribe("/quadrotor/uav/cog/odom", 1, &HapticsController::odomCb, this);

    target_x_ = 0.0;
    target_y_ = 0.0;
    pos_flag_ = true;
    output_ = 0.0;
}

void HapticsController::publishHapticsPwm(const std::vector<uint8_t>& indices, const std::vector<float>& pwms) {
    spinal::PwmTest msg;
    msg.motor_index = indices;
    msg.pwms = pwms;
    pwm_haptic_pub_.publish(msg);
    last_published_pwm_ = msg;
}

void HapticsController::odomCb(const nav_msgs::Odometry::ConstPtr& msg){
    pose_ = msg -> pose.pose;
    double qx = pose_.orientation.x;
    double qy = pose_.orientation.y;
    double qz = pose_.orientation.z;
    double qw = pose_.orientation.w;

    tf::Quaternion q(qx, qy, qz, qw);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    euler_.x = roll;
    euler_.y = pitch;
    euler_.z = yaw;
}

double HapticsController::calThrustPower(double strength) {
    double thrust = 5.0 * std::abs(strength);
    double pwm = -0.000679 * thrust * thrust + 0.044878 * thrust + 0.5;
    return std::min(pwm, 0.7);
}

void HapticsController::controlManual() {
    ROS_INFO("Manual mode start");
    double x = -joy_.axes[0];
    double y = joy_.axes[1];

    double deadzone = 0.05;
    x = (std::abs(x) > deadzone) ? x : 0.0;
    y = (std::abs(y) > deadzone) ? y : 0.0;

    Eigen::Vector2d target_force(x,y);
    ROS_ERROR("Target force: (%.2f, %.2f)", target_force.x(), target_force.y());
    double target_force_norm = target_force.norm();
    std::vector<float> motor_pwms(4, 0.5);

    Eigen::Matrix<double, 2, 4> motor_dirs;
    motor_dirs <<  1, -1, -1,  1,
                 -1, -1,  1,  1;
    Eigen::Vector4d alpha =  Eigen::Vector4d::Zero();
    Eigen::Vector2d target_force_dir = target_force.normalized();
    const int max_iter = 100;
    const double lr = 0.1;

    for (int i = 0; i < max_iter; ++i) {
        Eigen::Vector2d residual = motor_dirs * alpha - target_force_dir;
        Eigen::Vector4d gradient = motor_dirs.transpose() * residual;
        alpha -= lr * gradient;
        alpha = alpha.cwiseMax(0.0); // Ensure non-negative thrust
    }
    alpha *= target_force_norm;

    if (target_force_norm < 1e-6) {
        publishHapticsPwm({0,1,2,3}, {0.5, 0.5, 0.5, 0.5});
        return;
    }else{
        for (size_t i = 0; i < 4; ++i) {
        motor_pwms[i] = calThrustPower(alpha[i]);
        }   
        publishHapticsPwm({0,1,2,3}, motor_pwms);
    }

    //debug
    spinal::PwmTest alpha_msg;
    alpha_msg.motor_index = {0, 1, 2, 3};
    for (size_t i = 0; i < 4; ++i) {
        alpha_msg.pwms.push_back(static_cast<float>(alpha[i]));
    }
    alpha_pub_.publish(alpha_msg);
}

void HapticsController::controlAuto() {
    ROS_INFO("AUTO mode start");
    if (pos_flag_) {
        target_x_ = pose_.position.x - 0.5;
        target_y_ = pose_.position.y - 0.5;
        last_pos_ = pose_.position;
        pos_flag_ = false;
        ROS_INFO("Current position: (%.2f, %.2f)", pose_.position.x, pose_.position.y);
        ROS_INFO("Target position : (%.2f, %.2f)", target_x_, target_y_);
    }

    Eigen::Vector2d target_vec = Eigen::Vector2d(target_x_, target_y_) - Eigen::Vector2d(pose_.position.x, pose_.position.y);
    double target_norm = target_vec.norm();
    ROS_INFO("Target vector: (%.2f, %.2f), norm: %.2f", target_vec.x(), target_vec.y(), target_norm);
    if (!first_haptics_done_) {
        std::vector<float> motor_pwms = computeMotorPwm(target_vec);
        ROS_INFO("Outputting initial haptics PWM pattern.");
        outputPulsePattern(target_norm, motor_pwms);
        first_haptics_done_ = true;
    }
    //チェックはずっとやっていて，この状態がくるときは挙動を変えるようにする
    //もしtrue→true 出力しないまま
    // もしfalse→false　出力するまま
    // false→true　出力しなくなる
    // もしtrue→false　出力する
    //できればチェックの回数をもっと細かくして，最初の出力の挙動をもっとでかく
    // チェックはずっとしておく？　最初と，違うよーこっちだよーだけほしい
    if ((ros::Time::now() - last_check_time_).toSec() > 2.0) {  // 5s
        isApproachingTarget(target_vec, target_norm);
        last_check_time_ = ros::Time::now();
    }

    if (target_norm < 0.05 && haptics_finished_flag_ == false) {
        ROS_ERROR("target is close enough, stopping motors.");
        vibratePwms();
        finished_cnt_ += 1;
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
                motor_pwms_ = computeMotorPwm(target_vec);
                outputPulsePattern(target_norm, motor_pwms_);
                
            }
        }
    }   
}

std::vector<float> HapticsController::computeMotorPwm(const Eigen::Vector2d& target_vec){
    double cos_yaw = cos(euler_.z);
    double sin_yaw = sin(euler_.z);
    // ROS_INFO("cos_yaw: %.2f, sin_yaw: %.2f", cos_yaw, sin_yaw);
    // transform to local coordinate
    // double dx_body = dx * cos_yaw + dy * sin_yaw;
    // double dy_body = -dx * sin_yaw + dy * cos_yaw;
    // Eigen::Vector2d target_force(dx_body, dy_body);
    // ROS_INFO("dx_body: %.2f, dy_body: %.2f", dx_body, dy_body);
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

void HapticsController::vibratePwms(){
    if (vibrate_count_ > 5) {
        vibrate_toggle_ = !vibrate_toggle_;
        vibrate_count_ = 0;
    }
    if (vibrate_toggle_) {
        publishHapticsPwm({0,1,2,3}, {0.65, 0.65, 0.5, 0.5});
    } else {
        publishHapticsPwm({0,1,2,3}, {0.5, 0.5, 0.65, 0.65});
        //ros::Duration(0.5).sleep();
    }
    vibrate_count_ += 1;
    ROS_INFO("Vibration pattern completed.");
}

void HapticsController::outputPulsePattern(double target_norm, const std::vector<float>& motor_pwms){
    static const int rest_toggle_interval = 50;

    if (pulse_count_ == 0) {
        if (target_norm < 0.1) {
            pulse_target_ = 1*2;
        } else if (target_norm < 0.3) {
            pulse_target_ = 2*2; 
        } else {
            pulse_target_ = 3*2; 
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
        ROS_INFO("Pulse pattern %d/%d, rest_toggle: %s", pulse_count_ + 1, pulse_target_, rest_toggle_ ? "ON" : "OFF");
    }else{
        pulse_count_ = 0; 
        rest_count_ = 0; 
        rest_toggle_ = false; 
        ROS_ERROR("Pulse pattern completed, resetting.");
    }
}

void HapticsController::isApproachingTarget(const Eigen::Vector2d& target_vec, double target_norm) {
    Eigen::Vector2d last_vec_ = Eigen::Vector2d(target_x_, target_y_) - Eigen::Vector2d(last_pos_.x, last_pos_.y);
    Eigen::Vector2d delta_vec_ = Eigen::Vector2d(pose_.position.x, pose_.position.y) - Eigen::Vector2d(last_pos_.x, last_pos_.y);

    if (last_vec_.norm() > target_norm) {
        if (delta_vec_.norm() < move_distance_threshold_) {
            approaching_target_flag_ = false;  // 動いていない
            return;
        }
        double dot = last_vec_.normalized().dot(delta_vec_.normalized());
        if (dot > direction_threshold_) {
            last_pos_ = pose_.position;
            approaching_target_flag_ = true;  // 一定以上方向が合っている
        }else{
            approaching_target_flag_ = false;
        }
    }else{
        approaching_target_flag_ = false;
    }
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
    ROS_INFO("Checking if arm is raised with roll: %.2f, pitch: %.2f", euler_.x, euler_.y);
    return (std::abs(euler_.x) < roll_threshold_) && (std::abs(euler_.y) < pitch_threshold_);
}

void HapticsController::stopAllMotors() {
    ROS_INFO("Stopping all motors.");
    publishHapticsPwm({0,1,2,3}, {0.5, 0.5, 0.5, 0.5});
}
