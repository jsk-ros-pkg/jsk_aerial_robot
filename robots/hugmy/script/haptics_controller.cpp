#include "haptics_controller.h"

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

void HapticsController::update() {
    publishTargetMarker();
}

spinal::PwmTest HapticsController::getHapticsPwm() const {
    return last_published_pwm_;
}

void HapticsController::setJoy(const sensor_msgs::Joy& msg) {
    joy_ = msg;
}

void HapticsController::publishHapticsPwm(const std::vector<int>& indices, const std::vector<float>& pwms) {
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

void HapticsController::publishTargetMarker() {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "target";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = target_x_;
    marker.pose.position.y = target_y_;
    marker.pose.position.z = 0.8;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker_pub_.publish(marker);
}

double HapticsController::calThrustPower(double strength_norm) {
    double thrust = 4.0 * std::abs(strength_norm);
    double pwm = -0.000679 * thrust * thrust + 0.044878 * thrust + 0.5;
    return std::min(pwm, 0.65);
}

void HapticsController::controlManual() {
    ROS_INFO("Manual mode start");
    double x = -joy_.axes[0];
    double y = joy_.axes[1];

    double deadzone = 0.05;
    x = (std::abs(x) > deadzone) ? x : 0.0;
    y = (std::abs(y) > deadzone) ? y : 0.0;

    Eigen::Vector2d target_force(x,y);
    Eigen::Matrix<double, 2, 4> motor_dirs;
    motor_dirs <<  1, -1, -1,  1,
                 -1, -1,  1,  1;
    Eigen::Vector4d alpha =  Eigen::Vector4d::Zero();
    Eigen::Vector4d alpha_tmp = Eigen::Vector4d::Zero();
    const int max_iter = 100;
    const double ls = 0.1;

    if (target_force.norm() > 1e-6) {
        target_force.normalize();
        target_force *= std::hypot(x, y);
    }

    for (int i = 0; i < max_iter; ++i) {
        Eigen::Vector2d residual = motor_dirs * alpha - target_force;
        Eigen::Vector2d gradient = motor_dirs.transpose() * residual;
        alpha_tmp = lr * gradient;
        alpha_tmp = alpha_tmp.cwiseMax(0.0); // Ensure non-negative thrust
    }
    alpha = alpha_tmp;

    std::vector<float> motor_pwms(4, 0.5);
    if (x != 0.0 || y != 0.0) {
        double norm = std::hypot(x, y);
        std::vector<double> direction = { (norm > 1e-6) ? x / norm : 0.0,
                                            (norm > 1e-6) ? y / norm : 0.0 };

        std::vector<std::vector<double>> motor_dirs = {
            {1, -1}, {-1, -1}, {-1, 1}, {1, 1}
        };

        std::vector<double> alpha(4, 0.0);

        for (size_t i = 0; i < 4; ++i) {
            double dot = motor_dirs[i][0] * direction[0] + motor_dirs[i][1] * direction[1];
            alpha[i] = std::max(dot * norm, 0.0);
        }

        for (size_t i = 0; i < 4; ++i) {
            motor_pwms[i] = calThrustPower(alpha[i]);
        }
    }

    publishHapticsPwm({0,1,2,3}, motor_pwms);

    //debag
    spinal::PwmTest alpha_msg;
    alpha_msg.motor_index = {0, 1, 2, 3};  // モータ番号
    for (size_t i = 0; i < 4; ++i) {
        alpha_msg.pwms.push_back(static_cast<float>(alpha[i]));
    }
    alpha_pub_.publish(alpha_msg);
}

void HapticsController::controlAuto() {
    ROS_INFO("AUTO mode start");
    if (pos_flag_) {
        target_x_ = pose_.position.x + 0.5;
        target_y_ = pose_.position.y + 0.5;
        pos_flag_ = false;
        ROS_INFO("Target position set to: (%.2f, %.2f)", target_x_, target_y_);
    }

    double dx = target_x_ - pose_.position.x;
    double dy = target_y_ - pose_.position.y;
    ROS_INFO("dx: %.2f, dy: %.2f", dx, dy);

    double cos_yaw = cos(-euler_.z);
    double sin_yaw = sin(-euler_.z);
    double dx_body = dx * cos_yaw + dy * sin_yaw;
    double dy_body = -dx * sin_yaw + dy * cos_yaw;

    std::vector<float> motor_pwms(4, 0.5);
    if (std::abs(dx_body) > 0.1 || std::abs(dy_body) > 0.1) {
        std::vector<double> dir = {dx_body, dy_body};
        double norm = std::hypot(dx_body, dy_body);
        std::vector<double> dir_n = { (norm > 1e-6) ? dx_body / norm : 0.0,
                                        (norm > 1e-6) ? dy_body / norm : 0.0 };

        std::vector<std::vector<double>> motor_dirs = {
            {1, -1}, {-1, -1}, {-1, 1}, {1, 1}
        };

        std::vector<double> alpha(4, 0.0);
        for (size_t i = 0; i < 4; ++i) {
            double dot = motor_dirs[i][0] * dir_n[0] + motor_dirs[i][1] * dir_n[1];
            alpha[i] = std::max(dot * norm, 0.0);
        }

        for (size_t i = 0; i < 4; ++i) {
            motor_pwms[i] = calThrustPower(alpha[i]);
        }
    }
    publishHapticsPwm({0,1,2,3}, motor_pwms);

    //debag
    spinal::PwmTest alpha_msg;
    alpha_msg.motor_index = {0, 1, 2, 3};  // モータ番号
    for (size_t i = 0; i < 4; ++i) {
        alpha_msg.pwms.push_back(static_cast<float>(alpha[i]));
    }
    alpha_pub_.publish(alpha_msg);
}

void HapticsController::stopAllMotors() {
    ROS_INFO("Stopping all motors.");
    publishHapticsPwm({0,1,2,3}, {0.5, 0.5, 0.5, 0.5});
}
