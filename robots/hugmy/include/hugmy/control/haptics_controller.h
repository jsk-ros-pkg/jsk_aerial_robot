#ifndef HAPTICS_CONTROLLER_H
#define HAPTICS_CONTROLLER_H

#include <ros/ros.h>
#include <spinal/PwmTest.h>
#include <spinal/Thrust.h>
#include <spinal/Imu.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Float32MultiArray.h>
#include <algorithm>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <sensor_msgs/Joy.h>
#include <XmlRpcValue.h>
// #include <tf/transform_listener.h>

class HapticsController{
public:
    HapticsController(ros::NodeHandle& nh);
    virtual ~HapticsController() = default;
    void controlManual();
    virtual void controlAuto();
    void stopAllMotors();
    void setJoy(const sensor_msgs::Joy& msg) { joy_ = msg; };
    void setNormModeSwitch(int norm_switch) { norm_mode_switch_ = norm_switch; };
    void setBaseThrust(double base_thrust) { base_thrust_ = base_thrust; };
    void setEmotionSwitch(bool emotion_switch) {emotion_switch_ = emotion_switch; };
    spinal::PwmTest getHapticsPwm() const {return last_published_pwm_; }
    bool getHapticsFinished() const { return haptics_finished_flag_; }
    bool pos_flag_;
    void vibratePwms();
    Eigen::Vector4d computeAlphaFixedTotal(const Eigen::Vector2d& target_vec, double total_thrust_c);
    std::vector<float> computeMotorPwmFixedTotal(const Eigen::Vector2d& target_vec, double total_thrust_c);

    double auto_target_x_= 0.5;
    double auto_target_y_= 0.5;
    std::vector<Eigen::Vector2d> waypoints_;
    int current_wp_idx_ = 0;
    double waypoint_reached_thresh_ = 0.8;
protected:
    void odomCb(const nav_msgs::Odometry::ConstPtr& msg);
    void imuCb(const spinal::Imu::ConstPtr& msg);
    void publishHapticsPwm(const std::vector<uint8_t>& indices, const std::vector<float>& pwms);
    double calThrustPower(double alpha);
    std::vector<float> computeMotorPwm(const Eigen::Vector2d& target_force);
    void outputPulsePattern(double target_force_norm, const std::vector<float>& motor_pwms);
    void isApproachingTarget(const Eigen::Vector2d& target_vec, double target_norm);
    bool isArmRaised();
    void toggleSwitch();
    double computeDirectionGain(const Eigen::Vector2d& d_body);

    void outputStrength(double target_norm);
    void outputPulse(const std::vector<float>& motor_pwms, int on_interval);
    void outputPulseLengthPattern(double target_norm, const std::vector<float>& motor_pwms);

    void publishEmotion(const Eigen::Vector2d& target_vec, double target_norm);

    // tf::TransformListener tfListener_;
  
    ros::Publisher pwm_haptic_pub_;
    ros::Publisher emotion_pub_;
    ros::Publisher marker_pub_;
    ros::Publisher alpha_pub_;
    ros::Publisher thrust_pub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber imu_sub_;

    sensor_msgs::Joy joy_;
    geometry_msgs::Pose pose_;
    geometry_msgs::Vector3 euler_;
    geometry_msgs::Point last_pos_;
    std_msgs::Float32MultiArray emotion_msg_;
    spinal::Imu imu_;
    double target_x_, target_y_;
    double output_;
    bool rest_toggle_ = false;
    int rest_count_ = 0;
    int pulse_count_ = 0;
    int pulse_target_ = 2;
    int norm_mode_switch_ = 0;
    std::vector<float> motor_pwms_ = {0.5, 0.5, 0.5, 0.5};
    bool first_haptics_done_ = false;
    bool haptics_finished_flag_ = false;
    bool approaching_target_flag_ = true;
    ros::Time last_check_time_;
    double move_distance_threshold_ = 0.1;
    double direction_threshold_ = 0.8;
    double pitch_threshold_ = 0.8;
    double roll_threshold_ = 0.8;
    int vibrate_count_ = 0;
    bool vibrate_toggle_ = true;
    int finished_cnt_ = 0;
    spinal::PwmTest last_published_pwm_;
    double thrust_ = 0.0;

    double min_target_norm_ = std::numeric_limits<double>::infinity(); 

    bool in_cooldown_ = false;
    ros::Time cooldown_start_;
    double cooldown_duration_sec_ = 1.0; 
    double forward_gain_ = 1.0;
  
    double thrust_strength_ = 1.0;
    double total_thrust_c_ = 1.0;

    double base_thrust_ = 3.0;
    bool emotion_switch_ = false;
    double v_ = 0.0;
    double a_ = 0.0;
    double d_ = 0.0;
};

#endif
