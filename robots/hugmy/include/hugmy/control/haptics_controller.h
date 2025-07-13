#ifndef HAPTICS_CONTROLLER_H
#define HAPTICS_CONTROLLER_H

#include <ros/ros.h>
#include <spinal/PwmTest.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>
#include <algorithm>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <sensor_msgs/Joy.h>

class HapticsController{
public:
    HapticsController(ros::NodeHandle& nh);
    virtual ~HapticsController() = default;
    void controlManual();
    virtual void controlAuto();
    void stopAllMotors();
    void setJoy(const sensor_msgs::Joy& msg);
    spinal::PwmTest getHapticsPwm() const;
    bool pos_flag_;

protected:
    void odomCb(const nav_msgs::Odometry::ConstPtr& msg);
    void publishHapticsPwm(const std::vector<uint8_t>& indices, const std::vector<float>& pwms);
    double calThrustPower(double strength);
    std::vector<float> computeMotorPwm(const Eigen::Vector2d& target_force);
    void outputPulsePattern(double target_force_norm, const std::vector<float>& motor_pwms);
    void vibratePwms();
    void isApproachingTarget(const Eigen::Vector2d& target_vec, double target_norm);
    bool isArmRaised(); 
    void toggleSwitch();

    ros::Publisher pwm_haptic_pub_;
    ros::Publisher marker_pub_;
    ros::Publisher alpha_pub_;
    ros::Subscriber odom_sub_;

    sensor_msgs::Joy joy_;
    geometry_msgs::Pose pose_;
    geometry_msgs::Vector3 euler_;
    geometry_msgs::Point last_pos_;
    double target_x_, target_y_;
    double output_;
    bool rest_toggle_ = false;
    int rest_count_ = 0;
    int pulse_count_ = 0;
    int pulse_target_ = 2;
    std::vector<float> motor_pwms_ = {0.5, 0.5, 0.5, 0.5};
    bool first_haptics_done_ = false;
    bool approaching_target_flag_ = true;
    ros::Time last_check_time_;
    double move_distance_threshold_ = 0.1;
    double direction_threshold_ = 0.8; 
    double pitch_threshold_ = 0.8;
    double roll_threshold_ = 0.8;
    int vibrate_count_ = 0;
    bool vibrate_toggle_ = true;
    spinal::PwmTest last_published_pwm_;
};

#endif
