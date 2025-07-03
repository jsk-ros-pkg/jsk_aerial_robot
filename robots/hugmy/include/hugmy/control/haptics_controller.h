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
    void update();
    void controlManual();
    void controlAuto();
    void stopAllMotors();
    void setJoy(const sensor_msgs::Joy& msg);
    spinal::PwmTest getHapticsPwm() const;
    bool pos_flag_;

private:
    void odomCb(const nav_msgs::Odometry::ConstPtr& msg);
    void publishTargetMarker();
    void publishHapticsPwm(const std::vector<uint8_t>& indices, const std::vector<float>& pwms);
    double calThrustPower(double strength);

    ros::Publisher pwm_haptic_pub_;
    ros::Publisher marker_pub_;
    ros::Publisher alpha_pub_;
    ros::Subscriber odom_sub_;

    sensor_msgs::Joy joy_;
    geometry_msgs::Pose pose_;
    geometry_msgs::Vector3 euler_;
    double target_x_, target_y_;
    double output_;
    spinal::PwmTest last_published_pwm_;
};

#endif
