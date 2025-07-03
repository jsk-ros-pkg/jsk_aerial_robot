#ifndef AIR_PRESSURE_CONTROLLER_H
#define AIR_PRESSURE_CONTROLLER_H

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <spinal/PwmTest.h>
#include <algorithm>

class AirPressureController {
public:
    AirPressureController(ros::NodeHandle& nh);

    void update();
    void stopPump();
    void stopAllPneumatics();

    double getOutput() const;
    int getAirPressureJoint() const;
    int getAirPressureBottom() const;
    spinal::PwmTest getAirPwm() const;

private:
    void sensorCb(const std_msgs::Int8::ConstPtr& msg);
    void sensor1Cb(const std_msgs::Int8::ConstPtr& msg);
    void publishAirPwm(const std::vector<uint8_t>& indices, const std::vector<float>& pwms);
    // void adjustPump();
    // void startSVSwitch();
    // void stopSVSwitch();
    // void startSVExhaust();
    // void stopSVExhaust();
    void calPressure(int target_pressure, int sensor_index);
    void adjustAirPressure();

    ros::Subscriber sensor_joint_sub_;
    ros::Subscriber sensor_bottom_sub_;
    ros::Publisher pwm_air_pub_;

    int air_pressure_joint_;
    int air_pressure_bottom_;
    double output_;
    int joint_max_pressure_;
    int bottom_max_pressure_;
    int joint_perching_pressure_;
    int bottom_usual_pressure_;

    spinal::PwmTest pwm_air_cmd_;
    spinal::PwmTest last_published_pwm_;
};

#endif // AIR_PRESSURE_CONTROLLER_H