#ifndef AIR_PRESSURE_CONTROLLER_H
#define AIR_PRESSURE_CONTROLLER_H

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <spinal/PwmTest.h>
#include <algorithm>
#include <vector>
#include <cstdint>
#include <map>

class AirPressureController {
public:
    AirPressureController(ros::NodeHandle& nh);

    // void update();
    void stopAllPneumatics();
    void maxWorkPumptoJoint();
    void initializePneumatics();

    double getOutput() const {return output_; }
    int getAirPressureJoint() const { return air_pressure_joint_; }
    int getAirPressureBottom() const { return air_pressure_bottom_; }
    void setPerchingState(int state) { perching_flag_ = state; }
    int getPerchingState() const { return perching_flag_; }
    void setPrepareFinished(int v) { prepare_finished_ = v; }
    int getPrepareFinished() const { return prepare_finished_; }
    spinal::PwmTest getAirPwm() const { return last_published_pwm_; }

    void bottomPressurePrepare();
    void readyPerching();
    void startPerching();
    void keepPerching();

    int perching_flag_ = 0;

private:
    void sensorCb(const std_msgs::Int8::ConstPtr& msg);
    void sensor1Cb(const std_msgs::Int8::ConstPtr& msg);

    void calPressure(int target_pressure, int sensor_index);
    void setAirPwm(uint8_t index, float pwm_value);
    void setAirPwm(const std::vector<uint8_t>& indices, const std::vector<float>& pwm_values);
    void publishAirPwmMerged();
    void publishAirPwm(const std::vector<uint8_t>& indices, const std::vector<float>& pwms);
    void failsafe();

    // void adjustAirPressure();
    void adjustPump();
    void startSVSwitch();
    void stopSVSwitch();
    void startSVExhaust();
    void stopSVExhaust();
    void stopAllSV();
    void startAllSV();
    void stopPump();


    ros::Subscriber sensor_joint_sub_;
    ros::Subscriber sensor_bottom_sub_;
    ros::Publisher pwm_air_pub_;
    ros::Publisher pwm_pub_;

    int prepare_finished_ = 0;


    int air_pressure_joint_ = 0;
    int air_pressure_bottom_ = 0;
    float output_ = 0.0;
    int bottom_approaching_pressure_ = 10; // 20
    int bottom_ready_pressure_ = 20; // 40
    int joint_flex_pressure_ = 15; // 15
    int joint_limit_pressure_ = 40; // 60
    int bottom_limit_pressure_ = 30; // 50
    int joint_max_pressure_ = 25; // 50
    int joint_perching_pressure_ = 25; // 45
    int bottom_perching_pressure_ = 10; // 20

    spinal::PwmTest pwm_air_cmd_;
    spinal::PwmTest last_published_pwm_;
    std::map<uint8_t,float> pwm_state_;
    int cnt_ = 0;
};

#endif // AIR_PRESSURE_CONTROLLER_H
