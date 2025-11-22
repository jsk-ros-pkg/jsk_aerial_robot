#ifndef AIR_PRESSURE_CONTROLLER_H
#define AIR_PRESSURE_CONTROLLER_H

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>
#include <spinal/PwmTest.h>
#include <algorithm>
#include <vector>
#include <cstdint>
#include <map>
#include <atomic>


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

    bool test_mode_ = false;
    bool external_mode_ = false;
    int calib_sensor_index_ = 0;

    std::atomic<bool> emergency_stop_{false};


    void engageEmergencyStop() {
     if (!emergency_stop_) {
      emergency_stop_ = true;
      ROS_ERROR("[Air] EMERGENCY STOP engaged. All pneumatics will be stopped.");
     }
    }
    void clearEmergencyStop() {
      emergency_stop_ = false;
      ROS_WARN("[Air] Emergency stop cleared.");
    }
    bool isEmergencyStop() const { return emergency_stop_.load(); }

private:
    void sensorCb(const std_msgs::Int8::ConstPtr& msg);
    void sensor1Cb(const std_msgs::Int8::ConstPtr& msg);

    void calPressure(int target_pressure, int sensor_index);
    void setAirPwm(uint8_t index, float pwm_value);
    void setAirPwm(const std::vector<uint8_t>& indices, const std::vector<float>& pwm_values);
    void publishAirPwmMerged();
    void publishAirPwm(const std::vector<uint8_t>& indices, const std::vector<float>& pwms, bool to_air_bus);
    void failsafe();

    void startLeakCalibration(int sensor_index);
    double estimateLeak(const std::vector<double>& t, const std::vector<double>& pg);
    void leakTimerCb(const ros::TimerEvent& ev);

    void switchSingleMode(const std::vector<uint8_t>& induces, const std::vector<float>& pwm_values);
    void switchSingleMode(uint8_t induce, float pwm_value);

    // void adjustAirPressure();
    void inicialPump();
    void adjustPump();
    void startSVSwitch();
    void stopSVSwitch();
    void startSVExhaust();
    void stopSVExhaust();
    void stopAllSV();
    void startAllSV();
    void stopPump();

    double pid(double target, double measured, int sensor_index);
    static inline double clamp (double v, double low, double hi){
      return std::max(low, std::min(hi, v));
    }
    static inline double clamp_int (int v, int low, int hi){
      return std::max(low, std::min(hi, v));
    }

    void controlLoopCb(const ros::TimerEvent& e);
    void targetJointCb(const std_msgs::Int8::ConstPtr& msg);
    void targetBottomCb(const std_msgs::Int8::ConstPtr& msg);


    ros::Subscriber sensor_joint_sub_;
    ros::Subscriber sensor_bottom_sub_;
    ros::Publisher pwm_air_pub_;
    ros::Publisher pwm_pub_;
    ros::Subscriber target_joint_sub_, target_bottom_sub_;
    ros::Publisher joint_filtered_pub_;

    int prepare_finished_ = 0;


    int air_pressure_joint_ = 0;
    int air_pressure_bottom_ = 0;
    float output_ = 0.0;
    int bottom_approaching_pressure_ = 10; // 20
    int bottom_ready_pressure_ = 20; // 40
    int joint_flex_pressure_ = 15; // 15
    int joint_limit_pressure_ = 60; // 60
    int bottom_limit_pressure_ = 50; // 50
    int joint_max_pressure_ = 25; // 50
    int joint_perching_pressure_ = 45; // 45
    int bottom_perching_pressure_ = 20; // 20

    spinal::PwmTest pwm_air_cmd_;
    spinal::PwmTest last_published_pwm_;
    std::map<uint8_t,float> pwm_state_;
    int cnt_ = 0;

    double kp_joint_ = 0.02;
    double ki_joint_ = 0.10;
    double kd_joint_ = 0.00;
    double kp_bottom_ = 0.02;
    double ki_bottom_ = 0.10;
    double kd_bottom_ = 0.00;
    double u_limit_ = 0.8;

    bool leak_calib_enable_ = true;
    bool leak_calib_finished_ = false;

    int calib_target_pressure_ = 30;
    double calib_duration_ = 8.0;
    double calib_min_pg_ = 5.0;
    bool enable_leak_ff_ = true;
    double k_ff_ = 1.0;

    bool enable_gain_sched_ = true;
    int p_max_  = 50;
    double alpha_sched_ = 0.3;
    double  kp_leak_scale_ = 0.01;
    double  ki_leak_scale_ = 0.005;

    bool leak_calib_running_ = false;
    std::vector<double> t_log_;
    std::vector<double> pg_log_;
    int sensor_data_ = 0;
    ros::Timer leak_timer_;
    ros::Time calib_start_;
    double g_leak_bottom_ = 0.0;
    double g_leak_joint_ = 0.0;

    bool leak_pressurizing_ = true;
    ros::Timer press_timer_;
    double press_check_period_ = 0.05;
    double press_deadband_kpa = 1.0;
    double press_timeout_sec_ = 20.0;
    ros::Time press_start_;
    void pressTimerCb(const ros::TimerEvent&);
  

    struct PIDState {
      double I = 0.0;
      double e_prev = 0.0;
      ros::Time t_prev; bool
      inited=false;
    };
    PIDState pid_joint_, pid_bottom_;


    int target_joint_cmd_ = 0;
    int target_bottom_cmd_ = 0;
    bool has_target_joint_ = false;
    bool has_target_bottom_ = false;


    double control_rate_hz_ = 50.0;
    ros::Timer control_timer_;

    double lpf_tau_ = 0.05;

    double joint_filt_ = 0.0;
    ros::Time joint_last_;
    ros::Time bottom_last_;
    bool joint_filt_inited_ = false;

};

#endif // AIR_PRESSURE_CONTROLLER_H
