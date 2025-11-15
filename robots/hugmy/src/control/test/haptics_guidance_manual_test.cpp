#include <hugmy/control/air_pressure_controller.h>
#include <hugmy/control/haptics_controller.h>

#include <ros/ros.h>
#include <spinal/PwmTest.h>
#include <std_msgs/Empty.h>
#include <map>
#include <std_msgs/Int8.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>

class GuidanceManualController {
public:
  GuidanceManualController()
  : nh_(),
  air_(nh_),
  hap_(nh_)
{
  pwm_pub_ = nh_.advertise<spinal::PwmTest>("/pwm_test", 1);
  joy_sub_ = nh_.subscribe("/quadrotor/joy", 1, &GuidanceManualController::joyCb, this);
  ros::NodeHandle pnh("~");
  pnh.param("norm_control_switch", haptics_norm_mode_switch_, 0);
}

void spin(){
  ros::Rate rate(100);
  while (ros::ok()){
    ROS_INFO_STREAM("control:" << control_mode_);
    hap_.setNormModeSwitch(haptics_norm_mode_switch_);
    if (control_mode_ == 0 || air_.getAirPressureJoint() >= 60 || air_.getAirPressureBottom() >= 50){
      air_.stopAllPneumatics();
      hap_.stopAllMotors();
    } else {
      air_.keepPerching();
      if (vibrate_mode_) {
        ROS_INFO("Vibrate mode: output vibration pattern.");
        hap_.vibratePwms();
      } else {
        ROS_INFO("Manual haptics: controlManual.");
        vibrate_mode_ = false;
        air_.keepPerching();
        hap_.controlManual();
      }
    }
    publishMergedPwm();
    ros::spinOnce();
    rate.sleep();
  }
}

private:
  ros::NodeHandle nh_;
  AirPressureController air_;
  HapticsController hap_;
  ros::Publisher pwm_pub_;
  ros::Subscriber joy_sub_;
  sensor_msgs::Joy joy_;
  bool vibrate_mode_ = false;
  int control_mode_ = 1; // 0: STOP, 1: MANUAL, 2: AUTO
  int haptics_norm_mode_switch_ = 0;

  void joyCb(const sensor_msgs::Joy::ConstPtr& msg){
    joy_ = *msg;
    hap_.setJoy(joy_);
    if (joy_.buttons[1] == 1){
      control_mode_ = 0;
      air_.stopAllPneumatics();
      hap_.stopAllMotors();
      ROS_INFO("Emergency stop");
    }else if (joy_.buttons[0] == 1) {
      vibrate_mode_ = true;
      control_mode_ = 1;
    }else if(joy_.buttons[2] == 1) {
      vibrate_mode_ = false;
      control_mode_ = 1;
    }
  }


  // pwmをパーチング時以外で使うと飛べないので注意
  // keep_perching の所以外は力覚提示とマージする必要はない
  void publishMergedPwm() {
    spinal::PwmTest air_pwm_msg = air_.getAirPwm();
    spinal::PwmTest haptics_pwm_msg = hap_.getHapticsPwm();
    ROS_INFO("Manual haptics: merge");

    std::map<uint8_t, float> merged_pwm;
    for (size_t i = 0; i < haptics_pwm_msg.motor_index.size(); ++i) {
      merged_pwm[haptics_pwm_msg.motor_index[i]] = haptics_pwm_msg.pwms[i];
    }
    for (size_t i = 0; i < air_pwm_msg.motor_index.size(); ++i) {
      merged_pwm[air_pwm_msg.motor_index[i]] = air_pwm_msg.pwms[i];
    }
    spinal::PwmTest merged_msg;
    for (const auto& pair : merged_pwm) {
      merged_msg.motor_index.push_back(pair.first);
      merged_msg.pwms.push_back(pair.second);
    }
    pwm_pub_.publish(merged_msg);
  }

};
int main(int argc, char** argv) {
  ros::init(argc, argv, "guidance_manual_controller");
  GuidanceManualController ctrl;
  ctrl.spin();
  return 0;
}
