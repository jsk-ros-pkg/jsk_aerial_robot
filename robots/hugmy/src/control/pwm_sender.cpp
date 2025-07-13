#include <hugmy/control/air_pressure_controller.h>
#include <hugmy/control/haptics_controller.h>
#include <hugmy/control/haptics_visualizer.h>

#include <ros/ros.h>
#include <spinal/PwmTest.h>
#include <std_msgs/Empty.h>
#include <map>
#include <sensor_msgs/Joy.h>

class IntegratedController {
public:
  IntegratedController()
  : nh_(),
  air_(nh_),
  haptics_(nh_),
  // visualizer_(nh_),
  control_mode_(0)
{
  pwm_pub_ = nh_.advertise<spinal::PwmTest>("/quadrotor/pwm_test", 1);
  joy_sub_ = nh_.subscribe("/quadrotor/joy", 1, &IntegratedController::joyCb, this);
  halt_pub_ = nh_.advertise<std_msgs::Empty>("/quadrotor/teleop_command/halt", 1);
}

  void spin() {
    ros::Rate rate(100);
    while (ros::ok()) {
      ROS_INFO("Mode: %d", control_mode_);
      if (control_mode_ == 0) {
        air_.stopAllPneumatics();
        haptics_.stopAllMotors();
        haptics_.pos_flag_ = true;
      } else {
        air_.update();
        if (control_mode_ == 1) {
          haptics_.controlManual();
          haptics_.pos_flag_ = true;
        } else if (control_mode_ == 2) {
          haptics_.controlAuto();
        }
      }
      haptics_.updateRviz();

      publishMergedPwm();

      ros::spinOnce();
      rate.sleep();
    }
  }

private:
  ros::NodeHandle nh_;
  AirPressureController air_;
  // HapticsController haptics_;
  HapticsVisualizer haptics_;
  ros::Publisher pwm_pub_;
  ros::Subscriber joy_sub_;
  ros::Publisher halt_pub_;
  int control_mode_; // 0: STOP, 1: MANUAL, 2: AUTO
  sensor_msgs::Joy joy_;

  void joyCb(const sensor_msgs::Joy::ConstPtr& msg) {
    joy_ = *msg;
    haptics_.setJoy(joy_);

    auto buttons = joy_.buttons;
    if (buttons[1] == 1) {
      control_mode_ = 0;
      ROS_INFO("Switched to STOP mode");
    } else if (buttons[2] == 1) {
      control_mode_ = 1;
      ROS_INFO("Switched to MANUAL mode");
    } else if (buttons[0] == 1) {
      control_mode_ = 2;
      ROS_INFO("Switched to AUTO mode");
    } else if (buttons[3] == 1) {
      air_.stopAllPneumatics();
      haptics_.stopAllMotors();
    //   halt_pub_.publish(std_msgs::Empty());
      ROS_INFO("Emergency stop");
    }
  }

  void publishMergedPwm() {
    spinal::PwmTest air_pwm_msg = air_.getAirPwm();
    spinal::PwmTest haptics_pwm_msg = haptics_.getHapticsPwm();

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
  ros::init(argc, argv, "integrated_controller");
  IntegratedController ctrl;
  ctrl.spin();
  return 0;
}
