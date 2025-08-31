#include <hugmy/control/air_pressure_controller.h>
#include <hugmy/control/haptics_controller.h>
#include <hugmy/control/haptics_visualizer.h>

#include <ros/ros.h>
#include <spinal/PwmTest.h>
#include <std_msgs/Empty.h>
#include <map>
#include <std_msgs/Int8.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Bool.h>

class IntegratedController {
public:
  IntegratedController()
  : nh_(),
  air_(nh_),
  haptics_(nh_),
  // visualizer_(nh_),
  control_mode_(1)
{
  pwm_pub_ = nh_.advertise<spinal::PwmTest>("/quadrotor/pwm_test", 1);
  joy_sub_ = nh_.subscribe("/quadrotor/joy", 1, &IntegratedController::joyCb, this);
  arming_on_pub_ = nh_.advertise<std_msgs::Empty>("/quadrotor/teleop_command/start", 1);
  halt_pub_ = nh_.advertise<std_msgs::Empty>("/quadrotor/teleop_command/halt", 1);
  takeoff_pub_ = nh_.advertise<std_msgs::Empty>("/quadrotor/teleop_command/takeoff", 1);
  land_pub_ = nh_.advertise<std_msgs::Empty>("/quadrotor/teleop_command/land", 1);
  perching_state_pub_ = nh_.advertise<std_msgs::UInt8>("/perching_state", 1);
  flight_state_sub_ = nh_.subscribe<std_msgs::UInt8>("/quadrotor/flight_state", 1, &IntegratedController::flightStateCb, this);
  reach_to_human_sub_ = nh_.subscribe<std_msgs::Bool>("/reach_flag", 1, &IntegratedController::reachHumanCb, this);
}

void spin() {
    ros::Rate rate(100);
    while (ros::ok()) {
        perching_state_ = air_.getPerchingState();
        perching_state_msg_.data = perching_state_;
        perching_state_pub_.publish(perching_state_msg_);
        ROS_INFO("Perch flag: %d", perching_state_);
        ROS_INFO("Joint Pressure: %d | Bottom Pressure: %d", air_.getAirPressureJoint(), air_.getAirPressureBottom());
        if (perching_state_ == 1) {
            air_.readyPerching();
        } else if (perching_state_ == 2) {
            air_.startPerching();
            // if (air_.getPrepareFinished() == 1 && flight_state_flag_) {
            if (air_.getPrepareFinished() == 1) {
                ROS_WARN("halt");
                std_msgs::Empty e;
                halt_pub_.publish(e);
                air_.maxWorkPumptoJoint();
                air_.setPrepareFinished(2);
                flight_state_flag_ = false;
            }
        } else if (perching_state_ == 3) {
            if (control_mode_ == 0) {
                air_.stopAllPneumatics();
                haptics_.stopAllMotors();
                haptics_.pos_flag_ = true;
            } else {
                air_.keepPerching();
                haptics_.controlAuto();
                if (haptics_.getHapticsFinished()){
                    air_.setPerchingState(4);
                    ros::Duration(1.0).sleep();
                    ROS_INFO("deperch ready");
                }
                // if (control_mode_ == 1) {
                //     haptics_.controlManual();
                //     haptics_.pos_flag_ = true;
                // } else if (control_mode_ == 2) {
                //     haptics_.controlAuto();
                // }
            publishMergedPwm();
            }
        } else if (perching_state_ == 4) {
            deperching();
        } else if (perching_state_ == 5 || air_.getAirPressureJoint() >= 60 || air_.getAirPressureBottom() >= 50) {
            air_.initializePneumatics();
            haptics_.stopAllMotors();
        } else {
            air_.bottomPressurePrepare();
        }
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

  ros::Subscriber flight_state_sub_;
  ros::Subscriber reach_to_human_sub_;
  ros::Publisher arming_on_pub_;
  ros::Publisher takeoff_pub_;
  ros::Publisher land_pub_;
  ros::Publisher perching_state_pub_;
  std_msgs::UInt8 flight_state_msg_{};
  std_msgs::UInt8 perching_state_msg_{};
  bool flight_state_flag_ = false; // true when data==5(arming)
  int perching_state_ = 0;
  int halt_flag_ = 0;
  int control_mode_; // 0: STOP, 1: MANUAL, 2: AUTO
  bool reach_flag_;
  int deperching_cnt = 0;
  sensor_msgs::Joy joy_;

  void joyCb(const sensor_msgs::Joy::ConstPtr& msg) {
    joy_ = *msg;
    haptics_.setJoy(joy_);

    auto buttons = joy_.buttons;
    if (buttons[4] == 1) { // left up
        air_.setPerchingState(1);
    } else if (buttons[7] == 1) { // right down
        air_.setPerchingState(2);
    } else if (buttons[5] == 1) { // right up
        air_.setPerchingState(3);
    } else if (buttons[6] == 1) { // left down
        air_.setPerchingState(4);
    } else if (buttons[1] == 1) {
        // control_mode_ = 0;
        ROS_INFO("Switched to STOP mode");
    } else if (buttons[2] == 1) {
        // control_mode_ = 1;
        ROS_INFO("Switched to MANUAL mode");
    } else if (buttons[0] == 1) {
        // control_mode_ = 2;
        ROS_INFO("Switched to AUTO mode");
    } else if (buttons[3] == 1) {
        control_mode_ = 0;
        air_.stopAllPneumatics();
        haptics_.stopAllMotors();
        air_.setPerchingState(5);
        //   halt_pub_.publish(std_msgs::Empty());
        ROS_INFO("Emergency stop");
    }
  }

  void flightStateCb(const std_msgs::UInt8::ConstPtr& msg){
    flight_state_msg_ = *msg;
    //flight_state_flag_ = (flight_state_msg_.data == 5); //arming
  }
    
  void reachHumanCb(const std_msgs::Bool::ConstPtr& msg){
    reach_flag_ = msg -> data;
    if (reach_flag_) {
        air_.setPerchingState(1);
    }
  }

  //pwmをパーチング時以外で使うと飛べないので注意
  //keep_perchingの所以外は力覚提示とマージする必要はない
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

  void deperching(){
    ROS_WARN("==============deperching==================");
    // ros::Duration(2.0).sleep();
    deperching_cnt++;
    if (deperching_cnt >= 30){
      std_msgs::Empty e;
      ROS_WARN("Arming");
      arming_on_pub_.publish(e);
      ros::Duration(2.0).sleep();
      air_.initializePneumatics();

      if (!flight_state_flag_) {
        ros::Duration(1.0).sleep();
        air_.initializePneumatics();
        ROS_WARN("Takeoff");
        takeoff_pub_.publish(e);
        ros::Duration(12.0).sleep();
        land_pub_.publish(e);
      }
      air_.setPerchingState(0);
    }
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "integrated_controller");
  IntegratedController ctrl;
  ctrl.spin();
  return 0;
}
