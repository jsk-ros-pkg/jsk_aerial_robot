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

#include <std_msgs/Float32MultiArray.h>

class GuidanceManualController {
public:
  GuidanceManualController()
  : nh_(),
  air_(nh_),
  hap_(nh_)
  
{
  pwm_pub_ = nh_.advertise<spinal::PwmTest>("/pwm_test", 1);
  joy_sub_ = nh_.subscribe("/quadrotor/joy", 1, &GuidanceManualController::joyCb, this);
  vad_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/vad", 1);
  arming_on_pub_ = nh_.advertise<std_msgs::Empty>("/quadrotor/teleop_command/start", 1);
  takeoff_pub_ = nh_.advertise<std_msgs::Empty>("/quadrotor/teleop_command/takeoff", 1);
  land_pub_ = nh_.advertise<std_msgs::Empty>("/quadrotor/teleop_command/land", 1);
  interaction_state_sub_ = nh_.subscribe<std_msgs::Int8>("/interaction/state", 1, &DeformationPlanning::interactionStateCallback, this);
  interaction_pub_ = nh_.advertise<std_msgs::Int8>("/interaction/state", 1);

  pressure_cmd_bottom_pub_ = nh_.advertise<std_msgs::Int8>("/air/target_bottom", 1);
  pressure_cmd_joint_pub_ = nh_.advertise<std_msgs::Int8>("/air/target_joint", 1);

  ros::NodeHandle pnh("~");
  pnh.param("norm_control_switch", haptics_norm_mode_switch_, 1);
  pnh.param("waypoint_reached_thresh", waypoint_reached_thresh_, 0.8);
  pnh.param("base_thrust", base_thrust_, 3.0);
  pnh.param("emotion_on", emotion_switch_, false);
  hap_.setBaseThrust(base_thrust_);
  hap_.setEmotionSwitch(emotion_switch_);
}

  // control_mode = -1 stop
  // 0 normal
  // 1 approach
  // 2 perching
  // 3 perhing finish
  // 4 deperch
void spin(){
  ros::Rate rate(100);
  while (ros::ok()){
    perching_state_ = air_.getPerchingState();
    perching_state_msg_.data = perching_state_;
    perching_state_pub_.publish(perching_state_msg_);
    ROS_INFO("Perch state: %d", perching_state_);

    ROS_INFO_STREAM("control:" << control_mode_);
    hap_.setNormModeSwitch(haptics_norm_mode_switch_);

    if (control_mode_ == -1 || air_.getAirPressureJoint() >= 60 || air_.getAirPressureBottom() >= 50){
      air_.stopAllPneumatics();
      hap_.stopAllMotors();
      publishMergedPwm();
    }else if (control_mode_ == 1){
      // 底面20kPa   
      msg_bottom_P.data = 20;
      pressure_cmd_bottom_pub_.publish(msg_bottom_P_);
    //   air_.initializePneumatics();
      // publishMergedPwm();
    }else if (control_mode_ == 2){
      //deformation中（何もしない）
    }else if (control_mode_ == 3){
      air_.keepPerching();
	    hap_.controlAuto();
      publishMergedPwm();
    }else if (control_mode_ == 4){
      //deperching
      deperching();
      publishMergedPwm();
    }else{
      air_.stopAllPneumatics();
      publishMergedPwm();
    }

    hap_.updateRviz();
    ros::spinOnce();
    rate.sleep();
  }
}

private:
  ros::NodeHandle nh_;
  AirPressureController air_;
  HapticsVisualizer hap_;
  ros::Publisher pwm_pub_;
  ros::Publisher vad_pub_;
  ros::Publisher arming_on_pub_;
  ros::Publisher takeoff_pub_;
  ros::Publisher land_pub_;
  ros::Publisher interaction_pub_;
  ros::Publisher pressure_cmd_bottom_pub_;
  ros::Publisher pressure_cmd_joint_pub_;
  ros::Subscriber joy_sub_;
  ros::Subscriber interaction_state_sub_;
  sensor_msgs::Joy joy_;

  std_msgs::Int8 msg_joint_P_, msg_bottom_P_;

  bool vibrate_mode_ = false;
  bool emotion_switch_ = false;
  int control_mode_ = 0;
  int haptics_norm_mode_switch_ = 0;
  double waypoint_reached_thresh_ = 0.8;
  double base_thrust_ = 3.0;
  void joyCb(const sensor_msgs::Joy::ConstPtr& msg){
    joy_ = *msg;
    hap_.setJoy(joy_);
    if (joy_.buttons[1] == 1){
      control_mode_ = -1;
      air_.stopAllPneumatics();
      hap_.stopAllMotors();
      ROS_INFO("Emergency stop");
    }else if (joy_.buttons[0] == 1) {
      control_mode_ = 1;
    }else if(joy_.buttons[2] == 1) {
      control_mode_ = 0;
    }else if(joy_.buttons[3] == 1) {
      control_mode_ = 2;
    }else if(joy_.buttons[4] == 1) {
      control_mode_ = 3;
    }else if(joy_.buttons[5] == 1) {
      control_mode_ = 0;
    }
  }

  void reachHumanCb(const std_msgs::Bool::ConstPtr& msg){
    reach_flag_ = msg -> data;
    if (reach_flag_) {
        air_.setPerchingState(1);
    }
  }

  void interactionStateCallback(const std_msgs::Int8::ConstPtr& msg)
  {
    interaction_state_ = msg->data;
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

  void deperching(){
    ROS_WARN("==============deperching==================");
    //ros::Duration(2.0).sleep();

    std_msgs::Empty e;
    ROS_WARN("Arming");
    //arming_on_pub_.publish(e);
    //ros::Duration(2.0).sleep();
    msg_bottom_P.data = 0;
    msg_joint_P.data = 0;
    pressure_cmd_bottom_pub_.publish(msg_bottom_P_);
    pressure_cmd_joint_pub_.publish(msg_joint_P_);
    air_.initializePneumatics();
    //face expression
    vad_array_.data = {-0.5f, -0.3f, -1.0f};
    vad_pub_.publish(vad_array_);

    if (!flight_state_flag_) {
        ros::Duration(1.0).sleep();
        air_.initializePneumatics();
        ROS_WARN("Takeoff");
        // takeoff_pub_.publish(e);
        ros::Duration(3.0).sleep();
        //face expression
        vad_array_.data = {0.0f, 0.0f, 0.0f};
        vad_pub_.publish(vad_array_);
        ros::Duration(9.0).sleep();
        land_pub_.publish(e);
    }
    air_.setPerchingState(0);
  }

};
int main(int argc, char** argv) {
  ros::init(argc, argv, "guidance_manual_controller");
  GuidanceManualController ctrl;
  ctrl.spin();
  return 0;
}
