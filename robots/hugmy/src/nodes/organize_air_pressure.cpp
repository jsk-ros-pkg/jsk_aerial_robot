#include <ros/ros.h>
#include <hugmy/control/air_pressure_controller.h>
#include <sensor_msgs/Joy.h>

sensor_msgs::Joy latest_joy;
bool joy_received = false;
bool emergency_sent = false;

void joyCb(const sensor_msgs::Joy::ConstPtr& msg){
  // ROS_INFO_STREAM("Joy callback received. buttons size=" << msg->buttons.size()
  //                 << " axes size=" << msg->axes.size());

  // if (!msg->buttons.empty()) {
  //   std::stringstream ss;
  //   ss << "Buttons: ";
  //   for (size_t i = 0; i < msg->buttons.size(); ++i) {
  //     ss << msg->buttons[i] << " ";
  //   }
  //   ROS_INFO_STREAM(ss.str());
  // }
  latest_joy = *msg;
  joy_received = true;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "air_pressure_controller");
  ros::NodeHandle nh;

  AirPressureController ctrl(nh);
  ros::Subscriber joy_sub = nh.subscribe<sensor_msgs::Joy>("/quadrotor/joy", 1, joyCb);

  ros::Rate rate(50);
  while (ros::ok()){
    ros::spinOnce();

    // if (joy_received && !emergency_sent){
    //   if (!latest_joy.buttons.empty() && latest_joy.buttons[5] == 1){
    //     ROS_ERROR("[Main] Emergency button pressed. Engaging AirPressureController emergency stop.");
    //     ctrl.engageEmergencyStop();
    //     emergency_sent = true;
    //   }
    // }

    // if (ctrl.isEmergencyStop()){
    // }
    
    rate.sleep();
  }

  return 0;
}
