#include <ros/ros.h>
#include "spinal/Pwms.h"
#include "std_msgs/Float32.h"

class MotorPWMRepublisher
{
public:
  MotorPWMRepublisher(ros::NodeHandle nh, ros::NodeHandle nhp);
  ~MotorPWMRepublisher() {};

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;

  ros::Subscriber motor_pwm_sub_;
  std::vector<ros::Publisher> motor_pwm_pubs_;

  int motor_num_;

  void motorPwmCallback(const spinal::PwmsPtr & pwm_msg);
};
