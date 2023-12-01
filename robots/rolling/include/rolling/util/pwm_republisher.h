#include <ros/ros.h>
#include "sensor_msgs/JointState.h"
#include "spinal/DesireCoord.h"
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

  /* motor pwms */
  ros::Subscriber motor_pwm_sub_;
  std::vector<ros::Publisher> motor_pwm_pubs_;
  int motor_num_;

  /* desire coordinate */
  ros::Publisher desire_coordinate_roll_pub_, desire_coordinate_pitch_pub_;
  ros::Subscriber desire_coordinate_sub_;
  double desire_coordinate_roll_;
  double desire_coordinate_pitch_;

  /* gimbal angles */
  ros::Subscriber gimbals_control_sub_;
  std::vector<ros::Publisher> gimbal_control_pubs_;

  /* timer */
  ros::Timer timer_;

  void motorPwmCallback(const spinal::PwmsPtr & pwm_msg);
  void desireCoordinateCallback(const spinal::DesireCoordPtr & desire_coordinate_msg);
  void gimbalsControlCallback(const sensor_msgs::JointState & joint_state_msg);
  void timerCallback(const ros::TimerEvent & e);

};
