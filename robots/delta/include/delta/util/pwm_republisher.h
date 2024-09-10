#include <ros/ros.h>
#include <delta/model/delta_robot_model.h>
#include "sensor_msgs/JointState.h"
#include "spinal/DesireCoord.h"
#include "spinal/Pwms.h"
#include "spinal/FourAxisCommand.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Quaternion.h"
#include <tf/tf.h>
#include "geometry_msgs/WrenchStamped.h"

class MotorPWMRepublisher
{
public:
  MotorPWMRepublisher(ros::NodeHandle nh, ros::NodeHandle nhp);
  ~MotorPWMRepublisher() {};

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;

  boost::shared_ptr<RollingRobotModel> delta_robot_model_;

  int motor_num_;
  double m_f_rate_;
  std::string tf_prefix_;

  /* motor pwms */
  ros::Subscriber motor_pwm_sub_;
  std::vector<ros::Publisher> motor_pwm_pubs_;

  /* desire coordinate */
  ros::Publisher desire_coordinate_roll_pub_, desire_coordinate_pitch_pub_;
  ros::Subscriber desire_coordinate_sub_;
  double desire_coordinate_roll_;
  double desire_coordinate_pitch_;

  /* gimbal angles */
  ros::Subscriber gimbals_control_sub_;
  std::vector<ros::Publisher> gimbal_control_pubs_;

  /* four axis command */
  ros::Subscriber four_axis_command_sub_;
  std::vector<ros::Publisher> thruster_wrench_pubs_;

  /* timer */
  ros::Timer timer_;

  void motorPwmCallback(const spinal::PwmsPtr & pwm_msg);
  void desireCoordinateCallback(const geometry_msgs::Quaternion & msg);
  void gimbalsControlCallback(const sensor_msgs::JointState & joint_state_msg);
  void fourAxisCommandCallback(const spinal::FourAxisCommand & msg);
  void timerCallback(const ros::TimerEvent & e);

};
