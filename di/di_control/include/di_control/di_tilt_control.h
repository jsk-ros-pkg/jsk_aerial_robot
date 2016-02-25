#include <ros/ros.h>
#include <jsk_stm/JskImu.h>

#include <std_msgs/UInt16.h>
#include <std_msgs/UInt8.h>

#include <std_msgs/Float64.h>

#include <dynamixel_msgs/JointState.h>
#include <dynamixel_controllers/TorqueEnable.h>
#include <geometry_msgs/Vector3.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <string>


typedef struct{
  ros::Subscriber servos_state_sub[2];
  std::string servos_state_sub_name[2];
  ros::Publisher servos_ctrl_pub[2];
  std::string servos_ctrl_pub_name[2];
  ros::ServiceClient servos_torque_enable_client[2];
  std::string servos_torque_enable_service_name[2];
  double rotate_angle;
  float current_angle[2];
  float target_angle[2];
  int angle_sgn[2];
  double angle_offset[2];
  double angle_max[2];
  double angle_min[2];
}TiltModule;



class TiltControl
{
 public:
  TiltControl(ros::NodeHandle nh, ros::NodeHandle nhp);
  ~TiltControl();

  static const uint8_t ACTIVE_TILT_MODE = 0x00;
  static const uint8_t PASSIVE_TILT_MODE = 0x01;

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::Subscriber attitude_sub_;
  ros::Subscriber desire_attitude_sub_;

  std::vector<TiltModule> tilt_modules_;
  ros::Timer  control_timer_;

  geometry_msgs::Vector3 current_attitude_;
  //geometry_msgs::Vector3 attitude_threshold_;
  geometry_msgs::Vector3 desire_attitude_;

  bool gimbal_debug_;

  bool tilt_command_flag_;
  int tilt_mode_;
  int tilt_module_num_;
  double tilt_thre_;
  double control_rate_;

  void tiltModulesInit();
  void controlFunc(const ros::TimerEvent & e);
  void servoCallback(const dynamixel_msgs::JointStateConstPtr& msg, int i, int j);

  void attitudeCallback(const jsk_stm::JskImuConstPtr& msg);
  void desireAttitudeCallback(const geometry_msgs::Vector3ConstPtr& msg);

  void gimbalControl();

};
