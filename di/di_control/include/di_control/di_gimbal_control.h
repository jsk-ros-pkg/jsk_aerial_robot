#include <ros/ros.h>
#include <jsk_stm/JskImu.h>

#include <std_msgs/UInt16.h>
#include <std_msgs/UInt8.h>

#include <std_msgs/Float64.h>

#include <dynamixel_msgs/JointState.h>
#include <dynamixel_controllers/TorqueEnable.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <aerial_robot_base/FlightNav.h>
#include <aerial_robot_base/FourAxisPid.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <string>

//* for dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <di_control/GimbalDynReconfConfig.h>

//ring buffer
#include <iostream>
#include <deque>

//mutex
#include <boost/thread/mutex.hpp>
#if BOOST_VERSION>105200
#include <boost/thread/lock_guard.hpp>
#endif


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
}GimbalModule;



class GimbalControl
{
 public:
  GimbalControl(ros::NodeHandle nh, ros::NodeHandle nhp);
  ~GimbalControl();

  static const uint8_t ACTIVE_GIMBAL_MODE = 0x01;
  static const uint8_t PASSIVE_GIMBAL_MODE = 0x02;

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::Publisher desire_tilt_pub_;
  ros::Subscriber attitude_sub_;
  ros::Subscriber desire_attitude_sub_;
  ros::Subscriber attitude_command_sub_;
  ros::Publisher alt_control_pub_;

//debug for passive att compare method
  ros::Publisher att_diff_pub_;

  std::vector<GimbalModule> gimbal_modules_;
  ros::Timer  control_timer_;

  geometry_msgs::Vector3 current_attitude_;
  //geometry_msgs::Vector3 attitude_threshold_;
  geometry_msgs::Vector3 desire_attitude_;
  geometry_msgs::Vector3 final_attitude_;

  std::deque<geometry_msgs::Vector3> att_command_qu_;
  boost::mutex queue_mutex_;

  bool gimbal_debug_;
  bool gimbal_simulation_;

  bool gimbal_command_flag_;
  int gimbal_mode_;
  int gimbal_module_num_;
  double gimbal_thre_;
  double control_rate_;

  double body_diameter_;
  double active_gimbal_tilt_interval_;
  double active_gimbal_tilt_duration_;

  //passive 
  double passive_level_back_duration_;
  bool passive_tilt_mode_;
  double att_comp_duration_size_;
  double att_control_rate_;
  double att_comp_duration_;
  ros::Time att_comp_time_;

  float roll_diff_;
  float pitch_diff_;
  float roll_delay_;
  float pitch_delay_;
  
  double attitude_outlier_thre_;


  void gimbalModulesInit();
  void controlFunc(const ros::TimerEvent & e);
  void servoCallback(const dynamixel_msgs::JointStateConstPtr& msg, int i, int j);

  void attitudeCallback(const jsk_stm::JskImuConstPtr& msg);
  void desireAttitudeCallback(const geometry_msgs::Vector3ConstPtr& msg);
void attCommandCallback(const aerial_robot_base::FourAxisPidConstPtr& cmd_msg);


  void gimbalControl();

  bool attCommandCompare();

  //cfg
  dynamic_reconfigure::Server<di_control::GimbalDynReconfConfig>* gimbal_server_;
  dynamic_reconfigure::Server<di_control::GimbalDynReconfConfig>::CallbackType dyn_reconf_func_;

void GimbalDynReconfCallback(di_control::GimbalDynReconfConfig &config, uint32_t level);
  

};
