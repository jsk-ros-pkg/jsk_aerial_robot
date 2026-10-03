/*
 * servo_manager.h
 *
 *  Created on: 2026/3/20
 *      Author: K.Sugihara
 */

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#ifndef __SERVO_MANAGER_H
#define __SERVO_MANAGER_H

#include "servo/servo.h"
#include "Spine/spine.h"

#include <ros.h>
#include <spinal/ServoControlCmd.h>
#include <spinal/ServoStates.h>
#include <spinal/ServoTorqueStates.h>
#include <spinal/ServoTorqueCmd.h>
#include <spinal/GetBoardInfo.h>
#include <spinal/JointProfiles.h>

#include <vector>

class ServoManager
{
public:
  ServoManager()
    : servo_state_pub_("servo/states", &servo_state_msg_)
    , servo_torque_state_pub_("servo/torque_states", &servo_torque_state_msg_)
    , servo_position_sub_("servo/target_states", &ServoManager::servoPositionCallback, this)
    , servo_current_sub_("servo/target_current", &ServoManager::servoCurrentCallback, this)
    , servo_torque_ctrl_sub_("servo/torque_enable", &ServoManager::servoTorqueControlCallback, this)
    , joint_profiles_sub_("joint_profiles", &ServoManager::jointProfilesCallback, this)
    , board_config_srv_("set_board_config", &ServoManager::boardConfigCallback, this)
    , board_info_srv_("get_board_info", &ServoManager::boardInfoCallback, this)
  {
    servo_num_ = 0;
    use_spine_ = false;
  }

  ~ServoManager(){};

  void init(ros::NodeHandle* nh);

  void addDirectServo(DirectServo* direct_servo);
  void addSpineServo();
  void update();

private:
  /* ROS */
  ros::NodeHandle* nh_;
  ros::Publisher servo_state_pub_;
  ros::Publisher servo_torque_state_pub_;
  ros::Subscriber<spinal::ServoControlCmd, ServoManager> servo_position_sub_;
  ros::Subscriber<spinal::ServoControlCmd, ServoManager> servo_current_sub_;
  ros::Subscriber<spinal::ServoTorqueCmd, ServoManager> servo_torque_ctrl_sub_;
  ros::Subscriber<spinal::JointProfiles, ServoManager> joint_profiles_sub_;

  ros::ServiceServer<spinal::SetBoardConfig::Request, spinal::SetBoardConfig::Response, ServoManager> board_config_srv_;
  ros::ServiceServer<spinal::GetBoardInfo::Request, spinal::GetBoardInfo::Response, ServoManager> board_info_srv_;

  spinal::ServoStates servo_state_msg_;
  spinal::ServoTorqueStates servo_torque_state_msg_;
  spinal::GetBoardInfo::Response board_info_res_;

  uint32_t servo_last_pub_time_;
  uint32_t servo_torque_last_pub_time_;

  std::vector<DirectServo*> servo_handlers_;
  std::map<uint8_t, DirectServo*> servo_index_to_servo_handler_;  // translate total servo index to each servo handler
                                                                  // pointer. if the pointer is NULL, it means the servo
                                                                  // is controlled by spine.
  std::map<uint8_t, uint8_t> servo_index_in_each_handle_;  // translate total servo index to each servo handler's index
  int servo_num_;
  bool use_spine_;

  void servoPublish();

  void servoPositionCallback(const spinal::ServoControlCmd& control_msg);
  void servoCurrentCallback(const spinal::ServoControlCmd& control_msg);
  void servoTorqueControlCallback(const spinal::ServoTorqueCmd& control_msg);
  void jointProfilesCallback(const spinal::JointProfiles& joint_prof_msg);
  void boardInfoCallback(const spinal::GetBoardInfo::Request& req, spinal::GetBoardInfo::Response& res);
  void boardConfigCallback(const spinal::SetBoardConfig::Request& req, spinal::SetBoardConfig::Response& res);
};

#endif /* __SERVO_MANAGER_H */
