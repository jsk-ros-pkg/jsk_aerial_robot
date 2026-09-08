/**
******************************************************************************
* File Name          : servo.h
* Description        : universal servo control interface for Spinal
* Author             : J.Sugihara (2024/3/1)
includes ------------------------------------------------------------------*/


#ifndef APPLICATION_SERVO_TEMP_SERVO_H_
#define APPLICATION_SERVO_TEMP_SERVO_H_

#include "drivers/Dynamixel/dynamixel_serial.h"
#include "drivers/kondo_servo/kondo_servo.h"
#include <ros.h>
#include <spinal/ServoControlCmd.h>
#include <spinal/ServoStates.h>
#include <spinal/ServoCommStatus.h>
#include <spinal/ServoTorqueStates.h>
#include <spinal/ServoTorqueCmd.h>
#include <spinal/SetDirectServoConfig.h>
#include <spinal/GetBoardInfo.h>
#include <spinal/JointProfiles.h>
#include <string.h>
#include <config.h>
#include <map>
#include "flashmemory/flashmemory.h"

class Initializer;

namespace ValueType
{
  enum
    {BIT = 0, RADIAN = 1};
};

class DirectServo
{
public:
  DirectServo():
    joint_profiles_sub_("joint_profiles", &DirectServo::jointProfilesCallback,this),
    servo_comm_status_pub_("servo/comm_status", &servo_comm_status_msg_)
  {
    connected_ = false;
  }
  ~DirectServo(){}

  bool init(UART_HandleTypeDef* huart, ros::NodeHandle* nh, osMutexId* mutex);
  bool createRTOSObjects();
  void update();
  bool connected() {return connected_;}
  unsigned int getServoNum() const {return servo_handler_.getServoNum();}
  const ServoData& getServoData(uint8_t index) const {
    return servo_handler_.getServo()[index];
  }
  uint16_t getServoFeedbackFrequency() const {
    return servo_handler_.getServoFeedbackFrequency();
  }
  bool hasFreshState() const {return servo_handler_.getROSCommFlag();}
  void markStatePublished() {servo_handler_.setROSCommFlag(false);}
  bool requestConfig(uint8_t command, const int32_t* data,
                     uint8_t data_length);
  bool requestConfigByIndex(uint8_t servo_index, uint8_t command,
                            const int32_t* data, uint8_t data_length);
  bool setGoalTargets(uint8_t index,
                      bool has_position, int32_t position,
                      bool has_velocity, int32_t velocity,
                      bool has_current, int16_t current);
  bool setTorqueEnable(uint8_t index, bool enable);
  void fillBoardInfo(spinal::BoardInfo& board) const;
  void torqueEnable(const std::map<uint8_t, float>& servo_map);
  void setGoalAngle(const std::map<uint8_t, float>& servo_map, uint8_t value_type = 0);
#if KONDO
  KondoServo& getServoHnadler() {return servo_handler_;}
#else DYNAMIXEL
  DynamixelSerial& getServoHnadler() {return servo_handler_;}  
#endif

  uint32_t rad2Pos(float angle, float scale, uint32_t zero_point_pos){
    return static_cast<uint32_t>(angle /scale + zero_point_pos);
  }

private:
  /* ROS */
  ros::NodeHandle* nh_;
  ros::Subscriber<spinal::JointProfiles, DirectServo> joint_profiles_sub_;
  ros::Publisher servo_comm_status_pub_;
  spinal::ServoCommStatus servo_comm_status_msg_;

  uint32_t servo_comm_status_last_pub_time_;
  bool servo_comm_status_initialized_;
  bool last_bus_map_valid_;

  void jointProfilesCallback(const spinal::JointProfiles& joint_prof_msg);
  void publishCommStatus();
  
  /* Servo state */
  struct ServoState{
    int16_t angle;
    uint8_t temperature;
    uint8_t moving;
    int16_t current;
    uint8_t error;
    ServoState(uint16_t angle, uint8_t temperature, uint8_t moving, int16_t current, uint8_t error)
      :angle(angle), temperature(temperature), moving(moving), current(current), error(error){}
  };

  struct JointProf{
    uint8_t servo_id;
    int8_t angle_sgn;
    float angle_scale;
    int16_t zero_point_offset;
  };

  JointProf joint_profiles_[MAX_SERVO_NUM];

#if KONDO
  KondoServo servo_handler_;
#else DYNAMIXEL
  DynamixelSerial servo_handler_;
#endif

  bool connected_;

  friend class Initializer;
};


#endif /* APPLICATION_SERVO_SERVO_H_ */
