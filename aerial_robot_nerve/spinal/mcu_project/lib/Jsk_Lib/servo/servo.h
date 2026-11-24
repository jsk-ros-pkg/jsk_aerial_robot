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
  struct JointProf{
    uint8_t servo_id;
    int8_t angle_sgn;
    float angle_scale;
    int16_t zero_point_offset;
  };

  DirectServo()
  {
    connected_ = false;
  }
  ~DirectServo(){}

  bool init(UART_HandleTypeDef* huart, ros::NodeHandle* nh, osMutexId* mutex);
  void update();
  bool connected() {return connected_;}
  void sendData(bool flag_send_asap);
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

  JointProf joint_profiles_[MAX_SERVO_NUM];

private:
  /* ROS */
  ros::NodeHandle* nh_;
  
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

#if KONDO
  KondoServo servo_handler_;
#else DYNAMIXEL
  DynamixelSerial servo_handler_;
#endif

  bool connected_;

  friend class Initializer;
};


#endif /* APPLICATION_SERVO_SERVO_H_ */
