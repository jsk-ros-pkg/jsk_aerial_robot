/*
******************************************************************************
* File Name          : kondo_servo.h
* Description        : kondo servo interface
******************************************************************************
*/

#ifndef __KONDO_SERVO_H
#define __KONDO_SERVO_H

#include "config.h"
#include <ros.h>
#include <sensor_msgs/JointState.h>
#include <map>

#define KONDO_SERVO_UPDATE_INTERVAL 2

#define MAX_SERVO_NUM 32
#define KONDO_SERVO_POSITION_MIN 3500
#define KONDO_SERVO_POSITION_MAX 11500
#define KONDO_SERVO_ANGLE_MIN -2.36
#define KONDO_SERVO_ANGLE_MAX 2.36
#define KONDO_BUFFER_SIZE 512
#define KONDO_POSITION_TX_SIZE 3
#define KONDO_POSITION_RX_SIZE 3

#define KONDO_SERVO_ANGLE_LIMIT_MIN -1.57  // -90 degree
#define KONDO_SERVO_ANGLE_LIMIT_MAX 1.57  // 90 degree

namespace
{
  #ifdef STM32H7
    uint8_t kondo_rx_buf_[KONDO_BUFFER_SIZE] __attribute__((section(".KondoRxBufferSection")));
  #else
    uint8_t kondo_rx_buf_[KONDO_BUFFER_SIZE];
  #endif
  uint32_t kondo_rd_ptr_ = 0;
}

class KondoServo
{
private:
  UART_HandleTypeDef* huart_;
  sensor_msgs::JointState joint_state_msg_;
  ros::NodeHandle* nh_;
  ros::Subscriber<sensor_msgs::JointState, KondoServo> kondo_servo_control_sub_;
  ros::Publisher joint_state_pub_;
  uint16_t target_position_[MAX_SERVO_NUM];
  uint16_t current_position_[MAX_SERVO_NUM];
  bool activated_[MAX_SERVO_NUM];
  uint16_t pos_rx_buf_[KONDO_POSITION_RX_SIZE];
  uint32_t servo_state_pub_last_time_;

  uint8_t id_telem_ = 0;
  char* servo_name_ = "1234";
  bool is_receive_data = false;

  void receiveSendOnce(int id, uint16_t target_position);

  int readOneByte();

  void registerPos();

  void servoControlCallback(const sensor_msgs::JointState& cmd_msg);

  void sendServoState();

  uint16_t rad2KondoPosConv(float angle);

  float kondoPos2RadConv(int pos);

  void inactivate(int id)
  {
    activated_[id] = false;
  }

  void activate(int id)
  {
    activated_[id] = true;
  }

public:
  ~KondoServo(){}
  KondoServo():
    kondo_servo_control_sub_("gimbals_ctrl", &KondoServo::servoControlCallback, this),
    joint_state_pub_("joint_states", &joint_state_msg_)
  {
  }

  void init(UART_HandleTypeDef* huart, ros::NodeHandle* nh);
  bool available();
  void setTargetPos(const std::map<uint16_t, float>& servo_map);
  void update();
};

#endif
