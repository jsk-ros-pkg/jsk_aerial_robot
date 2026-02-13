/*
******************************************************************************
* File Name          : kondo_servo.h
* Description        : kondo servo interface
******************************************************************************
*/

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#ifndef __KONDO_SERVO_H
#define __KONDO_SERVO_H

#include "config.h"
#include <ros.h>
#include <spinal/ServoControlCmd.h>
#include <spinal/ServoState.h>
#include <map>

#include "servo/drivers/servo_base_driver.h"

#define KONDO_SERVO_UPDATE_INTERVAL 10
#define KONDO_SERVO_TIMEOUT 1
#define KONDO_SERVO_POSITION_MIN 3500
#define KONDO_SERVO_POSITION_MAX 11500
#define KONDO_SERVO_ANGLE_MIN -2.36
#define KONDO_SERVO_ANGLE_MAX 2.36
#define SERVO_STATE_PUB_INTERVAL 25 //40Hz
#define KONDO_BUFFER_SIZE 512
#define KONDO_POSITION_RX_SIZE 3

class KondoServo : public ServoBase
{
private:
  ros::NodeHandle* nh_;
  uint16_t current_position_[MAX_SERVO_NUM];
  bool activated_[MAX_SERVO_NUM];
  uint16_t pos_rx_buf_[KONDO_POSITION_RX_SIZE];
  uint32_t dma_write_ptr_ ;
  uint32_t pos_rx_ptr_ ;
  uint32_t servo_state_read_last_time_;
  uint32_t read_servo_index_;

  inline void setStatusReturnLevel() override;
  inline void getHomingOffset() override;
  inline void getCurrentLimit() override;
  inline void getPositionGains() override;
  inline void getProfileVelocity() override;

public:
  KondoServo(){}
  void init(UART_HandleTypeDef* huart, osMutexId* mutex = NULL) override;
  void pinReconfig() override;
  void ping() override;
  HAL_StatusTypeDef read(uint8_t* data,  uint32_t timeout) override;
  void reboot(uint8_t servo_index) override;
  void setTorque(uint8_t servo_index) override;
  void setTorqueFromPresetnPos(uint8_t servo_index) override;
  void setHomingOffset(uint8_t servo_index) override;
  void setRoundOffset(uint8_t servo_index, int32_t ref_value) override;
  void setPositionGains(uint8_t servo_index) override;
  void setProfileVelocity(uint8_t servo_index) override;
  void setCurrentLimit(uint8_t servo_index) override;
  void update() override;

  void writePosCmd(int id, uint16_t target_position);
  void registerPos();  
  bool available();
  void sendServoState();
  uint16_t rad2KondoPosConv(float angle);
  float kondoPos2RadConv(int pos);
  void inactivate(int id);
  void activate(int id);
};

#endif
