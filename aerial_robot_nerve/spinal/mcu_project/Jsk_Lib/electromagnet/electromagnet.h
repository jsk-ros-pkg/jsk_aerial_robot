/*
******************************************************************************
* File Name          : electromagnet.h
* Description        : electronic meganet device to pick up object in task3
******************************************************************************
*/

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#ifndef __ELECTRO_MAGNET_H
#define __ELECTRO_MAGNET_H

#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_can.h"
#include "can.h"

/* ros */
#include <ros.h>
#include <std_msgs/UInt8.h>

//can ID
#define CAN_HOST_ID 0x66
#define CAN_HOST_SUB_ID 0x6D
#define CAN_SLAVE_ID 0xDD
#define CAN_SLAVE_SUB_ID 0xD6
#define CAN_TX_BYTES 8
#define CAN_RE_BYTES 8
#define RX_INTERVAL 100 //100ms

class ElectroMagnet
{
public:
  ElectroMagnet();
  ~ElectroMagnet(){}

  void init(CAN_HandleTypeDef *hcan, ros::NodeHandle* nh);
  void update();

  ros::Publisher contact_status_pub_;
  std_msgs::UInt8 contact_status_msg_;

private:
  ros::NodeHandle* nh_;
  ros::Subscriber<std_msgs::UInt8, ElectroMagnet> control_sub_;

  CAN_HandleTypeDef *hcan_;
  CanTxMsgTypeDef can_tx_;
  CanRxMsgTypeDef can_re_;


  void controlCallback(const std_msgs::UInt8& control_msg);


};

extern ElectroMagnet electro_magnet_;
extern bool have_slave_;

#endif // __ELECTRO_MAGNET_H

