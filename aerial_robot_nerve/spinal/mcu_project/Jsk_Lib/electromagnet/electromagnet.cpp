/*
******************************************************************************
* File Name          : electromagnet.cpp
* Description        : electronic meganet device to pick up object in task3
******************************************************************************
*/

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#include "electromagnet/electromagnet.h"

//ElectroMagnet electro_magnet_;
bool have_slave_;

ElectroMagnet::ElectroMagnet():
  contact_status_pub_("contact_status", &contact_status_msg_),
  control_sub_("mag_control", &ElectroMagnet::controlCallback, this)
{
}

void ElectroMagnet::init(CAN_HandleTypeDef *hcan, ros::NodeHandle* nh)
{
  nh_ = nh;
  nh_->advertise(contact_status_pub_);
  nh_->subscribe< ros::Subscriber<std_msgs::UInt8, ElectroMagnet> >(control_sub_);

  /* CAN */
  have_slave_ = false;
  hcan_ = hcan;
  hcan_->pTxMsg = &can_tx_;
  hcan_->pRxMsg = &can_re_;
  hcan_->pTxMsg->StdId = CAN_SLAVE_ID;
  hcan_->pTxMsg->ExtId = CAN_SLAVE_SUB_ID;
  //default IDE and RTR type,
  hcan_->pTxMsg->DLC = CAN_TX_BYTES;   //transmit 8 bytes
  hcan_->pTxMsg->RTR = CAN_RTR_DATA;
  hcan_->pTxMsg->IDE = CAN_ID_STD;

  hcan_->pRxMsg->FIFONumber = CAN_FIFO1;
  CAN_FilterConfTypeDef sFilterConfig;
  sFilterConfig.FilterNumber = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO1;
  sFilterConfig.FilterActivation = ENABLE;
  HAL_CAN_ConfigFilter(hcan_, &sFilterConfig);
}

void ElectroMagnet::controlCallback(const std_msgs::UInt8& control_msg)
{
  if(!have_slave_) return;
  if(control_msg.data == 0) //release
    {
      for(int i = 0; i < 8; i ++)
        hcan_->pTxMsg->Data[i] = 0; 
      HAL_CAN_Transmit(hcan_, 100);  //100ms?
    }
  else if(control_msg.data == 1) //catch
    {
      for(int i = 0; i < 8; i ++)
        hcan_->pTxMsg->Data[i] = 1; 
      HAL_CAN_Transmit(hcan_, 100);
    }
}

void ElectroMagnet::update()
{
  static uint32_t last_time = HAL_GetTick();
  if(HAL_GetTick() - last_time > RX_INTERVAL)
    {
      last_time = HAL_GetTick();
      HAL_CAN_Receive_IT(hcan_,CAN_FIFO1);
    }

}
#if 0
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan)
{
  electro_magnet_.contact_status_msg_.data = 0;
  //receive the data from the SLAVE
  if(hcan->pRxMsg->StdId!=CAN_HOST_ID)
    {
      return;
    }
  for(int i = 1; i < 6; i++)
    electro_magnet_.contact_status_msg_.data += hcan->pRxMsg->Data[i];

  electro_magnet_.contact_status_msg_.data = ~(electro_magnet_.contact_status_msg_.data  & 248) - 7 ; //reverse(GPIO init input is opposite)

  electro_magnet_.contact_status_pub_.publish(&electro_magnet_.contact_status_msg_);
  //HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_2);

  //need to call    HAL_CAN_Receive_IT(&hcan,CAN_FIFO1) in main while
  if(!have_slave_) have_slave_ = true;
}

#endif
