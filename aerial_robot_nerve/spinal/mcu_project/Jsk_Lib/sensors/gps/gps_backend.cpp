#include "sensors/gps/gps_backend.h"
#include "it.h"

namespace
{
  uint8_t gps_rx_buf_[GPS_RX_SIZE];
}

void GPS_Backend::init(UART_HandleTypeDef* huart, ros::NodeHandle* nh)
{
  huart_ =huart;
  nh_ = nh;
  nh_->subscribe< ros::Subscriber<std_msgs::UInt8, GPS_Backend> >(gps_config_sub_);

}

void GPS_Backend::startReceive()
{
  HAL_UART_Receive_IT(huart_, gps_rx_buf_, GPS_RX_SIZE);
}

uint8_t* GPS_Backend::getRxBuf() {return gps_rx_buf_;}

void GPS_ErrorCpltCallback(UART_HandleTypeDef *huart)
{
  huart->ErrorCode = HAL_UART_ERROR_NONE;
  HAL_UART_Receive_IT(huart, gps_rx_buf_, GPS_RX_SIZE);
}

/* CAUTION: void GPS_RxCpltCallback(UART_HandleTypeDef *huart) is def */
