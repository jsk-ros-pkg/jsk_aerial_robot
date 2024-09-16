#ifndef __BASE_SERVO_H
#define __BASE_SERVO_H

#include <map>
#include <ros.h>

#define SERVO_UPDATE_INTERVAL 2  // ms for one round. In my case, the MCU controls one servo at one round.
// So if 4 servos are connected, the MCU controls 4 servos in 8ms, i.e. 125hz.
#define MAX_SERVO_NUM 32

#define SERVO_BUFFER_SIZE 512
namespace
{
#ifdef STM32H7
uint8_t servo_rx_buf_[SERVO_BUFFER_SIZE] __attribute__((section(".KondoRxBufferSection")));
#else
uint8_t servo_rx_buf_[SERVO_BUFFER_SIZE];
#endif
uint32_t servo_rd_ptr_ = 0;
}  // namespace

class BaseServo
{
public:
  virtual ~BaseServo()
  {
  }
  virtual void init(UART_HandleTypeDef* huart, ros::NodeHandle* nh) = 0;
  virtual bool available() = 0;
  virtual void setTargetPos(const std::map<uint16_t, float>& servo_map) = 0;
  virtual void update() = 0;
};

#endif
