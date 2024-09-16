#ifndef __BASE_SERVO_H
#define __BASE_SERVO_H

#include <map>
#include <ros.h>

class BaseServo
{
public:
  virtual ~BaseServo() {}
  virtual void init(UART_HandleTypeDef* huart, ros::NodeHandle* nh) = 0;
  virtual bool available() = 0;
  virtual void setTargetPos(const std::map<uint16_t, float>& servo_map) = 0;
  virtual void update() = 0;
};

#endif
