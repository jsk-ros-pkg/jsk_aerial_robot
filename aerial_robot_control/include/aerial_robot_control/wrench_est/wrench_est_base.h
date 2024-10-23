//
// Created by li-jinjie on 24-10-23.
//

#ifndef AERIAL_ROBOT_CONTROL_WRENCH_EST_BASE_H
#define AERIAL_ROBOT_CONTROL_WRENCH_EST_BASE_H

namespace aerial_robot_control
{
class WrenchEstBase
{
public:
  virtual void initialize(double side_length) = 0;
  virtual double area() = 0;
  virtual ~WrenchEstBase(){}

protected:
  WrenchEstBase(){}
};
};  // namespace aerial_robot_control

#endif  // AERIAL_ROBOT_CONTROL_WRENCH_EST_BASE_H
