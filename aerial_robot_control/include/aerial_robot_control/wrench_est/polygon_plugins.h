//
// Created by li-jinjie on 24-10-23.
//

#ifndef AERIAL_ROBOT_CONTROL_POLYGON_PLUGINS_H
#define AERIAL_ROBOT_CONTROL_POLYGON_PLUGINS_H

#include "aerial_robot_control/wrench_est/polygon_base.h"
#include <cmath>

namespace polygon_plugins
{
class Triangle : public polygon_base::RegularPolygon
{
public:
  Triangle(){}

  void initialize(double side_length)
  {
    side_length_ = side_length;
  }

  double area()
  {
    return 0.5 * side_length_ * getHeight();
  }

  double getHeight()
  {
    return sqrt((side_length_ * side_length_) - ((side_length_ / 2) * (side_length_ / 2)));
  }

private:
  double side_length_;
};

class Square : public polygon_base::RegularPolygon
{
public:
  Square(){}

  void initialize(double side_length)
  {
    side_length_ = side_length;
  }

  double area()
  {
    return side_length_ * side_length_;
  }

private:
  double side_length_;

};
};

#endif  // AERIAL_ROBOT_CONTROL_POLYGON_PLUGINS_H
