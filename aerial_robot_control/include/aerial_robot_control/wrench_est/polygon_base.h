//
// Created by li-jinjie on 24-10-23.
//

#ifndef AERIAL_ROBOT_CONTROL_POLYGON_BASE_H
#define AERIAL_ROBOT_CONTROL_POLYGON_BASE_H

namespace polygon_base
{
class RegularPolygon
{
public:
  virtual void initialize(double side_length) = 0;
  virtual double area() = 0;
  virtual ~RegularPolygon(){}

protected:
  RegularPolygon(){}
};
};  // namespace polygon_base

#endif  // AERIAL_ROBOT_CONTROL_POLYGON_BASE_H
