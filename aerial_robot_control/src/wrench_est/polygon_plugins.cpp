//
// Created by li-jinjie on 24-10-23.
//

#include <pluginlib/class_list_macros.h>
#include "aerial_robot_control/wrench_est/polygon_base.h"
#include "aerial_robot_control/wrench_est/polygon_plugins.h"

PLUGINLIB_EXPORT_CLASS(polygon_plugins::Triangle, polygon_base::RegularPolygon)
PLUGINLIB_EXPORT_CLASS(polygon_plugins::Square, polygon_base::RegularPolygon)
