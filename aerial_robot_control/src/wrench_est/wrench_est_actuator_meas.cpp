//
// Created by li-jinjie on 24-10-25.
//

#include "aerial_robot_control/wrench_est/wrench_est_momentum.h"
#include "aerial_robot_control/wrench_est/wrench_est_mom_acc.h"
#include "aerial_robot_control/wrench_est/wrench_est_acceleration.h"



#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::WrenchEstMomentum, aerial_robot_control::WrenchEstBase)
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::WrenchEstMomAcc, aerial_robot_control::WrenchEstBase)
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::WrenchEstAcceleration, aerial_robot_control::WrenchEstBase)