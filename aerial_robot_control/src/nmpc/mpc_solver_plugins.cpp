//
// Created by li-jinjie on 24-10-27.
//

#include <pluginlib/class_list_macros.h>

#include "aerial_robot_control/nmpc/base_mpc_solver.h"

// fixed quadrotor
#include "aerial_robot_control/nmpc/fix_qd_thrust_out_mdl/nmpc_solver.h"

// tilt quadrotor
#include "aerial_robot_control/nmpc/tilt_qd_servo_dist_mdl/nmpc_solver.h"

PLUGINLIB_EXPORT_CLASS(aerial_robot_control::mpc_solver::FixQdMdlMPCSolver,
                       aerial_robot_control::mpc_solver::BaseMPCSolver)

PLUGINLIB_EXPORT_CLASS(aerial_robot_control::mpc_solver::TiltQdServoDistMdlMPCSolver,
                       aerial_robot_control::mpc_solver::BaseMPCSolver)
