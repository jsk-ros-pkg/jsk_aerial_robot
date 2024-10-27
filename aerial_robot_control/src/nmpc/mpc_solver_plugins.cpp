//
// Created by li-jinjie on 24-10-27.
//

#include <pluginlib/class_list_macros.h>

#include "aerial_robot_control/nmpc/base_mpc_solver.h"

// fixed quadrotor
#include "aerial_robot_control/nmpc/fix_qd_thrust_out_mdl/nmpc_solver.h"

// tilt quadrotor
#include "aerial_robot_control/nmpc/tilt_qd_servo_mdl/nmpc_solver.h"
#include "aerial_robot_control/nmpc/tilt_qd_servo_dist_mdl/nmpc_solver.h"
#include "aerial_robot_control/nmpc/tilt_qd_servo_thrust_dist_mdl/nmpc_solver.h"

// tilt tri-rotor
#include "aerial_robot_control/nmpc/tilt_tri_servo_mdl/nmpc_solver.h"

// tilt bi-rotor
#include "aerial_robot_control/nmpc/tilt_bi_servo_mdl/nmpc_solver.h"
#include "aerial_robot_control/nmpc/tilt_bi_2_ord_servo_mdl/nmpc_solver.h"

PLUGINLIB_EXPORT_CLASS(aerial_robot_control::mpc_solver::FixQdMdlMPCSolver,
                       aerial_robot_control::mpc_solver::BaseMPCSolver)

PLUGINLIB_EXPORT_CLASS(aerial_robot_control::mpc_solver::TiltQdServoMdlMPCSolver,
                       aerial_robot_control::mpc_solver::BaseMPCSolver)
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::mpc_solver::TiltQdServoDistMdlMPCSolver,
                       aerial_robot_control::mpc_solver::BaseMPCSolver)
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::mpc_solver::TiltQdServoThrustDistMdlMPCSolver,
                       aerial_robot_control::mpc_solver::BaseMPCSolver)

PLUGINLIB_EXPORT_CLASS(aerial_robot_control::mpc_solver::TiltTriServoMdlMPCSolver,
                       aerial_robot_control::mpc_solver::BaseMPCSolver)

PLUGINLIB_EXPORT_CLASS(aerial_robot_control::mpc_solver::TiltBiServoMdlMPCSolver,
                       aerial_robot_control::mpc_solver::BaseMPCSolver)
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::mpc_solver::TiltBi2OrdServoMdlMPCSolver,
                       aerial_robot_control::mpc_solver::BaseMPCSolver)
