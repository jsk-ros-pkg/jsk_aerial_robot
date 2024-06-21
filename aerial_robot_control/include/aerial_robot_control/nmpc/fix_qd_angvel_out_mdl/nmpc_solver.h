// -*- mode: c++ -*-
//
// Created by lijinjie on 23/10/22.
//

#ifndef UNDER_ACT_BODY_RATE_MPC_SOLVER_H
#define UNDER_ACT_BODY_RATE_MPC_SOLVER_H

#endif  // UNDER_ACT_BODY_RATE_MPC_SOLVER_H

#include <ros/console.h>
#include <aerial_robot_msgs/PredXU.h>
#include <nav_msgs/Odometry.h>
#include <iostream>

// acados
#include "acados/utils/math.h"
#include "acados/utils/print.h"
#include "acados_c/external_function_interface.h"
#include "acados_c/ocp_nlp_interface.h"
#include "aerial_robot_control/nmpc/fix_qd_angvel_out_mdl/c_generated_code/acados_solver_fix_qd_angvel_out_mdl.h"

#define NN FIX_QD_ANGVEL_OUT_MDL_N
#define NX FIX_QD_ANGVEL_OUT_MDL_NX
#define NZ FIX_QD_ANGVEL_OUT_MDL_NZ
#define NU FIX_QD_ANGVEL_OUT_MDL_NU
#define NP FIX_QD_ANGVEL_OUT_MDL_NP
#define NBX FIX_QD_ANGVEL_OUT_MDL_NBX
#define NBX0 FIX_QD_ANGVEL_OUT_MDL_NBX0
#define NBU FIX_QD_ANGVEL_OUT_MDL_NBU
#define NSBX FIX_QD_ANGVEL_OUT_MDL_NSBX
#define NSBU FIX_QD_ANGVEL_OUT_MDL_NSBU
#define NSH FIX_QD_ANGVEL_OUT_MDL_NSH
#define NSG FIX_QD_ANGVEL_OUT_MDL_NSG
#define NSPHI FIX_QD_ANGVEL_OUT_MDL_NSPHI
#define NSHN FIX_QD_ANGVEL_OUT_MDL_NSHN
#define NSGN FIX_QD_ANGVEL_OUT_MDL_NSGN
#define NSPHIN FIX_QD_ANGVEL_OUT_MDL_NSPHIN
#define NSBXN FIX_QD_ANGVEL_OUT_MDL_NSBXN
#define NS FIX_QD_ANGVEL_OUT_MDL_NS
#define NSN FIX_QD_ANGVEL_OUT_MDL_NSN
#define NG FIX_QD_ANGVEL_OUT_MDL_NG
#define NBXN FIX_QD_ANGVEL_OUT_MDL_NBXN
#define NGN FIX_QD_ANGVEL_OUT_MDL_NGN
#define NY0 FIX_QD_ANGVEL_OUT_MDL_NY0
#define NY FIX_QD_ANGVEL_OUT_MDL_NY
#define NYN FIX_QD_ANGVEL_OUT_MDL_NYN
#define NH FIX_QD_ANGVEL_OUT_MDL_NH
#define NPHI FIX_QD_ANGVEL_OUT_MDL_NPHI
#define NHN FIX_QD_ANGVEL_OUT_MDL_NHN
#define NPHIN FIX_QD_ANGVEL_OUT_MDL_NPHIN
#define NR FIX_QD_ANGVEL_OUT_MDL_NR

namespace aerial_robot_control
{

namespace nmpc_under_act_body_rate
{

struct PhysicalParams
{
  // physical params
  double mass;
  double gravity;

  // physical constraints
  double v_max;
  double v_min;
  double w_max;
  double w_min;
  double c_thrust_max;
  double c_thrust_min;
};

class MPCSolver
{
public:
  aerial_robot_msgs::PredXU x_u_out_;

  MPCSolver();
  ~MPCSolver();
  void initialize(PhysicalParams& phys_params);
  void reset(const aerial_robot_msgs::PredXU& x_u);
  int solve(const nav_msgs::Odometry& odom_now, const aerial_robot_msgs::PredXU& x_u_ref);

private:
  double* new_time_steps;
  fix_qd_angvel_out_mdl_solver_capsule* acados_ocp_capsule_;
  ocp_nlp_config* nlp_config_;
  ocp_nlp_dims* nlp_dims_;
  ocp_nlp_in* nlp_in_;
  ocp_nlp_out* nlp_out_;
  ocp_nlp_solver* nlp_solver_;
  void* nlp_opts_;

  void setReference(const aerial_robot_msgs::PredXU& x_u_ref, unsigned int x_stride, unsigned int u_stride);
  void setFeedbackConstraints(const nav_msgs::Odometry& odom_now);
  double solveOCPInLoop(int N_timings);
  void getSolution(unsigned int x_stride, unsigned int u_stride);

  void printStatus(int N_timings, double min_time);
  void printSolution();
};

void initPredXU(aerial_robot_msgs::PredXU& x_u);

}  // namespace nmpc_under_act_body_rate

}  // namespace aerial_robot_control
