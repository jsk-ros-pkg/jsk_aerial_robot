// -*- mode: c++ -*-
//
// Created by lijinjie on 23/11/25.
//

#ifndef UNDER_ACT_FULL_MPC_SOLVER_H
#define UNDER_ACT_FULL_MPC_SOLVER_H

#endif  // UNDER_ACT_FULL_MPC_SOLVER_H

#include <ros/console.h>
#include <aerial_robot_msgs/PredXU.h>
#include <nav_msgs/Odometry.h>
#include <iostream>

// acados
#include "acados/utils/math.h"
#include "acados/utils/print.h"
#include "acados_c/external_function_interface.h"
#include "acados_c/ocp_nlp_interface.h"
#include "aerial_robot_control/nmpc/under_act_full/c_generated_code/acados_solver_qd_full_model.h"

#define NN QD_FULL_MODEL_N
#define NX QD_FULL_MODEL_NX
#define NZ QD_FULL_MODEL_NZ
#define NU QD_FULL_MODEL_NU
#define NP QD_FULL_MODEL_NP
#define NBX QD_FULL_MODEL_NBX
#define NBX0 QD_FULL_MODEL_NBX0
#define NBU QD_FULL_MODEL_NBU
#define NSBX QD_FULL_MODEL_NSBX
#define NSBU QD_FULL_MODEL_NSBU
#define NSH QD_FULL_MODEL_NSH
#define NSG QD_FULL_MODEL_NSG
#define NSPHI QD_FULL_MODEL_NSPHI
#define NSHN QD_FULL_MODEL_NSHN
#define NSGN QD_FULL_MODEL_NSGN
#define NSPHIN QD_FULL_MODEL_NSPHIN
#define NSBXN QD_FULL_MODEL_NSBXN
#define NS QD_FULL_MODEL_NS
#define NSN QD_FULL_MODEL_NSN
#define NG QD_FULL_MODEL_NG
#define NBXN QD_FULL_MODEL_NBXN
#define NGN QD_FULL_MODEL_NGN
#define NY0 QD_FULL_MODEL_NY0
#define NY QD_FULL_MODEL_NY
#define NYN QD_FULL_MODEL_NYN
#define NH QD_FULL_MODEL_NH
#define NPHI QD_FULL_MODEL_NPHI
#define NHN QD_FULL_MODEL_NHN
#define NPHIN QD_FULL_MODEL_NPHIN
#define NR QD_FULL_MODEL_NR

namespace aerial_robot_control
{

namespace nmpc_under_act_full
{

struct Constraints
{
  double v_max;
  double v_min;
  double w_max;
  double w_min;
  double thrust_max;
  double thrust_min;
};

class MPCSolver
{
public:
  aerial_robot_msgs::PredXU x_u_out_;

  MPCSolver();
  ~MPCSolver();
  void initialize(Constraints& constraints);
  void reset(const aerial_robot_msgs::PredXU& x_u);
  int solve(const nav_msgs::Odometry& odom_now, const aerial_robot_msgs::PredXU& x_u_ref);

private:
  double* new_time_steps;
  qd_full_model_solver_capsule* acados_ocp_capsule_;
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

}  // namespace nmpc_under_act_full

}  // namespace aerial_robot_control
