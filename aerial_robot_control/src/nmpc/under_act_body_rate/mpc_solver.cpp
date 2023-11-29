// -*- mode: c++ -*-
//
// Created by lijinjie on 23/10/22.
//

#include "aerial_robot_control/nmpc/under_act_body_rate/mpc_solver.h"

using namespace aerial_robot_control;

nmpc_under_act_body_rate::MPCSolver::MPCSolver()
{
  // acados related variables should be initialized here
  NN_ = QD_BODY_RATE_MODEL_N, NX_ = QD_BODY_RATE_MODEL_NX, NZ_ = QD_BODY_RATE_MODEL_NZ, NU_ = QD_BODY_RATE_MODEL_NU,
  NP_ = QD_BODY_RATE_MODEL_NP, NBX_ = QD_BODY_RATE_MODEL_NBX, NBX0_ = QD_BODY_RATE_MODEL_NBX0,
  NBU_ = QD_BODY_RATE_MODEL_NBU, NSBX_ = QD_BODY_RATE_MODEL_NSBX, NSBU_ = QD_BODY_RATE_MODEL_NSBU,
  NSH_ = QD_BODY_RATE_MODEL_NSH, NSG_ = QD_BODY_RATE_MODEL_NSG, NSPHI_ = QD_BODY_RATE_MODEL_NSPHI,
  NSHN_ = QD_BODY_RATE_MODEL_NSHN, NSGN_ = QD_BODY_RATE_MODEL_NSGN, NSPHIN_ = QD_BODY_RATE_MODEL_NSPHIN,
  NSBXN_ = QD_BODY_RATE_MODEL_NSBXN, NS_ = QD_BODY_RATE_MODEL_NS, NSN_ = QD_BODY_RATE_MODEL_NSN,
  NG_ = QD_BODY_RATE_MODEL_NG, NBXN_ = QD_BODY_RATE_MODEL_NBXN, NGN_ = QD_BODY_RATE_MODEL_NGN,
  NY0_ = QD_BODY_RATE_MODEL_NY0, NY_ = QD_BODY_RATE_MODEL_NY, NYN_ = QD_BODY_RATE_MODEL_NYN,
  NH_ = QD_BODY_RATE_MODEL_NH, NPHI_ = QD_BODY_RATE_MODEL_NPHI, NHN_ = QD_BODY_RATE_MODEL_NHN,
  NPHIN_ = QD_BODY_RATE_MODEL_NPHIN, NR_ = QD_BODY_RATE_MODEL_NR;

  // acados related function pointers should be initialized here
  acados_create_capsule = reinterpret_cast<void* (*)()>(qd_body_rate_model_acados_create_capsule);
  acados_get_nlp_config = reinterpret_cast<ocp_nlp_config* (*)(void*)>(qd_body_rate_model_acados_get_nlp_config);
  acados_get_nlp_dims = reinterpret_cast<ocp_nlp_dims* (*)(void*)>(qd_body_rate_model_acados_get_nlp_dims);
  acados_get_nlp_in = reinterpret_cast<ocp_nlp_in* (*)(void*)>(qd_body_rate_model_acados_get_nlp_in);
  acados_get_nlp_out = reinterpret_cast<ocp_nlp_out* (*)(void*)>(qd_body_rate_model_acados_get_nlp_out);
  acados_get_nlp_solver = reinterpret_cast<ocp_nlp_solver* (*)(void*)>(qd_body_rate_model_acados_get_nlp_solver);
  acados_get_nlp_opts = reinterpret_cast<void* (*)(void*)>(qd_body_rate_model_acados_get_nlp_opts);
  acados_create_with_discretization = reinterpret_cast<int (*)(void*, int, double*)>(qd_body_rate_model_acados_create);
  acados_solve = reinterpret_cast<int (*)(void*)>(qd_body_rate_model_acados_solve);
  acados_update_params = reinterpret_cast<int (*)(void*, int, double*, int)>(qd_body_rate_model_acados_update_params);
  acados_update_params_sparse =
      reinterpret_cast<int (*)(void*, int, int*, double*, int)>(qd_body_rate_model_acados_update_params_sparse);
  acados_free = reinterpret_cast<int (*)(void*)>(qd_body_rate_model_acados_free);
  acados_free_capsule = reinterpret_cast<int (*)(void*)>(qd_body_rate_model_acados_free_capsule);
  acados_print_stats = reinterpret_cast<void (*)(void*)>(qd_body_rate_model_acados_print_stats);
}

void nmpc_under_act_body_rate::MPCSolver::setFeedbackConstraints(const nav_msgs::Odometry& odom_now)
{
  double bx0[NBX0_];
  bx0[0] = odom_now.pose.pose.position.x;
  bx0[1] = odom_now.pose.pose.position.y;
  bx0[2] = odom_now.pose.pose.position.z;
  bx0[3] = odom_now.twist.twist.linear.x;
  bx0[4] = odom_now.twist.twist.linear.y;
  bx0[5] = odom_now.twist.twist.linear.z;
  bx0[6] = odom_now.pose.pose.orientation.w;
  bx0[7] = odom_now.pose.pose.orientation.x;
  bx0[8] = odom_now.pose.pose.orientation.y;
  bx0[9] = odom_now.pose.pose.orientation.z;

  ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, 0, "lbx", bx0);
  ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, 0, "ubx", bx0);
}

void nmpc_under_act_body_rate::MPCSolver::updateConstraints(double v_max, double v_min, double w_max, double w_min,
                                                            double c_thrust_max, double c_thrust_min, double mass,
                                                            double gravity)
{
  gravity_ = gravity;
  // Please note that the constraints have been set up inside the python interface. Only minimum adjustments are needed.
  // bx_0: initial state. Note that the value of lbx0 and ubx0 will be set in solve() function, feedback constraints bx
  double lbx[] = { v_min, v_min, v_min };
  assert(sizeof lbx / sizeof lbx[0] == NBX_);
  double ubx[] = { v_max, v_max, v_max };
  assert(sizeof ubx / sizeof ubx[0] == NBX_);
  for (int i = 1; i < NN_; i++)
  {
    ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, i, "lbx", lbx);
    ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, i, "ubx", ubx);
  }
  // bx_e: terminal state
  double lbx_e[] = { v_min, v_min, v_min };
  assert(sizeof lbx_e / sizeof lbx_e[0] == NBXN_);
  double ubx_e[] = { v_max, v_max, v_max };
  assert(sizeof ubx_e / sizeof ubx_e[0] == NBXN_);
  ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, NN_, "lbx", lbx_e);
  ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, NN_, "ubx", ubx_e);

  // bu
  double lbu[] = { w_min, w_min, w_min, c_thrust_min / mass };
  assert(sizeof lbu / sizeof lbu[0] == NBU_);
  double ubu[] = { w_max, w_max, w_max, c_thrust_max / mass };
  assert(sizeof ubu / sizeof ubu[0] == NBU_);
  for (int i = 0; i < NN_; i++)
  {
    ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, i, "lbu", lbu);
    ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, i, "ubu", ubu);
  }
}

void nmpc_under_act_body_rate::MPCSolver::initParameters()
{
  // Note that the code here initializes all parameters, including variables and constants.
  // Constants are initialized only once, while variables are initialized in every iteration
  double p[] = { 1.0, 0.0, 0.0, 0.0, gravity_ };
  assert(sizeof p / sizeof p[0] == NP_);
  for (int i = 0; i < NN_; i++)
  {
    acados_update_params(acados_ocp_capsule_, i, p, NP_);
  }
  acados_update_params(acados_ocp_capsule_, NN_, p, NP_);
}
