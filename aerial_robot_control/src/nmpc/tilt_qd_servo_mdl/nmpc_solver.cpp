//
// Created by lijinjie on 23/11/29.
//

#include "aerial_robot_control/nmpc/tilt_qd_servo_mdl/nmpc_solver.h"

using namespace aerial_robot_control;

void nmpc_over_act_full::MPCSolver::initialize()
{
  /* Set rti_phase */
  int rti_phase = 0;  //  (1) preparation, (2) feedback, (0) both. 0 is default
  ocp_nlp_solver_opts_set(nlp_config_, nlp_opts_, "rti_phase", &rti_phase);

  /* init weight matrix, W is a getCostWeightDim(0) * getCostWeightDim(0) double matrix */
  int nw = NX_ + NU_;

  W_ = (double*)malloc((nw * nw) * sizeof(double));
  for (int i = 0; i < nw * nw; i++)
    W_[i] = 0.0;
  WN_ = (double*)malloc((NX_ * NX_) * sizeof(double));
  for (int i = 0; i < NX_ * NX_; i++)
    WN_[i] = 0.0;

  //  /* Set constraints */
  //  // Please note that the constraints have been set up inside the python interface. Only minimum adjustments are
  //  needed.
  //  // bx_0: initial state. Note that the value of lbx0 and ubx0 will be set in solve() function, feedback constraints
  //  // bx
  //  double lbx[NBX] = { constraints.v_min, constraints.v_min, constraints.v_min,
  //                      constraints.w_min, constraints.w_min, constraints.w_min };
  //  double ubx[NBX] = { constraints.v_max, constraints.v_max, constraints.v_max,
  //                      constraints.w_max, constraints.w_max, constraints.w_max };
  //  for (int i = 1; i < NN_; i++)
  //  {
  //    ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, i, "lbx", lbx);
  //    ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, i, "ubx", ubx);
  //  }
  //  // bx_e: terminal state
  //  double lbx_e[NBXN] = { constraints.v_min, constraints.v_min, constraints.v_min,
  //                         constraints.w_min, constraints.w_min, constraints.w_min };
  //  double ubx_e[NBXN] = { constraints.v_max, constraints.v_max, constraints.v_max,
  //                         constraints.w_max, constraints.w_max, constraints.w_max };
  //  ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, NN_, "lbx", lbx_e);
  //  ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, NN_, "ubx", ubx_e);
  //
  //  // bu
  //  double lbu[NBU] = { constraints.thrust_min, constraints.thrust_min, constraints.thrust_min, constraints.thrust_min
  //  }; double ubu[NBU] = { constraints.thrust_max, constraints.thrust_max, constraints.thrust_max,
  //  constraints.thrust_max }; for (int i = 0; i < NN_; i++)
  //  {
  //    ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, i, "lbu", lbu);
  //    ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, i, "ubu", ubu);
  //  }

  /* Set parameters */
  // Note that the code here initializes all parameters, including variables and constants.
  // Constants are initialized only once, while variables are initialized in every iteration
  double p[] = { 1.0, 0.0, 0.0, 0.0 };
  for (int i = 0; i < NN_; i++)
  {
    acados_update_params(i, p);
  }
  acados_update_params(NN_, p);
}

int nmpc_over_act_full::MPCSolver::solve(const nav_msgs::Odometry& odom_now, const double joint_angles[4], const bool is_debug)
{
  std::vector<double> bx0(NBX0_);
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
  bx0[10] = odom_now.twist.twist.angular.x;
  bx0[11] = odom_now.twist.twist.angular.y;
  bx0[12] = odom_now.twist.twist.angular.z;
  bx0[13] = joint_angles[0];
  bx0[14] = joint_angles[1];
  bx0[15] = joint_angles[2];
  bx0[16] = joint_angles[3];

  setFeedbackConstraints(bx0);

  double min_time = solveOCPOnce();

  getSolution();

  if (is_debug)
  {
    printSolution();
    printStatus(min_time);
  }

  return 0;
}

void nmpc_over_act_full::MPCSolver::setCostWDiagElement(int index, double value, bool is_set_WN) const
{
  if (index < NX_ + NU_)
    W_[index + index * (NX_ + NU_)] = (double)value;
  else
    ROS_ERROR("index should be less than NX_ + NU_");

  if (is_set_WN)
  {
    if (index < NX_)
      WN_[index + index * NX_] = (double)value;
    else
      ROS_ERROR("index should be less than NX_");
  }
}

void nmpc_over_act_full::MPCSolver::setCostWeight(bool is_update_W, bool is_update_WN)
{
  if (is_update_W)
  {
    for (int i = 0; i < NN_; i++)
      ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, i, "W", W_);
  }
  if (is_update_WN)
    ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, NN_, "W", WN_);
}
