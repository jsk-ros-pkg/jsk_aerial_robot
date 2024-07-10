//
// Created by lijinjie on 23/11/29.
//

#include "aerial_robot_control/nmpc/tilt_qd_servo_mdl/nmpc_solver.h"

using namespace aerial_robot_control;

void nmpc_over_act_full::MPCSolver::initialize()
{
  /* Allocate the array and fill it accordingly */
  acados_ocp_capsule_ = tilt_qd_servo_mdl_acados_create_capsule();

  new_time_steps = nullptr;

  int status = tilt_qd_servo_mdl_acados_create_with_discretization(acados_ocp_capsule_, NN_, new_time_steps);
  if (status)
  {
    ROS_WARN("tilt_qd_servo_mdl_acados_create() returned status %d. Exiting.\n", status);
    exit(1);
  }

  nlp_config_ = tilt_qd_servo_mdl_acados_get_nlp_config(acados_ocp_capsule_);
  nlp_dims_ = tilt_qd_servo_mdl_acados_get_nlp_dims(acados_ocp_capsule_);
  nlp_in_ = tilt_qd_servo_mdl_acados_get_nlp_in(acados_ocp_capsule_);
  nlp_out_ = tilt_qd_servo_mdl_acados_get_nlp_out(acados_ocp_capsule_);
  nlp_solver_ = tilt_qd_servo_mdl_acados_get_nlp_solver(acados_ocp_capsule_);
  nlp_opts_ = tilt_qd_servo_mdl_acados_get_nlp_opts(acados_ocp_capsule_);

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
    tilt_qd_servo_mdl_acados_update_params(acados_ocp_capsule_, i, p, NP_);
  }
  tilt_qd_servo_mdl_acados_update_params(acados_ocp_capsule_, NN_, p, NP_);

  /* Initialize output value */
  initPredXU(x_u_out_);
}

nmpc_over_act_full::MPCSolver::~MPCSolver()
{
  // 1. free solver
  int status = tilt_qd_servo_mdl_acados_free(acados_ocp_capsule_);
  if (status)
    ROS_WARN("tilt_qd_servo_mdl_acados_free() returned status %d. \n", status);

  // 2. free solver capsule
  status = tilt_qd_servo_mdl_acados_free_capsule(acados_ocp_capsule_);
  if (status)
    ROS_WARN("tilt_qd_servo_mdl_acados_free_capsule() returned status %d. \n", status);
}

void nmpc_over_act_full::MPCSolver::reset(const aerial_robot_msgs::PredXU& x_u)
{
  const unsigned int x_stride = x_u.x.layout.dim[1].stride;
  const unsigned int u_stride = x_u.u.layout.dim[1].stride;

  // reset initial guess
  double x[NX_];
  double u[NU_];
  for (int i = 0; i < NN_; i++)
  {
    std::copy(x_u.x.data.begin() + x_stride * i, x_u.x.data.begin() + x_stride * (i + 1), x);
    ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, i, "x", x);

    std::copy(x_u.u.data.begin() + u_stride * i, x_u.u.data.begin() + u_stride * (i + 1), u);
    ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, i, "u", u);
  }
  std::copy(x_u.x.data.begin() + x_stride * NN_, x_u.x.data.begin() + x_stride * (NN_ + 1), x);
  ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, NN_, "x", x);
}

int nmpc_over_act_full::MPCSolver::solve(const nav_msgs::Odometry& odom_now, double joint_angles[4],
                                         const aerial_robot_msgs::PredXU& x_u_ref, const bool is_debug)
{
  const unsigned int x_stride = x_u_ref.x.layout.dim[1].stride;
  const unsigned int u_stride = x_u_ref.u.layout.dim[1].stride;
  setReference(x_u_ref, x_stride, u_stride);

  setFeedbackConstraints(odom_now, joint_angles);

  double min_time = solveOCPOnce();

  getSolution(x_stride, u_stride);

  if (is_debug)
  {
    printSolution();
    printStatus(min_time);
  }

  return 0;
}

void nmpc_over_act_full::MPCSolver::initPredXU(aerial_robot_msgs::PredXU& x_u)
{
  x_u.x.layout.dim.emplace_back();
  x_u.x.layout.dim.emplace_back();
  x_u.x.layout.dim[0].label = "horizon";
  x_u.x.layout.dim[0].size = NN_ + 1;
  x_u.x.layout.dim[0].stride = (NN_ + 1) * NX_;
  x_u.x.layout.dim[1].label = "state";
  x_u.x.layout.dim[1].size = NX_;
  x_u.x.layout.dim[1].stride = NX_;
  x_u.x.layout.data_offset = 0;
  x_u.x.data.resize((NN_ + 1) * NX_);
  std::fill(x_u.x.data.begin(), x_u.x.data.end(), 0.0);
  // quaternion
  for (int i = 6; i < (NN_ + 1) * NX_; i += NX_)
    x_u.x.data[i] = 1.0;

  x_u.u.layout.dim.emplace_back();
  x_u.u.layout.dim.emplace_back();
  x_u.u.layout.dim[0].label = "horizon";
  x_u.u.layout.dim[0].size = NN_;
  x_u.u.layout.dim[0].stride = NN_ * NU_;
  x_u.u.layout.dim[1].label = "input";
  x_u.u.layout.dim[1].size = NU_;
  x_u.u.layout.dim[1].stride = NU_;
  x_u.u.layout.data_offset = 0;
  x_u.u.data.resize(NN_ * NU_);
  std::fill(x_u.u.data.begin(), x_u.u.data.end(), 0.0);
}

void nmpc_over_act_full::MPCSolver::setReference(const aerial_robot_msgs::PredXU& x_u_ref, const unsigned int x_stride,
                                                 const unsigned int u_stride)
{
  double yr[NX_ + NU_];
  double qr[4];
  int qr_idx[] = { 0, 1, 2, 3 };
  for (int i = 0; i < NN_; i++)
  {
    // yr = np.concatenate((xr[i, :], ur[i, :]))
    std::copy(x_u_ref.x.data.begin() + x_stride * i, x_u_ref.x.data.begin() + x_stride * (i + 1), yr);
    std::copy(x_u_ref.u.data.begin() + u_stride * i, x_u_ref.u.data.begin() + u_stride * (i + 1), yr + NX_);
    ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, i, "y_ref", yr);

    // quaternions
    std::copy(x_u_ref.x.data.begin() + x_stride * i + 6, x_u_ref.x.data.begin() + x_stride * i + 10, qr);
    tilt_qd_servo_mdl_acados_update_params_sparse(acados_ocp_capsule_, i, qr_idx, qr, 4);
  }
  // final x and p, no u
  double xr[NX_];
  std::copy(x_u_ref.x.data.begin() + x_stride * NN_, x_u_ref.x.data.begin() + x_stride * (NN_ + 1), xr);
  ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, NN_, "y_ref", xr);

  std::copy(x_u_ref.x.data.begin() + x_stride * NN_ + 6, x_u_ref.x.data.begin() + x_stride * NN_ + 10, qr);
  tilt_qd_servo_mdl_acados_update_params_sparse(acados_ocp_capsule_, NN_, qr_idx, qr, 4);
}

void nmpc_over_act_full::MPCSolver::setFeedbackConstraints(const nav_msgs::Odometry& odom_now,
                                                           const double joint_angles[4])
{
  // TODO: modify, to pass in variable array
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
  bx0[10] = odom_now.twist.twist.angular.x;
  bx0[11] = odom_now.twist.twist.angular.y;
  bx0[12] = odom_now.twist.twist.angular.z;
  bx0[13] = joint_angles[0];
  bx0[14] = joint_angles[1];
  bx0[15] = joint_angles[2];
  bx0[16] = joint_angles[3];

  ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, 0, "lbx", bx0);
  ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, 0, "ubx", bx0);
}

double nmpc_over_act_full::MPCSolver::solveOCPOnce()
{
  double min_time = 1e12;
  double elapsed_time;

  int status = tilt_qd_servo_mdl_acados_solve(acados_ocp_capsule_);
  if (status != ACADOS_SUCCESS)
  {
    ROS_WARN("tilt_qd_servo_mdl_acados_solve() returned status %d.\n", status);
  }

  ocp_nlp_get(nlp_config_, nlp_solver_, "time_tot", &elapsed_time);
  min_time = MIN(elapsed_time, min_time);

  return min_time;
}

void nmpc_over_act_full::MPCSolver::getSolution(const unsigned int x_stride, const unsigned int u_stride)
{
  for (int i = 0; i < NN_; i++)
  {
    ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, i, "x", x_u_out_.x.data.data() + x_stride * i);
    ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, i, "u", x_u_out_.u.data.data() + u_stride * i);
  }
  ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, NN_, "x", x_u_out_.x.data.data() + x_stride * NN_);
}

void nmpc_over_act_full::MPCSolver::printSolution()
{
  std::stringstream ss;

  ss << "\n--- x_traj ---\n";
  for (int i = 0; i <= NN_; i++)
  {
    ss << "X Row " << i << ":\n";
    for (int j = 0; j < NX_; j++)
    {
      int index = i * NX_ + j;
      ss << x_u_out_.x.data[index] << " ";
    }
    ss << "\n";
  }
  ROS_INFO_STREAM(ss.str());  // Logging the x_traj
  ss.str("");                 // Clearing the stringstream

  ss << "\n--- u_traj ---\n";
  for (int i = 0; i < NN_; i++)
  {
    ss << "U Row " << i << ":\n";
    for (int j = 0; j < NU_; j++)
    {
      int index = i * NU_ + j;
      ss << x_u_out_.u.data[index] << " ";
    }
    ss << "\n";
  }
  ROS_INFO_STREAM(ss.str());  // Logging the u_traj
}

void nmpc_over_act_full::MPCSolver::printStatus(const double min_time)
{
  double kkt_norm_inf;
  int sqp_iter;

  ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, 0, "kkt_norm_inf", &kkt_norm_inf);
  ocp_nlp_get(nlp_config_, nlp_solver_, "sqp_iter", &sqp_iter);
  tilt_qd_servo_mdl_acados_print_stats(acados_ocp_capsule_);
  ROS_DEBUG("\nSolver info:\n");
  ROS_DEBUG(" SQP iterations %2d\n minimum time for 1 solve %f [ms]\n KKT %e\n", sqp_iter, min_time * 1000,
            kkt_norm_inf);
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
