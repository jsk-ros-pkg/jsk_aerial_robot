// -*- mode: c++ -*-
//
// Created by lijinjie on 23/10/22.
//

#include "aerial_robot_control/control/base/mpc_solver.h"

MPC::MPCSolver::MPCSolver()
{
  MPC::initPredXU(x_u_out_);

  initSolver();

  //  set constraints idx for x0
  int idxbx0[NBX0];
  for (int i = 0; i < 6; i++)  // free quaternion
    idxbx0[i] = i;
  ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, 0, "idxbx", idxbx0);

  // set rti_phase TODO: check this according to the paper
  int rti_phase = 0;
  ocp_nlp_solver_opts_set(nlp_config_, nlp_opts_, "rti_phase", &rti_phase);
}

MPC::MPCSolver::~MPCSolver()
{
  // 1. free solver
  if (qd_body_rate_model_acados_free(acados_ocp_capsule_))
    std::cout << "qd_body_rate_model_acados_free() returned status " << status_ << ".\n";

  // 2. free solver capsule
  if (qd_body_rate_model_acados_free_capsule(acados_ocp_capsule_))
    std::cout << "qd_body_rate_model_acados_free_capsule() returned status " << status_ << ".\n";
}

void MPC::MPCSolver::reset(const aerial_robot_msgs::PredXU& x_u)
{
  const unsigned int x_stride = x_u.x.layout.dim[1].stride;
  const unsigned int u_stride = x_u.u.layout.dim[1].stride;

  // reset initial guess
  double x[NX];
  double u[NU];
  for (int i = 0; i < N; i++)
  {
    std::copy(x_u.x.data.begin() + x_stride * i, x_u.x.data.begin() + x_stride * (i + 1), x);
    ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, i, "x", x);

    std::copy(x_u.u.data.begin() + u_stride * i, x_u.u.data.begin() + u_stride * (i + 1), u);
    ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, i, "u", u);
  }
  std::copy(x_u.x.data.begin() + x_stride * N, x_u.x.data.begin() + x_stride * (N + 1), x);
  ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, N, "x", x);
}

int MPC::MPCSolver::solve(const nav_msgs::Odometry& odom_now, const aerial_robot_msgs::PredXU& x_u_ref)
{
  const unsigned int x_stride = x_u_ref.x.layout.dim[1].stride;
  const unsigned int u_stride = x_u_ref.u.layout.dim[1].stride;

  /* prepare evaluation */
  int N_timings = 1;

  setReference(x_u_ref, x_stride, u_stride);

  setFeedbackConstraints(odom_now);

  double min_time = solveOCPInLoop(N_timings);

  getSolution(x_stride, u_stride);

  if (getenv("ROS_DEBUG"))
  {
    printSolution();
    printStatus(N_timings, min_time);
  }

  return status_;
}

void MPC::initPredXU(aerial_robot_msgs::PredXU& x_u)
{
  x_u.x.layout.dim.emplace_back();
  x_u.x.layout.dim.emplace_back();
  x_u.x.layout.dim[0].label = "horizon";
  x_u.x.layout.dim[0].size = N + 1;
  x_u.x.layout.dim[0].stride = (N + 1) * NX;
  x_u.x.layout.dim[1].label = "state";
  x_u.x.layout.dim[1].size = NX;
  x_u.x.layout.dim[1].stride = NX;
  x_u.x.layout.data_offset = 0;

  x_u.u.layout.dim.emplace_back();
  x_u.u.layout.dim.emplace_back();
  x_u.u.layout.dim[0].label = "horizon";
  x_u.u.layout.dim[0].size = N;
  x_u.u.layout.dim[0].stride = N * NU;
  x_u.u.layout.dim[1].label = "input";
  x_u.u.layout.dim[1].size = NU;
  x_u.u.layout.dim[1].stride = NU;
  x_u.u.layout.data_offset = 0;
}

void MPC::MPCSolver::initSolver()
{
  acados_ocp_capsule_ = qd_body_rate_model_acados_create_capsule();

  // 1. allocate the array and fill it accordingly
  double* new_time_steps = nullptr;
  if (qd_body_rate_model_acados_create_with_discretization(acados_ocp_capsule_, N, new_time_steps))
  {
    printf("qd_body_rate_model_acados_create() returned status %d. Exiting.\n", status_);
    exit(1);
  }

  nlp_config_ = qd_body_rate_model_acados_get_nlp_config(acados_ocp_capsule_);
  nlp_dims_ = qd_body_rate_model_acados_get_nlp_dims(acados_ocp_capsule_);
  nlp_in_ = qd_body_rate_model_acados_get_nlp_in(acados_ocp_capsule_);
  nlp_out_ = qd_body_rate_model_acados_get_nlp_out(acados_ocp_capsule_);
  nlp_solver_ = qd_body_rate_model_acados_get_nlp_solver(acados_ocp_capsule_);
  nlp_opts_ = qd_body_rate_model_acados_get_nlp_opts(acados_ocp_capsule_);
}

void MPC::MPCSolver::setReference(const aerial_robot_msgs::PredXU& x_u_ref, const unsigned int x_stride,
                                  const unsigned int u_stride)
{
  double yr[NX + NU];
  double p[NP];
  for (int i = 0; i < N; i++)
  {
    // yr = np.concatenate((xr[i, :], ur[i, :]))
    std::copy(x_u_ref.x.data.begin() + x_stride * i, x_u_ref.x.data.begin() + x_stride * (i + 1), yr);
    std::copy(x_u_ref.u.data.begin() + u_stride * i, x_u_ref.u.data.begin() + u_stride * (i + 1), yr + NX);
    ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, i, "y_ref", yr);

    // quaternions
    std::copy(x_u_ref.x.data.begin() + x_stride * i + 6, x_u_ref.x.data.begin() + x_stride * i + 10, p);
    ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, i, "p", p);
  }
  // final x and p, no u
  double xr[NX];
  std::copy(x_u_ref.x.data.begin() + x_stride * N, x_u_ref.x.data.begin() + x_stride * (N + 1), xr);
  ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, N, "y_ref", xr);

  std::copy(x_u_ref.x.data.begin() + x_stride * N + 6, x_u_ref.x.data.begin() + x_stride * N + 10, p);
  ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, N, "p", p);
}

void MPC::MPCSolver::setFeedbackConstraints(const nav_msgs::Odometry& odom_now)
{
  double bx0[NBX0];
  bx0[0] = odom_now.pose.pose.position.x;
  bx0[1] = odom_now.pose.pose.position.y;
  bx0[2] = odom_now.pose.pose.position.z;
  bx0[3] = odom_now.twist.twist.linear.x;
  bx0[4] = odom_now.twist.twist.linear.y;
  bx0[5] = odom_now.twist.twist.linear.z;
  //  bx0[6] = odom_now.pose.pose.orientation.w;
  //  bx0[7] = odom_now.pose.pose.orientation.x;
  //  bx0[8] = odom_now.pose.pose.orientation.y;
  //  bx0[9] = odom_now.pose.pose.orientation.z;

  ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, 0, "lbx", bx0);
  ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, 0, "ubx", bx0);
}

double MPC::MPCSolver::solveOCPInLoop(const int N_timings)
{
  double min_time = 1e12;
  double elapsed_time;

  for (int i = 0; i < N_timings; i++)
  {
    if (qd_body_rate_model_acados_solve(acados_ocp_capsule_) != ACADOS_SUCCESS)
    {
      std::cout << "qd_body_rate_model_acados_solve() returned status " << status_ << ".\n";
    }

    ocp_nlp_get(nlp_config_, nlp_solver_, "time_tot", &elapsed_time);
    min_time = MIN(elapsed_time, min_time);
  }

  return min_time;
}

void MPC::MPCSolver::getSolution(const unsigned int x_stride, const unsigned int u_stride)
{
  for (int i = 0; i < N; i++)
  {
    ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, i, "x", x_u_out_.x.data.data() + x_stride * i);
    ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, i, "u", x_u_out_.u.data.data() + u_stride * i);
  }
  ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, N, "x", x_u_out_.x.data.data() + x_stride * N);
}

void MPC::MPCSolver::printSolution()
{
  std::cout << "\n--- x_traj ---\n" << std::endl;

  for (int i = 0; i <= N; i++)
  {
    std::cout << "X Row " << i << ":" << std::endl;
    for (int j = 0; j < NX; j++)
    {
      int index = i * NX + j;
      std::cout << x_u_out_.x.data[index] << " ";
    }
    std::cout << std::endl;
  }

  std::cout << "\n--- u_traj ---\n" << std::endl;

  for (int i = 0; i < N; i++)
  {
    std::cout << "U Row " << i << ":" << std::endl;
    for (int j = 0; j < NU; j++)
    {
      int index = i * NU + j;
      std::cout << x_u_out_.u.data[index] << " ";
    }
    std::cout << std::endl;
  }
}

void MPC::MPCSolver::printStatus(const int N_timings, const double min_time)
{
  double kkt_norm_inf;
  int sqp_iter;

  ROS_DEBUG("\nsolved ocp %d times, solution printed above\n\n", N_timings);

  ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, 0, "kkt_norm_inf", &kkt_norm_inf);
  ocp_nlp_get(nlp_config_, nlp_solver_, "sqp_iter", &sqp_iter);
  qd_body_rate_model_acados_print_stats(acados_ocp_capsule_);
  ROS_DEBUG("\nSolver info:\n");
  ROS_DEBUG(" SQP iterations %2d\n minimum time for %d solve %f [ms]\n KKT %e\n", sqp_iter, N_timings, min_time * 1000,
            kkt_norm_inf);
}
