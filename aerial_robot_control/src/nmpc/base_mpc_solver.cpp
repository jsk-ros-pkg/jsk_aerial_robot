//
// Created by li-jinjie on 23-11-25.
//

#include "aerial_robot_control/nmpc/base_mpc_solver.h"

using namespace aerial_robot_control;

base_nmpc::BaseMPCSolver::BaseMPCSolver()
{
  //  // acados related variables should be initialized here
  //  NN_ = int(), NX_ = int(), NZ_ = int(), NU_ = int(), NP_ = int(), NBX_ = int(), NBX0_ = int(), NBU_ = int(),
  //  NSBX_ = int(), NSBU_ = int(), NSH_ = int(), NSG_ = int(), NSPHI_ = int(), NSHN_ = int(), NSGN_ = int(),
  //  NSPHIN_ = int(), NSBXN_ = int(), NS_ = int(), NSN_ = int(), NG_ = int(), NBXN_ = int(), NGN_ = int(), NY0_ =
  //  int(), NY_ = int(), NYN_ = int(), NH_ = int(), NPHI_ = int(), NHN_ = int(), NPHIN_ = int(), NR_ = int();
  //
  //  // acados related function pointers should be initialized here
  //  acados_create_capsule = nullptr;
  //  acados_get_nlp_config = nullptr;
  //  acados_get_nlp_dims = nullptr;
  //  acados_get_nlp_in = nullptr;
  //  acados_get_nlp_out = nullptr;
  //  acados_get_nlp_solver = nullptr;
  //  acados_get_nlp_opts = nullptr;
  //  acados_create_with_discretization = nullptr;
  //  acados_solve = nullptr;
  //  acados_update_params = nullptr;
  //  acados_update_params_sparse = nullptr;
  //  acados_free = nullptr;
  //  acados_free_capsule = nullptr;
  //  acados_print_stats = nullptr;
}

void base_nmpc::BaseMPCSolver::initialize()
{
  /* Allocate the array and fill it accordingly */
  acados_ocp_capsule_ = acados_create_capsule();

  new_time_steps = nullptr;

  int status = acados_create_with_discretization(acados_ocp_capsule_, NN_, new_time_steps);
  if (status)
  {
    printf("qd_full_model_acados_create() returned status %d. Exiting.\n", status);
    exit(1);
  }

  nlp_config_ = acados_get_nlp_config(acados_ocp_capsule_);
  nlp_dims_ = acados_get_nlp_dims(acados_ocp_capsule_);
  nlp_in_ = acados_get_nlp_in(acados_ocp_capsule_);
  nlp_out_ = acados_get_nlp_out(acados_ocp_capsule_);
  nlp_solver_ = acados_get_nlp_solver(acados_ocp_capsule_);
  nlp_opts_ = acados_get_nlp_opts(acados_ocp_capsule_);

  /* Set rti_phase */
  int rti_phase = 0;  //  (1) preparation, (2) feedback, (0) both. 0 is default
  ocp_nlp_solver_opts_set(nlp_config_, nlp_opts_, "rti_phase", &rti_phase);

  /* Set parameters */
  initParameters();

  /* Initialize output value */
  initPredXU(x_u_out_);
}

base_nmpc::BaseMPCSolver::~BaseMPCSolver()
{
  // 1. free solver
  int status = acados_free(acados_ocp_capsule_);
  if (status)
    std::cout << "qd_full_model_acados_free() returned status " << status << ".\n";

  // 2. free solver capsule
  status = acados_free_capsule(acados_ocp_capsule_);
  if (status)
    std::cout << "qd_full_model_acados_free_capsule() returned status " << status << ".\n";
}

void base_nmpc::BaseMPCSolver::reset(const aerial_robot_msgs::PredXU& x_u)
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

int base_nmpc::BaseMPCSolver::solve(const nav_msgs::Odometry& odom_now, const aerial_robot_msgs::PredXU& x_u_ref)
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

  return 0;
}

void base_nmpc::BaseMPCSolver::initPredXU(aerial_robot_msgs::PredXU& x_u) const
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
}

void base_nmpc::BaseMPCSolver::setReference(const aerial_robot_msgs::PredXU& x_u_ref, const unsigned int x_stride,
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
    acados_update_params_sparse(acados_ocp_capsule_, i, qr_idx, qr, 4);
  }
  // final x and p, no u
  double xr[NX_];
  std::copy(x_u_ref.x.data.begin() + x_stride * NN_, x_u_ref.x.data.begin() + x_stride * (NN_ + 1), xr);
  ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, NN_, "y_ref", xr);

  std::copy(x_u_ref.x.data.begin() + x_stride * NN_ + 6, x_u_ref.x.data.begin() + x_stride * NN_ + 10, qr);
  acados_update_params_sparse(acados_ocp_capsule_, NN_, qr_idx, qr, 4);
}

void base_nmpc::BaseMPCSolver::setFeedbackConstraints(const nav_msgs::Odometry& odom_now)
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

double base_nmpc::BaseMPCSolver::solveOCPInLoop(const int N_timings)
{
  double min_time = 1e12;
  double elapsed_time;

  for (int i = 0; i < N_timings; i++)
  {
    int status = acados_solve(acados_ocp_capsule_);
    if (status != ACADOS_SUCCESS)
    {
      std::cout << "qd_full_model_acados_solve() returned status " << status << ".\n";
    }

    ocp_nlp_get(nlp_config_, nlp_solver_, "time_tot", &elapsed_time);
    min_time = MIN(elapsed_time, min_time);
  }

  return min_time;
}

void base_nmpc::BaseMPCSolver::getSolution(const unsigned int x_stride, const unsigned int u_stride)
{
  for (int i = 0; i < NN_; i++)
  {
    ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, i, "x", x_u_out_.x.data.data() + x_stride * i);
    ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, i, "u", x_u_out_.u.data.data() + u_stride * i);
  }
  ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, NN_, "x", x_u_out_.x.data.data() + x_stride * NN_);
}

void base_nmpc::BaseMPCSolver::printSolution()
{
  std::cout << "\n--- x_traj ---\n" << std::endl;

  for (int i = 0; i <= NN_; i++)
  {
    std::cout << "X Row " << i << ":" << std::endl;
    for (int j = 0; j < NX_; j++)
    {
      int index = i * NX_ + j;
      std::cout << x_u_out_.x.data[index] << " ";
    }
    std::cout << std::endl;
  }

  std::cout << "\n--- u_traj ---\n" << std::endl;

  for (int i = 0; i < NN_; i++)
  {
    std::cout << "U Row " << i << ":" << std::endl;
    for (int j = 0; j < NU_; j++)
    {
      int index = i * NU_ + j;
      std::cout << x_u_out_.u.data[index] << " ";
    }
    std::cout << std::endl;
  }
}

void base_nmpc::BaseMPCSolver::printStatus(const int N_timings, const double min_time)
{
  double kkt_norm_inf;
  int sqp_iter;

  ROS_DEBUG("\nsolved ocp %d times, solution printed above\n\n", N_timings);

  ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, 0, "kkt_norm_inf", &kkt_norm_inf);
  ocp_nlp_get(nlp_config_, nlp_solver_, "sqp_iter", &sqp_iter);
  acados_print_stats(acados_ocp_capsule_);
  ROS_DEBUG("\nSolver info:\n");
  ROS_DEBUG(" SQP iterations %2d\n minimum time for %d solve %f [ms]\n KKT %e\n", sqp_iter, N_timings, min_time * 1000,
            kkt_norm_inf);
}
