//
// Created by lijinjie on 23/11/29.
//

#ifndef BEETLE_NMPC_SOLVER_H
#define BEETLE_NMPC_SOLVER_H

#endif  // BEETLE_NMPC_SOLVER_H

#include <ros/console.h>
#include <aerial_robot_msgs/PredXU.h>
#include <nav_msgs/Odometry.h>
#include <iostream>

// acados
#include "acados/utils/math.h"
#include "acados/utils/print.h"
#include "acados_c/external_function_interface.h"
#include "acados_c/ocp_nlp_interface.h"
#include "aerial_robot_control/nmpc/tilt_qd_servo_mdl/c_generated_code/acados_solver_tilt_qd_servo_mdl.h"

namespace aerial_robot_control
{

namespace nmpc_over_act_full
{

class MPCSolver
{
public:
  /* acados */
  // params
  int NN_, NX_, NZ_, NU_, NP_, NBX_, NBX0_, NBU_, NSBX_, NSBU_, NSH_, NSH0_, NSG_, NSPHI_, NSHN_, NSGN_, NSPHIN_,
      NSPHI0_, NSBXN_, NS_, NS0_, NSN_, NG_, NBXN_, NGN_, NY0_, NY_, NYN_, NH_, NHM_, NH0_, NPHI0_, NPHI_, NHN_, NPHIN_,
      NR_;

  // references
  std::vector<std::vector<double>> xr_;
  std::vector<std::vector<double>> ur_;

  // used for the output, also is (sub) optimal
  std::vector<std::vector<double>> xo_;
  std::vector<std::vector<double>> uo_;

  double* W_;
  double* WN_;

  MPCSolver()
  {
    // macro
    NN_ = TILT_QD_SERVO_MDL_N;
    NX_ = TILT_QD_SERVO_MDL_NX;
    NZ_ = TILT_QD_SERVO_MDL_NZ;
    NU_ = TILT_QD_SERVO_MDL_NU;
    NP_ = TILT_QD_SERVO_MDL_NP;
    NBX_ = TILT_QD_SERVO_MDL_NBX;
    NBX0_ = TILT_QD_SERVO_MDL_NBX0;
    NBU_ = TILT_QD_SERVO_MDL_NBU;
    NSBX_ = TILT_QD_SERVO_MDL_NSBX;
    NSBU_ = TILT_QD_SERVO_MDL_NSBU;
    NSH_ = TILT_QD_SERVO_MDL_NSH;
    NSH0_ = TILT_QD_SERVO_MDL_NSH0;
    NSG_ = TILT_QD_SERVO_MDL_NSG;
    NSPHI_ = TILT_QD_SERVO_MDL_NSPHI;
    NSHN_ = TILT_QD_SERVO_MDL_NSHN;
    NSGN_ = TILT_QD_SERVO_MDL_NSGN;
    NSPHIN_ = TILT_QD_SERVO_MDL_NSPHIN;
    NSPHI0_ = TILT_QD_SERVO_MDL_NSPHI0;
    NSBXN_ = TILT_QD_SERVO_MDL_NSBXN;
    NS_ = TILT_QD_SERVO_MDL_NS;
    NS0_ = TILT_QD_SERVO_MDL_NS0;
    NSN_ = TILT_QD_SERVO_MDL_NSN;
    NG_ = TILT_QD_SERVO_MDL_NG;
    NBXN_ = TILT_QD_SERVO_MDL_NBXN;
    NGN_ = TILT_QD_SERVO_MDL_NGN;
    NY0_ = TILT_QD_SERVO_MDL_NY0;
    NY_ = TILT_QD_SERVO_MDL_NY;
    NYN_ = TILT_QD_SERVO_MDL_NYN;
    NH_ = TILT_QD_SERVO_MDL_NH;
    NHM_ = TILT_QD_SERVO_MDL_NHN;
    NH0_ = TILT_QD_SERVO_MDL_NH0;
    NPHI0_ = TILT_QD_SERVO_MDL_NPHI0;
    NPHI_ = TILT_QD_SERVO_MDL_NPHI;
    NPHIN_ = TILT_QD_SERVO_MDL_NPHIN;
    NR_ = TILT_QD_SERVO_MDL_NR;

    // acados functions that only using once
    acados_ocp_capsule_ = tilt_qd_servo_mdl_acados_create_capsule();

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

    // initialization that has no relation to acados
    xr_ = std::vector<std::vector<double>>(NN_ + 1, std::vector<double>(NX_, 0));
    ur_ = std::vector<std::vector<double>>(NN_, std::vector<double>(NU_, 0));

    xo_ = std::vector<std::vector<double>>(NN_ + 1, std::vector<double>(NX_, 0));
    uo_ = std::vector<std::vector<double>>(NN_, std::vector<double>(NU_, 0));
  };

  // acados functions that using multiple times
  inline int acados_update_params(int stage, double* value)
  {
    return tilt_qd_servo_mdl_acados_update_params(acados_ocp_capsule_, stage, value, NP_);
  }

  inline int acados_update_params_sparse(int stage, int* idx, double* p, int n_update)
  {
    return tilt_qd_servo_mdl_acados_update_params_sparse(acados_ocp_capsule_, stage, idx, p, n_update);
  }

  inline int acados_solve()
  {
    return tilt_qd_servo_mdl_acados_solve(acados_ocp_capsule_);
  }

  inline void acados_print_stats()
  {
    tilt_qd_servo_mdl_acados_print_stats(acados_ocp_capsule_);
  }

  ~MPCSolver()
  {
    // 1. free solver
    int status = tilt_qd_servo_mdl_acados_free(acados_ocp_capsule_);
    if (status)
      ROS_WARN("tilt_qd_servo_mdl_acados_free() returned status %d. \n", status);

    // 2. free solver capsule
    status = tilt_qd_servo_mdl_acados_free_capsule(acados_ocp_capsule_);
    if (status)
      ROS_WARN("tilt_qd_servo_mdl_acados_free_capsule() returned status %d. \n", status);
  };

  void initialize();

  void reset(const std::vector<std::vector<double>>& x_init, const std::vector<std::vector<double>>& u_init)
  {
    if (x_init.size() != NN_ + 1 || u_init.size() != NN_)
      throw std::invalid_argument("x_init or u_init size is not equal to NN_ + 1 or NN_");

    for (int i = 0; i < NN_; i++)
    {
      if (x_init[i].size() != NX_ || u_init[i].size() != NU_)
        throw std::invalid_argument("x_init[i] or u_init[i] size is not equal to NX_ or NU_");
      ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, i, "x", (void*)x_init[i].data());
      ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, i, "u", (void*)u_init[i].data());
    }
    if (x_init[NN_].size() != NX_)
      throw std::invalid_argument("x_init[NN_] size is not equal to NX_");
    ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, NN_, "x", (void*)x_init[NN_].data());
  }

  void reset_w_x0_u0(const std::vector<double>& x0, const std::vector<double>& u0)
  {
    if (x0.size() != NX_ || u0.size() != NU_)
      throw std::invalid_argument("x0 or u0 size is not equal to NX_ or NU_");

    std::vector<std::vector<double>> x_init(NN_ + 1, x0);
    std::vector<std::vector<double>> u_init(NN_, u0);
    reset(x_init, u_init);
  }

  void setReference(const std::vector<std::vector<double>>& xr, const std::vector<std::vector<double>>& ur,
                    bool is_set_quat = false)
  {
    if (xr.size() != NN_ + 1 || ur.size() != NN_)
      throw std::invalid_argument("xr or ur size is not equal to NN_ + 1 or NN_");

    for (int i = 0; i < NN_; i++)
    {
      if (xr[i].size() != NX_ || ur[i].size() != NU_)
        throw std::invalid_argument("xr[i] or ur[i] size is not equal to NX_ or NU_");

      std::vector<double> yr;
      yr.reserve(xr[i].size() + ur[i].size());
      yr.insert(yr.end(), xr[i].begin(), xr[i].end());
      yr.insert(yr.end(), ur[i].begin(), ur[i].end());

      ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, i, "y_ref", (void*)yr.data());

      if (is_set_quat)
      {
        int qr_idx[] = { 0, 1, 2, 3 };
        std::vector<double> qr;
        qr.reserve(4);
        qr.insert(qr.end(), xr[i].begin() + 6, xr[i].begin() + 10);
        acados_update_params_sparse(i, qr_idx, qr.data(), 4);
      }
    }

    if (xr[NN_].size() != NX_)
      throw std::invalid_argument("xr[NN_] size is not equal to NX_");

    ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, NN_, "y_ref", (void*)xr[NN_].data());

    if (is_set_quat)
    {
      int qr_idx[] = { 0, 1, 2, 3 };
      std::vector<double> qr;
      qr.reserve(4);
      qr.insert(qr.end(), xr[NN_].begin() + 6, xr[NN_].begin() + 10);
      acados_update_params_sparse(NN_, qr_idx, qr.data(), 4);
    }

    xr_ = xr;
    ur_ = ur;
  }

  int solve(const std::vector<double>& bx0, const bool is_debug = false)
  {
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

  /* Setters */
  void setCostWDiagElement(int index, double value, bool is_set_WN = true) const;
  void setCostWeight(bool is_update_W, bool is_update_WN);

  /* for debugging */
  void printStatus(double min_time)
  {
    double kkt_norm_inf;
    int sqp_iter;

    ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, 0, "kkt_norm_inf", &kkt_norm_inf);
    ocp_nlp_get(nlp_config_, nlp_solver_, "sqp_iter", &sqp_iter);
    acados_print_stats();
    ROS_DEBUG("\nSolver info:\n");
    ROS_DEBUG(" SQP iterations %2d\n minimum time for 1 solve %f [ms]\n KKT %e\n", sqp_iter, min_time * 1000,
              kkt_norm_inf);
  }

  void printSolution()
  {
    std::stringstream ss;

    ss << "\n--- x_traj ---\n";
    for (int i = 0; i <= NN_; i++)
    {
      ss << "X Row " << i << ":\n";
      for (int j = 0; j < NX_; j++)
      {
        ss << xo_[i][j] << " ";
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
        ss << uo_[i][j] << " ";
      }
      ss << "\n";
    }
    ROS_INFO_STREAM(ss.str());  // Logging the u_traj
  }

protected:
  tilt_qd_servo_mdl_solver_capsule* acados_ocp_capsule_ = nullptr;

  double* new_time_steps = nullptr;
  ocp_nlp_config* nlp_config_ = nullptr;
  ocp_nlp_dims* nlp_dims_ = nullptr;
  ocp_nlp_in* nlp_in_ = nullptr;
  ocp_nlp_out* nlp_out_ = nullptr;
  ocp_nlp_solver* nlp_solver_ = nullptr;
  void* nlp_opts_ = nullptr;

  inline void setFeedbackConstraints(const std::vector<double>& bx0)
  {
    if (bx0.size() != NBX0_)
      throw std::invalid_argument("bx0 size is not equal to NBX0_");

    ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, 0, "lbx", (void*)bx0.data());
    ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, 0, "ubx", (void*)bx0.data());
  }

  inline double solveOCPOnce()
  {
    double min_time = 1e12;
    double elapsed_time;

    int status = acados_solve();
    if (status != ACADOS_SUCCESS)
    {
      ROS_WARN("acados_solve() returned status %d.\n", status);
    }

    ocp_nlp_get(nlp_config_, nlp_solver_, "time_tot", &elapsed_time);
    min_time = MIN(elapsed_time, min_time);

    return min_time;
  }

  inline void getSolution()
  {
    if (xo_.size() != NN_ + 1 || uo_.size() != NN_)
      throw std::invalid_argument("xo_ or uo_ size is not equal to NN_ + 1 or NN_");

    for (int i = 0; i < NN_; i++)
    {
      if (xo_[i].size() != NX_ || uo_[i].size() != NU_)
        throw std::invalid_argument("xo_[i] or uo_[i] size is not equal to NX_ or NU_");

      ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, i, "x", xo_[i].data());
      ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, i, "u", uo_[i].data());
    }

    if (xo_[NN_].size() != NX_)
      throw std::invalid_argument("xo_[NN_] size is not equal to NX_");

    ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, NN_, "x", xo_[NN_].data());
  }
};

}  // namespace nmpc_over_act_full

}  // namespace aerial_robot_control
