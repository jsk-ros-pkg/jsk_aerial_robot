//
// Created by li-jinjie on 23-11-25.
//

#ifndef NMPC_BASE_MPC_SOLVER_H
#define NMPC_BASE_MPC_SOLVER_H

#endif  // NMPC_BASE_MPC_SOLVER_H

// acados
#include "acados/utils/math.h"
#include "acados/utils/print.h"
#include "acados_c/external_function_interface.h"
#include "acados_c/ocp_nlp_interface.h"

namespace aerial_robot_control
{

namespace nmpc
{

class AcadosSolveException : public std::runtime_error
{
public:
  int status_;
  explicit AcadosSolveException(int status) : std::runtime_error(createErrorMessage(status)), status_(status){};

private:
  static std::string createErrorMessage(int status)
  {
    std::ostringstream oss;
    oss << "acados returned status " << status << ". Exiting.";
    return oss.str();
  }
};

class BaseMPCSolver
{
public:
  // acados params
  int NN_, NX_, NZ_, NU_, NP_, NBX_, NBX0_, NBU_, NSBX_, NSBU_, NSH_, NSH0_, NSG_, NSPHI_, NSHN_, NSGN_, NSPHIN_,
      NSPHI0_, NSBXN_, NS_, NS0_, NSN_, NG_, NBXN_, NGN_, NY0_, NY_, NYN_, NH_, NH0_, NPHI0_, NPHI_, NHN_, NPHIN_, NR_;

  // references
  std::vector<std::vector<double>> xr_;
  std::vector<std::vector<double>> ur_;

  // outputs, also are (sub) optimal
  std::vector<std::vector<double>> xo_;
  std::vector<std::vector<double>> uo_;

  std::vector<double> W_;
  std::vector<double> WN_;

  BaseMPCSolver(){};   // should be overrided by the derived class
  ~BaseMPCSolver(){};  // should be overrided by the derived class

  void initialize()
  {
    xr_ = std::vector<std::vector<double>>(NN_ + 1, std::vector<double>(NX_, 0));
    ur_ = std::vector<std::vector<double>>(NN_, std::vector<double>(NU_, 0));

    xo_ = std::vector<std::vector<double>>(NN_ + 1, std::vector<double>(NX_, 0));
    uo_ = std::vector<std::vector<double>>(NN_, std::vector<double>(NU_, 0));

    W_ = std::vector<double>(NY_ * NY_, 0);   // NY = NX + NU
    WN_ = std::vector<double>(NX_ * NX_, 0);  // WN has the same size as NX

    setRTIPhase();

    std::vector<double> p(NP_, 0);
    p[0] = 1.0;  // quaternion
    setParameters(p);
  };

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

  void resetByX0U0(const std::vector<double>& x0, const std::vector<double>& u0)
  {
    if (x0.size() != NX_ || u0.size() != NU_)
      throw std::invalid_argument("x0 or u0 size is not equal to NX_ or NU_");

    std::vector<std::vector<double>> x_init(NN_ + 1, x0);
    std::vector<std::vector<double>> u_init(NN_, u0);
    reset(x_init, u_init);
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
  void setRTIPhase(int rti_phase = 0)  //  (1) preparation, (2) feedback, (0) both.
  {
    ocp_nlp_solver_opts_set(nlp_config_, nlp_opts_, "rti_phase", &rti_phase);
  }

  void setParameters(std::vector<double>& p)
  {
    if (p.size() != NP_)
      throw std::invalid_argument("p size is not equal to NP_");

    for (int i = 0; i < NN_ + 1; i++)
      acadosUpdateParams(i, p);
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
        acadosUpdateParamsSparse(i, qr_idx, qr.data(), 4);
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
      acadosUpdateParamsSparse(NN_, qr_idx, qr.data(), 4);
    }

    xr_ = xr;
    ur_ = ur;
  }

  void setCostWDiagElement(int index, double value, bool is_set_WN = true)
  {
    if (index >= NY_)
      throw std::invalid_argument("index should be less than NY_ = NX_ + NU_");

    W_[index + index * (NY_)] = value;

    if (is_set_WN)
    {
      if (index >= NX_)
        throw std::invalid_argument("index should be less than NX_");

      WN_[index + index * NX_] = value;
    }
  }

  void setCostWeight(bool is_update_W, bool is_update_WN)
  {
    if (is_update_W)
    {
      for (int i = 0; i < NN_; i++)
        ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, i, "W", W_.data());
    }
    if (is_update_WN)
      ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, NN_, "W", WN_.data());
  }

  /* for debugging */
  void printStatus(double min_time)
  {
    double kkt_norm_inf;
    int sqp_iter;

    ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, 0, "kkt_norm_inf", &kkt_norm_inf);
    ocp_nlp_get(nlp_config_, nlp_solver_, "sqp_iter", &sqp_iter);

    acadosPrintStats();

    std::cout << "\nSolver info:\n";
    std::cout << " SQP iterations " << sqp_iter << "\n minimum time for 1 solve " << min_time * 1000 << " [ms]\n KKT "
              << kkt_norm_inf << std::endl;
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
    std::cout << ss.str();  // Logging the x_traj

    ss.str("");  // Clearing the stringstream

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
    std::cout << ss.str();  // Logging the u_traj
  }

protected:
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

    int status = acadosSolve();
    if (status != ACADOS_SUCCESS)
      throw AcadosSolveException(status);

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

  // acados functions that using multiple times
  virtual inline int acadosUpdateParams(int stage, std::vector<double>& value) = 0;

  virtual inline int acadosUpdateParamsSparse(int stage, int* idx, double* p, int n_update) = 0;

  virtual inline int acadosSolve() = 0;

  virtual inline void acadosPrintStats() = 0;
};

}  // namespace nmpc

}  // namespace aerial_robot_control
