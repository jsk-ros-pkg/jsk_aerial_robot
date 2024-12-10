//
// Created by li-jinjie on 24-12-09.
//

#ifndef BASE_MHE_SOLVER_H
#define BASE_MHE_SOLVER_H

#include <stdexcept>
#include <sstream>
#include <vector>
#include <iostream>

#include "acados/utils/math.h"
#include "acados/utils/print.h"
#include "acados_c/external_function_interface.h"
#include "acados_c/ocp_nlp_interface.h"

namespace aerial_robot_control
{

namespace mhe_solver
{

class AcadosSolveException : public std::runtime_error
{
public:
  int status_;
  explicit AcadosSolveException(int status) : std::runtime_error(createErrorMessage(status)), status_(status) {};

private:
  static std::string createErrorMessage(int status)
  {
    std::ostringstream oss;
    oss << "acados returned status " << status << ". Exiting.";
    return oss.str();
  }
};

class BaseMHESolver
{
public:
  // acados params
  int NN_, NX_, NZ_, NU_, NP_, NBX_, NBX0_, NBU_, NSBX_, NSBU_, NSH_, NSH0_, NSG_, NSPHI_, NSHN_, NSGN_, NSPHIN_,
      NSPHI0_, NSBXN_, NS_, NS0_, NSN_, NG_, NBXN_, NGN_, NY0_, NY_, NYN_, NH_, NH0_, NPHI0_, NPHI_, NHN_, NPHIN_, NR_;

  int NM_;  // number of measurements, only for MHE

  // for cost function
  std::vector<double> x0_bar_;
  std::vector<std::vector<double>> meas_;
  std::vector<std::vector<double>> ctrl_;

  // outputs, also are (sub) optimal
  std::vector<std::vector<double>> xo_;
  std::vector<std::vector<double>> uo_;

  std::vector<double> W_;
  std::vector<double> WN_;

  BaseMHESolver() = default;           // should be overridden by the derived class
  virtual ~BaseMHESolver() = default;  // if the class has virtual functions, then the destructor should be virtual,
                                       // but not pure virtual.

  void initialize()
  {
    meas_ = std::vector<std::vector<double>>(NN_ + 1, std::vector<double>(NM_, 0));
    x0_bar_ = std::vector<double>(NX_, 0);

    if (NP_ > 0)
      ctrl_ = std::vector<std::vector<double>>(NN_ + 1, std::vector<double>(NP_, 0));

    xo_ = std::vector<std::vector<double>>(NN_ + 1, std::vector<double>(NX_, 0));
    uo_ = std::vector<std::vector<double>>(NN_, std::vector<double>(NU_, 0));

    W_ = std::vector<double>(NY_ * NY_, 0);   // NY = NX + NU
    WN_ = std::vector<double>(NX_ * NX_, 0);  // WN has the same size as NX

    setRTIPhase();
  };

  void reset()
  {
    meas_ = std::vector<std::vector<double>>(NN_ + 1, std::vector<double>(NM_, 0));
    x0_bar_ = std::vector<double>(NX_, 0);

    if (NP_ > 0)
      ctrl_ = std::vector<std::vector<double>>(NN_ + 1, std::vector<double>(NP_, 0));

    xo_ = std::vector<std::vector<double>>(NN_ + 1, std::vector<double>(NX_, 0));
    uo_ = std::vector<std::vector<double>>(NN_, std::vector<double>(NU_, 0));

    // reset solver
    for (int i = 0; i < NN_; i++)
    {
      ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, i, "x", (void*)xo_[i].data());
      ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, i, "u", (void*)uo_[i].data());
    }
    ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, NN_, "x", (void*)xo_[NN_].data());

    // update xo_, uo_
    getSolution();
  }

  int solve(const bool is_debug = false)
  {
    double min_time = solveOCPOnce();

    getSolution();

    if (is_debug)
    {
      printAcadosWeight();
      printAcadosSolution();
      printAcadosStatus(min_time);
    }

    // update x0_bar_
    std::copy(xo_[1].begin(), xo_[1].end(), x0_bar_.begin());

    return 0;
  }

  /* Getters */
  std::vector<double> getEstimatedState(int idx)
  {
    if (idx < 0 || idx > NN_)
      throw std::out_of_range("idx is out of range");

    return xo_[idx];
  }

  /* Setters */
  void setRTIPhase(int rti_phase = 0)  //  (1) preparation, (2) feedback, (0) both.
  {
    ocp_nlp_solver_opts_set(nlp_config_, nlp_opts_, "rti_phase", &rti_phase);
  }

  void setMeasurement(const std::vector<double>& meas, const std::vector<double>& control = {})
  {
    if (meas.size() != NM_)
      throw std::length_error("meas size is not equal to NM_");

    if (control.size() != NP_)
      throw std::length_error("control size is not equal to NP_");

    // step 1 & 2: shift the measurements and control
    for (int i = 0; i < NN_; i++)
    {
      meas_[i] = meas_[i + 1];
      ctrl_[i] = ctrl_[i + 1];
    }

    meas_[NN_] = meas;
    ctrl_[NN_] = control;

    // step 3: prepare noise_ref
    std::vector<double> noise = std::vector<double>(NU_, 0);

    // step 4: fill yr0 and p0
    std::vector<double> yr;
    yr.reserve(meas_[0].size() + noise.size() + x0_bar_.size());
    yr.insert(yr.end(), meas_[0].begin(), meas_[0].end());
    yr.insert(yr.end(), noise.begin(), noise.end());
    yr.insert(yr.end(), x0_bar_.begin(), x0_bar_.end());
    ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, 0, "y_ref", (void*)yr.data());

    acadosUpdateParams(0, ctrl_[0]);

    // step 5: fill yr 1~N
    for (int i = 1; i < NN_; i++)
    {
      yr.clear();
      yr.reserve(meas_[i].size() + noise.size());
      yr.insert(yr.end(), meas_[i].begin(), meas_[i].end());
      yr.insert(yr.end(), noise.begin(), noise.end());
      ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, i, "y_ref", (void*)yr.data());

      acadosUpdateParams(i, ctrl_[i]);
    }

    // step 6: fill yrN
    ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, NN_, "y_ref", (void*)meas_[NN_].data());

    acadosUpdateParams(NN_, ctrl_[NN_]);
  }

  /* for debugging */
  void printAcadosStatus(double min_time)
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

  void printAcadosWeight()
  {
    std::cout << "W matrix:\n";
    for (int i = 0; i < NY_; i++)
    {
      for (int j = 0; j < NY_; j++)
      {
        std::cout << W_[i * NY_ + j] << " ";
      }
      std::cout << "\n";
    }

    std::cout << "WN matrix:\n";
    for (int i = 0; i < NX_; i++)
    {
      for (int j = 0; j < NX_; j++)
      {
        std::cout << WN_[i * NX_ + j] << " ";
      }
      std::cout << "\n";
    }
  }

  void printAcadosSolution()
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

    ss.str("");  // Clearing the string stream

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
  ocp_nlp_config* nlp_config_ = nullptr;
  ocp_nlp_dims* nlp_dims_ = nullptr;
  ocp_nlp_in* nlp_in_ = nullptr;
  ocp_nlp_out* nlp_out_ = nullptr;
  ocp_nlp_solver* nlp_solver_ = nullptr;
  void* nlp_opts_ = nullptr;

  void setCostWeightMid(std::vector<double> W)  // if you want to make this function public, make sure W_ = W
  {
    if (W.size() != NY_ * NY_)
      throw std::length_error("W size is not equal to NY_ * NY_, please check.");

    for (int i = 0; i < NN_; i++)
      ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, i, "W", W.data());
  }

  void setCostWeightEnd(std::vector<double> WN)  // if you want to make this function public, make sure WN_ = WN
  {
    if (WN.size() != NX_ * NX_)
      throw std::length_error("W size is not equal to NX_ * NX_, please check.");

    ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, NN_, "W", WN.data());
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
      throw std::length_error("xo_ or uo_ size is not equal to NN_ + 1 or NN_");

    for (int i = 0; i < NN_; i++)
    {
      if (xo_[i].size() != NX_ || uo_[i].size() != NU_)
        throw std::length_error("xo_[i] or uo_[i] size is not equal to NX_ or NU_");

      ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, i, "x", xo_[i].data());
      ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, i, "u", uo_[i].data());
    }

    if (xo_[NN_].size() != NX_)
      throw std::length_error("xo_[NN_] size is not equal to NX_");

    ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, NN_, "x", xo_[NN_].data());
  }

  // acados functions that using multiple times
  virtual inline int acadosUpdateParams(int stage, std::vector<double>& value) = 0;

  virtual inline int acadosUpdateParamsSparse(int stage, std::vector<int>& idx, std::vector<double>& p,
                                              int n_update) = 0;

  virtual inline int acadosSolve() = 0;

  virtual inline void acadosPrintStats() = 0;
};

}  // namespace mhe_solver

}  // namespace aerial_robot_control

#endif  // BASE_MHE_SOLVER_H