//
// Created by li-jinjie on 24-12-09.
//

#ifndef BASE_MHE_SOLVER_H
#define BASE_MHE_SOLVER_H

#include "aerial_robot_control/nmpc/base_mpc_solver.h"

namespace aerial_robot_control
{

namespace mhe_solver
{

class BaseMHESolver : public mpc_solver::BaseMPCSolver
{
public:
  int NM_;  // number of measurements, only for MHE

  std::vector<double> x0_bar_;
  std::vector<std::vector<double>> meas_;
  std::vector<std::vector<double>> noise_;

  BaseMHESolver() = default;           // should be overridden by the derived class
  virtual ~BaseMHESolver() = default;  // if the class has virtual functions, then the destructor should be virtual, but
                                       // not pure virtual.

  void initialize(bool have_quat = false)
  {
    BaseMPCSolver::initialize(have_quat);

    meas_ = std::vector<std::vector<double>>(NN_ + 1, std::vector<double>(NM_, 0));
    noise_ = std::vector<std::vector<double>>(NN_, std::vector<double>(NU_, 0));
    x0_bar_ = std::vector<double>(NX_, 0);
  };

  int solve(const bool is_debug = false)
  {
    double min_time = solveOCPOnce();

    getSolution();

    if (is_debug)
    {
      printAcadosWeight();
      printAcadosMatrix();
      printAcadosReference();
      printAcadosSolution();
      printAcadosStatus(min_time);
    }

    return 0;
  }

  /* Setters */
  void setMeasurement(const std::vector<std::vector<double>>& meas, const std::vector<std::vector<double>>& noise,
                      std::vector<double> x0_bar)
  {
    if (meas.size() != NN_ + 1 || noise.size() != NN_)
      throw std::length_error("meas or noise size is not equal to NN_ + 1 or NN_");

    meas_ = meas;
    noise_ = noise;
    x0_bar_ = x0_bar;

    // i = 0
    std::vector<double> yr;
    yr.reserve(meas[0].size() + noise[0].size() + x0_bar.size());
    yr.insert(yr.end(), meas[0].begin(), meas[0].end());
    yr.insert(yr.end(), noise[0].begin(), noise[0].end());
    yr.insert(yr.end(), x0_bar.begin(), x0_bar.end());
    ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, 0, "y_ref", (void*)yr.data());

    for (int i = 1; i < NN_; i++)
    {
      yr.clear();
      yr.reserve(meas[i].size() + noise[i].size());
      yr.insert(yr.end(), meas[i].begin(), meas[i].end());
      yr.insert(yr.end(), noise[i].begin(), noise[i].end());

      ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, i, "y_ref", (void*)yr.data());
    }

    ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, NN_, "y_ref", (void*)meas[NN_].data());
  }

protected:
};

}  // namespace mpc_solver

}  // namespace aerial_robot_control

#endif  // BASE_MHE_SOLVER_H