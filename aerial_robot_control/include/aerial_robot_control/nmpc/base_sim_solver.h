//
// Created by li-jinjie on 24-07-19.
//

#ifndef BASE_SIM_SOLVER_H
#define BASE_SIM_SOLVER_H

#include <stdexcept>
#include <sstream>
#include <vector>
#include <iostream>

#include "acados_c/sim_interface.h"

namespace aerial_robot_control
{

namespace mpc_sim_solver
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

class BaseSimSolver
{
public:
  // acados params
  int NX_, NZ_, NU_, NP_;

  std::vector<double> xo_;

  BaseSimSolver() = default;           // should be overridden by the derived class
  virtual ~BaseSimSolver() = default;  // if the class has virtual functions, then the destructor should be virtual, but
                                       // not pure virtual.

  void initialize(double T)
  {
    setT(T);

    xo_ = std::vector<double>(NX_, 0);

    std::vector<double> p(NP_, 0);
    setParameters(p);
  };

  int solve(std::vector<double>& x, std::vector<double>& u, const bool is_debug = false)
  {
    if (x.size() != NX_ || u.size() != NU_)
      throw std::length_error("x or u size is not equal to NX_ or NU_");

    sim_solver_set(sim_solver_, "x", x.data());
    sim_solver_set(sim_solver_, "u", u.data());

    double min_time = solveSimOnce();

    getSolution();

    if (is_debug)
    {
      printAcadosMatrix();
      printAcadosSolution();
      printAcadosStatus(min_time);
    }

    return 0;
  }

  /* Setters */
  void setT(double& T)
  {
    sim_solver_set(sim_solver_, "T", &T);
  }

  void setParameters(std::vector<double>& p)
  {
    if (p.size() != NP_)
      throw std::length_error("p size is not equal to NP_");

    sim_solver_set(sim_solver_, "p", p.data());
  }

  /* Getters */
  std::vector<double> getMatrixA()
  {
    std::vector<double> mtx_A(NX_ * NX_);
    sim_out_get(sim_config_, sim_dims_, sim_out_, "Sx", mtx_A.data());
    return mtx_A;
  }

  std::vector<double> getMatrixB()
  {
    std::vector<double> mtx_B(NX_ * NU_);
    sim_out_get(sim_config_, sim_dims_, sim_out_, "Su", mtx_B.data());
    return mtx_B;
  }

  /* for debugging */
  void printAcadosStatus(double min_time)
  {
    std::cout << "\nSolver info:\n";
    std::cout << "minimum time for 1 solve " << min_time * 1000 << " [ms] " << std::endl;
  }

  void printAcadosMatrix(int stage = 0)
  {
    std::vector<double> mtx_A = getMatrixA();
    std::cout << "A matrix at stage " << stage << ":\n";
    for (int i = 0; i < NX_; i++)
    {
      for (int j = 0; j < NX_; j++)
      {
        std::cout << mtx_A[i * NX_ + j] << " ";
      }
      std::cout << "\n";
    }
  }

  void printAcadosSolution()
  {
    std::stringstream ss;

    ss << "\n--- new_x ---\n";
    for (int i = 0; i < NX_; i++)
    {
      ss << xo_[i] << " ";
    }
    ss << "\n";

    std::cout << ss.str();  // Logging the x_traj
  }

protected:
  sim_config* sim_config_ = nullptr;
  void* sim_dims_ = nullptr;
  sim_in* sim_in_ = nullptr;
  sim_out* sim_out_ = nullptr;
  sim_solver* sim_solver_ = nullptr;
  void* sim_opts_ = nullptr;

  inline double solveSimOnce()
  {
    double min_time = 1e12;
    double elapsed_time;

    int status = acadosSolve();
    if (status != ACADOS_SUCCESS)
      throw AcadosSolveException(status);

    sim_out_get(sim_config_, sim_dims_, sim_out_, "time_tot", &elapsed_time);
    min_time = MIN(elapsed_time, min_time);

    return min_time;
  }

  inline void getSolution()
  {
    if (xo_.size() != NX_)
      throw std::length_error("xo size is not equal to NX_");

    sim_out_get(sim_config_, sim_dims_, sim_out_, "x", xo_.data());
  }

  // acados functions that using multiple times
  virtual inline int acadosSolve() = 0;
};

}  // namespace mpc_sim_solver

}  // namespace aerial_robot_control

#endif  // BASE_SIM_SOLVER_H