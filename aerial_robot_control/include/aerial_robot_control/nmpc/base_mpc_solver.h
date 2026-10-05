//
// Created by li-jinjie on 23-11-25.
//

#ifndef BASE_MPC_SOLVER_H
#define BASE_MPC_SOLVER_H

#include <stdexcept>
#include <sstream>
#include <vector>
#include <iostream>
#include <numeric>

#include "acados/utils/math.h"
#include "acados/utils/print.h"
#include "acados_c/external_function_interface.h"
#include "acados_c/ocp_nlp_interface.h"

namespace aerial_robot_control
{

namespace mpc_solver
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
    oss << "acados returned status " << status << ". Exiting." << std::endl;
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

  // weights
  std::vector<double> W_;
  std::vector<double> WN_;

  BaseMPCSolver() = default;           // should be overridden by the derived class
  virtual ~BaseMPCSolver() = default;  // if the class has virtual functions, then the destructor should be virtual, but
                                       // not pure virtual.

  void initialize(bool have_quat = true)
  {
    xr_ = std::vector<std::vector<double>>(NN_ + 1, std::vector<double>(NX_, 0));
    ur_ = std::vector<std::vector<double>>(NN_, std::vector<double>(NU_, 0));

    xo_ = std::vector<std::vector<double>>(NN_ + 1, std::vector<double>(NX_, 0));
    uo_ = std::vector<std::vector<double>>(NN_, std::vector<double>(NU_, 0));

    W_ = std::vector<double>(NY_ * NY_, 0);   // NY = NX + NU
    WN_ = std::vector<double>(NX_ * NX_, 0);  // WN has the same size as NX

    setRTIPhase();
  };

  /* Note: extract resetXrUr out of resetSolver since the xr & ur might be different from x & u in the solver.
   * For example, for EE-centric NMPC, the xr & ur are in the EE frame but x & u are in the CoG frame. */
  void resetXrUr(const std::vector<std::vector<double>>& xr_init, const std::vector<std::vector<double>>& ur_init)
  {
    xr_ = xr_init;
    ur_ = ur_init;
  }

  void resetSolver(const std::vector<std::vector<double>>& x_init, const std::vector<std::vector<double>>& u_init)
  {
    // reset solver
    if (x_init.size() != NN_ + 1 || u_init.size() != NN_)
      throw std::length_error("x_init or u_init size is not equal to NN_ + 1 or NN_");

    for (int i = 0; i < NN_; i++)
    {
      if (x_init[i].size() != NX_ || u_init[i].size() != NU_)
        throw std::length_error("x_init[i] or u_init[i] size is not equal to NX_ or NU_");
      ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, nlp_in_, i, "x", (void*)x_init[i].data());
      ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, nlp_in_, i, "u", (void*)u_init[i].data());
    }
    if (x_init[NN_].size() != NX_)
      throw std::length_error("x_init[NN_] size is not equal to NX_");
    ocp_nlp_out_set(nlp_config_, nlp_dims_, nlp_out_, nlp_in_, NN_, "x", (void*)x_init[NN_].data());

    // update xo_, uo_
    getSolution();
  }

  void resetXrUrByX0U0(const std::vector<double>& x0, const std::vector<double>& u0)
  {
    if (x0.size() != NX_ || u0.size() != NU_)
      throw std::length_error("x0 or u0 size is not equal to NX_ or NU_");

    std::vector xr_init(NN_ + 1, x0);
    std::vector ur_init(NN_, u0);

    resetXrUr(xr_init, ur_init);
  }

  void resetSolverByX0U0(const std::vector<double>& x0, const std::vector<double>& u0)
  {
    if (x0.size() != NX_ || u0.size() != NU_)
      throw std::length_error("x0 or u0 size is not equal to NX_ or NU_");

    std::vector x_init(NN_ + 1, x0);
    std::vector u_init(NN_, u0);
    resetSolver(x_init, u_init);
  }

  int solve(const std::vector<double>& bx0, const bool is_debug = false)
  {
    setFeedbackConstraints(bx0);

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
  void setRTIPhase(int rti_phase = 0)  //  (1) preparation, (2) feedback, (0) both.
  {
    ocp_nlp_solver_opts_set(nlp_config_, nlp_opts_, "rti_phase", &rti_phase);
  }

  /* after 2025-3-30, the p includes physical params so should be set values during activate().
   * Too early cannot get the correct values. */
  void setParameters(std::vector<double>& p, const int start_idx = 0)
  {
    if (start_idx + p.size() > NP_)
    {
      std::stringstream ss;
      ss << "start_idx + p size" << start_idx + p.size() << " is larger than NP_ = " << NP_;
      throw std::length_error(ss.str());
    }

    std::vector<int> index(p.size());
    std::iota(index.begin(), index.end(), start_idx);

    setParamSparseAllStages(index, p);
  }

  void setReference(const std::vector<std::vector<double>>& xr, const std::vector<std::vector<double>>& ur,
                    bool is_set_quat = true)
  {
    if (xr.size() != NN_ + 1 || ur.size() != NN_)
      throw std::length_error("xr or ur size is not equal to NN_ + 1 or NN_");

    for (int i = 0; i < NN_; i++)
    {
      if (xr[i].size() != NX_ || ur[i].size() != NU_)
        throw std::length_error("xr[i] or ur[i] size is not equal to NX_ or NU_");

      std::vector<double> yr;
      yr.reserve(xr[i].size() + ur[i].size());
      yr.insert(yr.end(), xr[i].begin(), xr[i].end());
      yr.insert(yr.end(), ur[i].begin(), ur[i].end());

      ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, i, "y_ref", (void*)yr.data());

      if (is_set_quat)
      {
        std::vector<int> qr_idx = { 0, 1, 2, 3 };
        std::vector<double> qr;
        qr.reserve(4);
        qr.insert(qr.end(), xr[i].begin() + 6, xr[i].begin() + 10);
        acadosUpdateParamsSparse(i, qr_idx, qr, 4);
      }
    }

    if (xr[NN_].size() != NX_)
      throw std::length_error("xr[NN_] size is not equal to NX_");

    ocp_nlp_cost_model_set(nlp_config_, nlp_dims_, nlp_in_, NN_, "y_ref", (void*)xr[NN_].data());

    if (is_set_quat)
    {
      std::vector<int> qr_idx = { 0, 1, 2, 3 };
      std::vector<double> qr;
      qr.reserve(4);
      qr.insert(qr.end(), xr[NN_].begin() + 6, xr[NN_].begin() + 10);
      acadosUpdateParamsSparse(NN_, qr_idx, qr, 4);
    }

    xr_ = xr;
    ur_ = ur;
  }

  void setCostWDiagElement(int index, double value, bool is_set_WN = true)
  {
    // For Q, is_set_WN should be true. For R, is_set_WN should be false.
    setCostWeightElement(index, index, value, is_set_WN);
  }

  void setCostWeightElement(int idx, int idy, double value, bool is_set_NN = true)
  {
    if (idx >= NY_ || idy >= NY_)
      throw std::length_error("idx or idy should be less than NY_ = NX_ + NU_");

    W_[idx + idy * NY_] = value;

    setCostWeightMid(W_);

    if (is_set_NN)
    {
      if (idx >= NX_ || idy >= NX_)
        throw std::length_error("idx or idy should be less than NX_");

      WN_[idx + idy * NX_] = value;

      setCostWeightEnd(WN_);
    }
  }

  // Note: the stage 0 should not be set by this function, as the first constraint is estimated states.
  void setConstraintsLbx(const std::vector<double>& lbx) const
  {
    if (lbx.size() != NBX_)
      throw std::length_error("lbx size is not equal to NBX_");

    for (int i = 1; i < NN_; i++)  // Note: 1 to NN_ - 1
      setConstraintsValue("lbx", i, lbx, false);
  }

  void setConstraintsLbxe(const std::vector<double>& lbx) const
  {
    if (lbx.size() != NBXN_)
      throw std::length_error("lbx size is not equal to NBXN_");

    setConstraintsValue("lbx", NN_, lbx, false);
  }

  void setConstraintsUbx(const std::vector<double>& ubx) const
  {
    if (ubx.size() != NBX_)
      throw std::length_error("ubx size is not equal to NBX_");

    for (int i = 1; i < NN_; i++)  // Note: 1 to NN_ - 1
      setConstraintsValue("ubx", i, ubx, false);
  }

  void setConstraintsUbxe(const std::vector<double>& ubx) const
  {
    if (ubx.size() != NBXN_)
      throw std::length_error("ubx size is not equal to NBXN_");

    setConstraintsValue("ubx", NN_, ubx, false);
  }

  void setConstraintsLbu(const std::vector<double>& lbu) const
  {
    if (lbu.size() != NBU_)
      throw std::length_error("lbu size is not equal to NBU_");

    for (int i = 0; i < NN_; i++)  // Note: for u, 0 to NN_ - 1. No NN_ for u.
      setConstraintsValue("lbu", i, lbu, false);
  }

  void setConstraintsUbu(const std::vector<double>& ubu) const
  {
    if (ubu.size() != NBU_)
      throw std::length_error("ubu size is not equal to NBU_");

    for (int i = 0; i < NN_; i++)  // Note: 1 to NN_ - 1
      setConstraintsValue("ubu", i, ubu, false);
  }

  // clang-format off
  std::vector<int> getConstraintsIdxbx() const { return getConstraintsIdx("idxbx", 1); }
  std::vector<int> getConstraintsIdxbxe() const { return getConstraintsIdx("idxbx", NN_); }
  std::vector<int> getConstraintsIdxbu() const { return getConstraintsIdx("idxbu", 1); }
  std::vector<int> getConstraintsIdxbue() const { return getConstraintsIdx("idxbu", NN_); }

  std::vector<double> getConstraintsLbx() const { return getConstraintsValue("lbx", 1); }
  std::vector<double> getConstraintsLbxe() const { return getConstraintsValue("lbx", NN_); }
  std::vector<double> getConstraintsUbx() const { return getConstraintsValue("ubx", 1); }
  std::vector<double> getConstraintsUbxe() const { return getConstraintsValue("ubx", NN_); }
  std::vector<double> getConstraintsLbu() const { return getConstraintsValue("lbu", 1); }
  std::vector<double> getConstraintsLbue() const { return getConstraintsValue("lbu", NN_); }
  std::vector<double> getConstraintsUbu() const { return getConstraintsValue("ubu", 1); }
  std::vector<double> getConstraintsUbue() const { return getConstraintsValue("ubu", NN_); }
  // clang-format on

  /* Getters */
  std::vector<double> getMatrixA(int stage)
  {
    std::vector<double> mtx_A(NX_ * NX_);
    ocp_nlp_get_at_stage(nlp_solver_, stage, "A", mtx_A.data());
    return mtx_A;
  }

  std::vector<double> getMatrixB(int stage)
  {
    std::vector<double> mtx_B(NX_ * NU_);
    ocp_nlp_get_at_stage(nlp_solver_, stage, "B", mtx_B.data());
    return mtx_B;
  }

  /* for debugging */
  void printAcadosStatus(double min_time)
  {
    double kkt_norm_inf;
    int sqp_iter;

    ocp_nlp_out_get(nlp_config_, nlp_dims_, nlp_out_, 0, "kkt_norm_inf", &kkt_norm_inf);
    ocp_nlp_get(nlp_solver_, "sqp_iter", &sqp_iter);

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

  void printAcadosMatrix(int stage = 0)
  {
    std::vector<double> mtx_A = getMatrixA(stage);
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

  void printAcadosReference()
  {
    std::stringstream ss;

    ss << "\n--- xr ---\n";
    for (int i = 0; i <= NN_; i++)
    {
      ss << "Xr Row " << i << ":\n";
      for (int j = 0; j < NX_; j++)
      {
        ss << xr_[i][j] << " ";
      }
      ss << "\n";
    }
    std::cout << ss.str();  // Logging the xr

    ss.str("");  // Clearing the string stream

    ss << "\n--- ur ---\n";
    for (int i = 0; i < NN_; i++)
    {
      ss << "Ur Row " << i << ":\n";
      for (int j = 0; j < NU_; j++)
      {
        ss << ur_[i][j] << " ";
      }
      ss << "\n";
    }
    std::cout << ss.str();  // Logging the ur
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

  inline void setFeedbackConstraints(const std::vector<double>& bx0)
  {
    if (bx0.size() != NBX0_)
      throw std::length_error("bx0 size is not equal to NBX0_");

    setConstraintsValue("lbx", 0, bx0);
    setConstraintsValue("ubx", 0, bx0);
  }

  inline double solveOCPOnce()
  {
    double min_time = 1e12;
    double elapsed_time;

    int status = acadosSolve();
    if (status != ACADOS_SUCCESS)
      throw AcadosSolveException(status);

    ocp_nlp_get(nlp_solver_, "time_tot", &elapsed_time);
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

private:
  void setParamSparseOneStage(int stage, std::vector<int>& idx, std::vector<double>& p, bool if_check_len = true)
  {
    if (if_check_len)
    {
      if (idx.size() != p.size())
        throw std::length_error("idx size is not equal to p size");
    }

    acadosUpdateParamsSparse(stage, idx, p, p.size());
  }

  void setParamSparseAllStages(std::vector<int>& idx, std::vector<double>& p)
  {
    if (idx.size() != p.size())
      throw std::length_error("idx size is not equal to p size");

    for (int i = 0; i < NN_ + 1; i++)
      setParamSparseOneStage(i, idx, p, false);
  }

  void setConstraintsValue(const std::string& constraint_type, int stage, std::vector<double> value,
                           bool is_check_len = true) const
  {
    if (is_check_len)
    {
      if (constraint_type == "lbx" or constraint_type == "ubx")
      {
        if (stage == 0)
        {
          if (value.size() != NBX0_)
            throw std::length_error("data size is not equal to NBX0_");
        }
        else if (stage == NN_)
        {
          if (value.size() != NBXN_)
            throw std::length_error("data size is not equal to NBXN_");
        }
        else
        {
          if (value.size() != NBX_)
            throw std::length_error("data size is not equal to NBX_");
        }
      }
      else if (constraint_type == "lbu" or constraint_type == "ubu")
      {
        if (value.size() != NBU_)
          throw std::length_error("data size is not equal to NBU_");
      }
      else
      {
        throw std::invalid_argument("Invalid constraint type");
      }
    }

    ocp_nlp_constraints_model_set(nlp_config_, nlp_dims_, nlp_in_, nlp_out_, stage, constraint_type.c_str(),
                                  value.data());
  }

  std::vector<int> getConstraintsIdx(const std::string& constraint_type, int stage) const
  {
    std::vector<int> idx;
    if (constraint_type == "idxbx")
    {
      int len;
      if (stage == 0)
      {
        len = NBX0_;
      }
      else if (stage == NN_)
      {
        len = NBXN_;
      }
      else
      {
        len = NBX_;
      }
      idx.resize(len);
    }
    else if (constraint_type == "idxbu")
    {
      idx.resize(NBU_);
    }
    else
    {
      throw std::invalid_argument("Invalid constraint type");
    }

    ocp_nlp_constraints_model_get(nlp_config_, nlp_dims_, nlp_in_, stage, constraint_type.c_str(), idx.data());
    return idx;
  }

  std::vector<double> getConstraintsValue(const std::string& constraint_type, int stage) const
  {
    std::vector<double> constraints;
    if (constraint_type == "lbx" or constraint_type == "ubx")
    {
      int len;
      if (stage == 0)
      {
        len = NBX0_;
      }
      else if (stage == NN_)
      {
        len = NBXN_;
      }
      else
      {
        len = NBX_;
      }
      constraints.resize(len);
    }
    else if (constraint_type == "lbu" or constraint_type == "ubu")
    {
      constraints.resize(NBU_);
    }
    else
    {
      throw std::invalid_argument("Invalid constraint type");
    }

    ocp_nlp_constraints_model_get(nlp_config_, nlp_dims_, nlp_in_, stage, constraint_type.c_str(), constraints.data());
    return constraints;
  }
};

}  // namespace mpc_solver

}  // namespace aerial_robot_control

#endif  // BASE_MPC_SOLVER_H
