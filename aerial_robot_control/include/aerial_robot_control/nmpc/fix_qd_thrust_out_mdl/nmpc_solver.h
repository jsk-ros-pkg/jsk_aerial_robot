// -*- mode: c++ -*-
//
// Created by lijinjie on 23/11/25.
//

#ifndef FIX_QD_MPC_SOLVER_H
#define FIX_QD_MPC_SOLVER_H

#include "aerial_robot_control/nmpc/base_mpc_solver.h"
#include "aerial_robot_control/nmpc/fix_qd_thrust_out_mdl/c_generated_code/acados_solver_fix_qd_thrust_out_mdl.h"

namespace aerial_robot_control
{

namespace mpc_solver
{

class FixQdMdlMPCSolver : public BaseMPCSolver
{
public:
  FixQdMdlMPCSolver()
  {
    // acados macro
    NN_ = FIX_QD_THRUST_OUT_MDL_N;
    NX_ = FIX_QD_THRUST_OUT_MDL_NX;
    NZ_ = FIX_QD_THRUST_OUT_MDL_NZ;
    NU_ = FIX_QD_THRUST_OUT_MDL_NU;
    NP_ = FIX_QD_THRUST_OUT_MDL_NP;
    NBX_ = FIX_QD_THRUST_OUT_MDL_NBX;
    NBX0_ = FIX_QD_THRUST_OUT_MDL_NBX0;
    NBU_ = FIX_QD_THRUST_OUT_MDL_NBU;
    NSBX_ = FIX_QD_THRUST_OUT_MDL_NSBX;
    NSBU_ = FIX_QD_THRUST_OUT_MDL_NSBU;
    NSH_ = FIX_QD_THRUST_OUT_MDL_NSH;
    NSH0_ = FIX_QD_THRUST_OUT_MDL_NSH0;
    NSG_ = FIX_QD_THRUST_OUT_MDL_NSG;
    NSPHI_ = FIX_QD_THRUST_OUT_MDL_NSPHI;
    NSHN_ = FIX_QD_THRUST_OUT_MDL_NSHN;
    NSGN_ = FIX_QD_THRUST_OUT_MDL_NSGN;
    NSPHIN_ = FIX_QD_THRUST_OUT_MDL_NSPHIN;
    NSPHI0_ = FIX_QD_THRUST_OUT_MDL_NSPHI0;
    NSBXN_ = FIX_QD_THRUST_OUT_MDL_NSBXN;
    NS_ = FIX_QD_THRUST_OUT_MDL_NS;
    NS0_ = FIX_QD_THRUST_OUT_MDL_NS0;
    NSN_ = FIX_QD_THRUST_OUT_MDL_NSN;
    NG_ = FIX_QD_THRUST_OUT_MDL_NG;
    NBXN_ = FIX_QD_THRUST_OUT_MDL_NBXN;
    NGN_ = FIX_QD_THRUST_OUT_MDL_NGN;
    NY0_ = FIX_QD_THRUST_OUT_MDL_NY0;
    NY_ = FIX_QD_THRUST_OUT_MDL_NY;
    NYN_ = FIX_QD_THRUST_OUT_MDL_NYN;
    NH_ = FIX_QD_THRUST_OUT_MDL_NH;
    NHN_ = FIX_QD_THRUST_OUT_MDL_NHN;
    NH0_ = FIX_QD_THRUST_OUT_MDL_NH0;
    NPHI0_ = FIX_QD_THRUST_OUT_MDL_NPHI0;
    NPHI_ = FIX_QD_THRUST_OUT_MDL_NPHI;
    NPHIN_ = FIX_QD_THRUST_OUT_MDL_NPHIN;
    NR_ = FIX_QD_THRUST_OUT_MDL_NR;

    // acados functions that only using once
    acados_ocp_capsule_ = fix_qd_thrust_out_mdl_acados_create_capsule();

    int status = fix_qd_thrust_out_mdl_acados_create_with_discretization(acados_ocp_capsule_, NN_, new_time_steps);
    if (status)
      throw std::runtime_error("fix_qd_thrust_out_mdl_acados_create_with_discretization() returned status " +
                               std::to_string(status) + ". Exiting.");

    nlp_config_ = fix_qd_thrust_out_mdl_acados_get_nlp_config(acados_ocp_capsule_);
    nlp_dims_ = fix_qd_thrust_out_mdl_acados_get_nlp_dims(acados_ocp_capsule_);
    nlp_in_ = fix_qd_thrust_out_mdl_acados_get_nlp_in(acados_ocp_capsule_);
    nlp_out_ = fix_qd_thrust_out_mdl_acados_get_nlp_out(acados_ocp_capsule_);
    nlp_solver_ = fix_qd_thrust_out_mdl_acados_get_nlp_solver(acados_ocp_capsule_);
    nlp_opts_ = fix_qd_thrust_out_mdl_acados_get_nlp_opts(acados_ocp_capsule_);
  };

  ~FixQdMdlMPCSolver() override
  {
    int status = fix_qd_thrust_out_mdl_acados_free(acados_ocp_capsule_);
    if (status)
      std::cout << "fix_qd_thrust_out_mdl_acados_free() returned status " << status << ". \n" << std::endl;

    status = fix_qd_thrust_out_mdl_acados_free_capsule(acados_ocp_capsule_);
    if (status)
      std::cout << "fix_qd_thrust_out_mdl_acados_free_capsule() returned status " << status << ". \n" << std::endl;
  };

protected:
  fix_qd_thrust_out_mdl_solver_capsule* acados_ocp_capsule_ = nullptr;

  // acados functions that using multiple times
  inline int acadosUpdateParams(int stage, std::vector<double>& value) override
  {
    return fix_qd_thrust_out_mdl_acados_update_params(acados_ocp_capsule_, stage, value.data(), NP_);
  }

  inline int acadosUpdateParamsSparse(int stage, std::vector<int>& idx, std::vector<double>& p, int n_update) override
  {
    return fix_qd_thrust_out_mdl_acados_update_params_sparse(acados_ocp_capsule_, stage, idx.data(), p.data(),
                                                             n_update);
  }

  inline int acadosSolve() override
  {
    return fix_qd_thrust_out_mdl_acados_solve(acados_ocp_capsule_);
  }

  inline void acadosPrintStats() override
  {
    fix_qd_thrust_out_mdl_acados_print_stats(acados_ocp_capsule_);
  }
};

}  // namespace mpc_solver

}  // namespace aerial_robot_control

#endif  // FIX_QD_MPC_SOLVER_H
