//
// Created by li-jinjie on 24-12-8.
//

#ifndef MHE_WRENCH_EST_ACC_MOM_SOLVER_H
#define MHE_WRENCH_EST_ACC_MOM_SOLVER_H

#include "aerial_robot_control/nmpc/base_mhe_solver.h"
#include "aerial_robot_control/wrench_est/mhe_wrench_est_acc_mom_mdl/c_generated_code/acados_solver_mhe_wrench_est_acc_mom_mdl.h"


namespace aerial_robot_control
{

namespace mhe_solver
{

class MHEWrenchEstAccMom : public mhe_solver::BaseMHESolver
{
public:
  MHEWrenchEstAccMom()
  {
    // acados macro
    NN_ = MHE_WRENCH_EST_ACC_MOM_MDL_N;
    NX_ = MHE_WRENCH_EST_ACC_MOM_MDL_NX;
    NZ_ = MHE_WRENCH_EST_ACC_MOM_MDL_NZ;
    NU_ = MHE_WRENCH_EST_ACC_MOM_MDL_NU;
    NP_ = MHE_WRENCH_EST_ACC_MOM_MDL_NP;
    NBX_ = MHE_WRENCH_EST_ACC_MOM_MDL_NBX;
    NBX0_ = MHE_WRENCH_EST_ACC_MOM_MDL_NBX0;
    NBU_ = MHE_WRENCH_EST_ACC_MOM_MDL_NBU;
    NSBX_ = MHE_WRENCH_EST_ACC_MOM_MDL_NSBX;
    NSBU_ = MHE_WRENCH_EST_ACC_MOM_MDL_NSBU;
    NSH_ = MHE_WRENCH_EST_ACC_MOM_MDL_NSH;
    NSH0_ = MHE_WRENCH_EST_ACC_MOM_MDL_NSH0;
    NSG_ = MHE_WRENCH_EST_ACC_MOM_MDL_NSG;
    NSPHI_ = MHE_WRENCH_EST_ACC_MOM_MDL_NSPHI;
    NSHN_ = MHE_WRENCH_EST_ACC_MOM_MDL_NSHN;
    NSGN_ = MHE_WRENCH_EST_ACC_MOM_MDL_NSGN;
    NSPHIN_ = MHE_WRENCH_EST_ACC_MOM_MDL_NSPHIN;
    NSPHI0_ = MHE_WRENCH_EST_ACC_MOM_MDL_NSPHI0;
    NSBXN_ = MHE_WRENCH_EST_ACC_MOM_MDL_NSBXN;
    NS_ = MHE_WRENCH_EST_ACC_MOM_MDL_NS;
    NS0_ = MHE_WRENCH_EST_ACC_MOM_MDL_NS0;
    NSN_ = MHE_WRENCH_EST_ACC_MOM_MDL_NSN;
    NG_ = MHE_WRENCH_EST_ACC_MOM_MDL_NG;
    NBXN_ = MHE_WRENCH_EST_ACC_MOM_MDL_NBXN;
    NGN_ = MHE_WRENCH_EST_ACC_MOM_MDL_NGN;
    NY0_ = MHE_WRENCH_EST_ACC_MOM_MDL_NY0;
    NY_ = MHE_WRENCH_EST_ACC_MOM_MDL_NY;
    NYN_ = MHE_WRENCH_EST_ACC_MOM_MDL_NYN;
    NH_ = MHE_WRENCH_EST_ACC_MOM_MDL_NH;
    NHN_ = MHE_WRENCH_EST_ACC_MOM_MDL_NHN;
    NH0_ = MHE_WRENCH_EST_ACC_MOM_MDL_NH0;
    NPHI0_ = MHE_WRENCH_EST_ACC_MOM_MDL_NPHI0;
    NPHI_ = MHE_WRENCH_EST_ACC_MOM_MDL_NPHI;
    NPHIN_ = MHE_WRENCH_EST_ACC_MOM_MDL_NPHIN;
    NR_ = MHE_WRENCH_EST_ACC_MOM_MDL_NR;

    // only for MHE
    NM_ = 6;

    // acados functions that only using once
    acados_ocp_capsule_ = mhe_wrench_est_acc_mom_mdl_acados_create_capsule();

    int status = mhe_wrench_est_acc_mom_mdl_acados_create(acados_ocp_capsule_);
    if (status)
      throw std::runtime_error("mhe_wrench_est_acc_mom_mdl_acados_create() returned status " + std::to_string(status) +
                               ". Exiting.");

    nlp_config_ = mhe_wrench_est_acc_mom_mdl_acados_get_nlp_config(acados_ocp_capsule_);
    nlp_dims_ = mhe_wrench_est_acc_mom_mdl_acados_get_nlp_dims(acados_ocp_capsule_);
    nlp_in_ = mhe_wrench_est_acc_mom_mdl_acados_get_nlp_in(acados_ocp_capsule_);
    nlp_out_ = mhe_wrench_est_acc_mom_mdl_acados_get_nlp_out(acados_ocp_capsule_);
    nlp_solver_ = mhe_wrench_est_acc_mom_mdl_acados_get_nlp_solver(acados_ocp_capsule_);
    nlp_opts_ = mhe_wrench_est_acc_mom_mdl_acados_get_nlp_opts(acados_ocp_capsule_);
  };

  ~MHEWrenchEstAccMom() override
  {
    int status = mhe_wrench_est_acc_mom_mdl_acados_free(acados_ocp_capsule_);
    if (status)
      std::cout << "mhe_wrench_est_acc_mom_mdl_acados_free() returned status " << status << ". \n" << std::endl;

    status = mhe_wrench_est_acc_mom_mdl_acados_free_capsule(acados_ocp_capsule_);
    if (status)
      std::cout << "mhe_wrench_est_acc_mom_mdl_acados_free_capsule() returned status " << status << ". \n" << std::endl;
  };

protected:
  mhe_wrench_est_acc_mom_mdl_solver_capsule* acados_ocp_capsule_ = nullptr;

  // acados functions that using multiple times
  inline int acadosUpdateParams(int stage, std::vector<double>& value) override
  {
    return mhe_wrench_est_acc_mom_mdl_acados_update_params(acados_ocp_capsule_, stage, value.data(), NP_);
  }

  inline int acadosUpdateParamsSparse(int stage, std::vector<int>& idx, std::vector<double>& p, int n_update) override
  {
    return mhe_wrench_est_acc_mom_mdl_acados_update_params_sparse(acados_ocp_capsule_, stage, idx.data(), p.data(), n_update);
  }

  inline int acadosSolve() override
  {
    return mhe_wrench_est_acc_mom_mdl_acados_solve(acados_ocp_capsule_);
  }

  inline void acadosPrintStats() override
  {
    mhe_wrench_est_acc_mom_mdl_acados_print_stats(acados_ocp_capsule_);
  }
};

}  // namespace mpc_solver

}  // namespace aerial_robot_control

#endif //MHE_WRENCH_EST_ACC_MOM_SOLVER_H
