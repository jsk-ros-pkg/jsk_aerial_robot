//
// Created by li-jinjie on 24-11-20.
//

#ifndef TILT_QD_SERVO_THRUST_DIST_IMP_MDL_NMPC_SOLVER_H
#define TILT_QD_SERVO_THRUST_DIST_IMP_MDL_NMPC_SOLVER_H

#include "aerial_robot_control/nmpc/base_mpc_solver.h"
#include "aerial_robot_control/nmpc/tilt_qd_servo_thrust_dist_imp_mdl/c_generated_code/acados_solver_tilt_qd_servo_thrust_dist_imp_mdl.h"

#include "Eigen/Dense"

namespace aerial_robot_control
{

namespace mpc_solver
{

class TiltQdServoThrustDistImpMdlMPCSolver : public BaseMPCSolver
{
public:
  TiltQdServoThrustDistImpMdlMPCSolver()
  {
    // acados macro
    NN_ = TILT_QD_SERVO_THRUST_DIST_IMP_MDL_N;
    NX_ = TILT_QD_SERVO_THRUST_DIST_IMP_MDL_NX;
    NZ_ = TILT_QD_SERVO_THRUST_DIST_IMP_MDL_NZ;
    NU_ = TILT_QD_SERVO_THRUST_DIST_IMP_MDL_NU;
    NP_ = TILT_QD_SERVO_THRUST_DIST_IMP_MDL_NP;
    NBX_ = TILT_QD_SERVO_THRUST_DIST_IMP_MDL_NBX;
    NBX0_ = TILT_QD_SERVO_THRUST_DIST_IMP_MDL_NBX0;
    NBU_ = TILT_QD_SERVO_THRUST_DIST_IMP_MDL_NBU;
    NSBX_ = TILT_QD_SERVO_THRUST_DIST_IMP_MDL_NSBX;
    NSBU_ = TILT_QD_SERVO_THRUST_DIST_IMP_MDL_NSBU;
    NSH_ = TILT_QD_SERVO_THRUST_DIST_IMP_MDL_NSH;
    NSH0_ = TILT_QD_SERVO_THRUST_DIST_IMP_MDL_NSH0;
    NSG_ = TILT_QD_SERVO_THRUST_DIST_IMP_MDL_NSG;
    NSPHI_ = TILT_QD_SERVO_THRUST_DIST_IMP_MDL_NSPHI;
    NSHN_ = TILT_QD_SERVO_THRUST_DIST_IMP_MDL_NSHN;
    NSGN_ = TILT_QD_SERVO_THRUST_DIST_IMP_MDL_NSGN;
    NSPHIN_ = TILT_QD_SERVO_THRUST_DIST_IMP_MDL_NSPHIN;
    NSPHI0_ = TILT_QD_SERVO_THRUST_DIST_IMP_MDL_NSPHI0;
    NSBXN_ = TILT_QD_SERVO_THRUST_DIST_IMP_MDL_NSBXN;
    NS_ = TILT_QD_SERVO_THRUST_DIST_IMP_MDL_NS;
    NS0_ = TILT_QD_SERVO_THRUST_DIST_IMP_MDL_NS0;
    NSN_ = TILT_QD_SERVO_THRUST_DIST_IMP_MDL_NSN;
    NG_ = TILT_QD_SERVO_THRUST_DIST_IMP_MDL_NG;
    NBXN_ = TILT_QD_SERVO_THRUST_DIST_IMP_MDL_NBXN;
    NGN_ = TILT_QD_SERVO_THRUST_DIST_IMP_MDL_NGN;
    NY0_ = TILT_QD_SERVO_THRUST_DIST_IMP_MDL_NY0;
    NY_ = TILT_QD_SERVO_THRUST_DIST_IMP_MDL_NY;
    NYN_ = TILT_QD_SERVO_THRUST_DIST_IMP_MDL_NYN;
    NH_ = TILT_QD_SERVO_THRUST_DIST_IMP_MDL_NH;
    NHN_ = TILT_QD_SERVO_THRUST_DIST_IMP_MDL_NHN;
    NH0_ = TILT_QD_SERVO_THRUST_DIST_IMP_MDL_NH0;
    NPHI0_ = TILT_QD_SERVO_THRUST_DIST_IMP_MDL_NPHI0;
    NPHI_ = TILT_QD_SERVO_THRUST_DIST_IMP_MDL_NPHI;
    NPHIN_ = TILT_QD_SERVO_THRUST_DIST_IMP_MDL_NPHIN;
    NR_ = TILT_QD_SERVO_THRUST_DIST_IMP_MDL_NR;

    // acados functions that only using once
    acados_ocp_capsule_ = tilt_qd_servo_thrust_dist_imp_mdl_acados_create_capsule();

    int status = tilt_qd_servo_thrust_dist_imp_mdl_acados_create(acados_ocp_capsule_);
    if (status)
      throw std::runtime_error("tilt_qd_servo_thrust_dist_imp_mdl_acados_create() returned status " +
                               std::to_string(status) + ". Exiting.");

    nlp_config_ = tilt_qd_servo_thrust_dist_imp_mdl_acados_get_nlp_config(acados_ocp_capsule_);
    nlp_dims_ = tilt_qd_servo_thrust_dist_imp_mdl_acados_get_nlp_dims(acados_ocp_capsule_);
    nlp_in_ = tilt_qd_servo_thrust_dist_imp_mdl_acados_get_nlp_in(acados_ocp_capsule_);
    nlp_out_ = tilt_qd_servo_thrust_dist_imp_mdl_acados_get_nlp_out(acados_ocp_capsule_);
    nlp_solver_ = tilt_qd_servo_thrust_dist_imp_mdl_acados_get_nlp_solver(acados_ocp_capsule_);
    nlp_opts_ = tilt_qd_servo_thrust_dist_imp_mdl_acados_get_nlp_opts(acados_ocp_capsule_);
  };

  ~TiltQdServoThrustDistImpMdlMPCSolver() override
  {
    int status = tilt_qd_servo_thrust_dist_imp_mdl_acados_free(acados_ocp_capsule_);
    if (status)
      std::cout << "tilt_qd_servo_thrust_dist_imp_mdl_acados_free() returned status " << status << ". \n" << std::endl;

    status = tilt_qd_servo_thrust_dist_imp_mdl_acados_free_capsule(acados_ocp_capsule_);
    if (status)
      std::cout << "tilt_qd_servo_thrust_dist_imp_mdl_acados_free_capsule() returned status " << status << ". \n"
                << std::endl;
  };

  void setImpedanceWeight(const std::string& type, const double value, const bool is_update_W_WN = true)
  {
    if (type == "pMx")
      pM_[0] = value;
    else if (type == "pMy")
      pM_[1] = value;
    else if (type == "pMz")
      pM_[2] = value;
    else if (type == "pDx")
      pD_[0] = value;
    else if (type == "pDy")
      pD_[1] = value;
    else if (type == "pDz")
      pD_[2] = value;
    else if (type == "pKx")
      pK_[0] = value;
    else if (type == "pKy")
      pK_[1] = value;
    else if (type == "pKz")
      pK_[2] = value;
    else if (type == "oMx")
      oM_[0] = value;
    else if (type == "oMy")
      oM_[1] = value;
    else if (type == "oMz")
      oM_[2] = value;
    else if (type == "oDx")
      oD_[0] = value;
    else if (type == "oDy")
      oD_[1] = value;
    else if (type == "oDz")
      oD_[2] = value;
    else if (type == "oKx")
      oK_[0] = value;
    else if (type == "oKy")
      oK_[1] = value;
    else if (type == "oKz")
      oK_[2] = value;
    else
      std::cout << "Not working. Invalid impedance weight type: " << type << std::endl;

    if (is_update_W_WN)
      setCostWeightByImpMDK();
  }

protected:
  tilt_qd_servo_thrust_dist_imp_mdl_solver_capsule* acados_ocp_capsule_ = nullptr;

  // impedance related
  double pM_[3], oM_[3];
  double pD_[3], oD_[3];
  double pK_[3], oK_[3];

  void setCostWeightByImpMDK(bool is_set_WN = true)
  {
    if (W_.size() != NY_ * NY_)
      throw std::length_error("W size is not equal to NY_ * NY_, please check.");
    if (WN_.size() != NX_ * NX_)
      throw std::length_error("WN size is not equal to NX_ * NX_, please check.");

    // position
    Eigen::MatrixXd pK_imp = Eigen::DiagonalMatrix<double, 3>(pK_[0], pK_[1], pK_[2]);
    Eigen::MatrixXd pD_imp = Eigen::DiagonalMatrix<double, 3>(pD_[0], pD_[1], pD_[2]);
    Eigen::MatrixXd pM_imp = Eigen::DiagonalMatrix<double, 3>(pM_[0], pM_[1], pM_[2]);

    Eigen::MatrixXd p_weight(9, 3);
    p_weight.block<3, 3>(0, 0) = pK_imp;
    p_weight.block<3, 3>(3, 0) = pD_imp;
    p_weight.block<3, 3>(6, 0) = pM_imp;

    Eigen::MatrixXd p_weight_mtx = p_weight * p_weight.transpose();

    // orientation
    Eigen::MatrixXd oK_imp = Eigen::DiagonalMatrix<double, 3>(oK_[0], oK_[1], oK_[2]);
    Eigen::MatrixXd oD_imp = Eigen::DiagonalMatrix<double, 3>(oD_[0], oD_[1], oD_[2]);
    Eigen::MatrixXd oM_imp = Eigen::DiagonalMatrix<double, 3>(oM_[0], oM_[1], oM_[2]);

    Eigen::MatrixXd o_weight(9, 3);
    o_weight.block<3, 3>(0, 0) = oK_imp;
    o_weight.block<3, 3>(3, 0) = oD_imp;
    o_weight.block<3, 3>(6, 0) = oM_imp;

    Eigen::MatrixXd o_weight_mtx = o_weight * o_weight.transpose();

    // convert W_ from std::vector to Eigen::matrix
    Eigen::MatrixXd W_mtx = Eigen::Map<Eigen::MatrixXd>(W_.data(), NY_, NY_);

    // assign values: position
    //  W[0:6, 0:6] = p_weight_mtx[0:6, 0:6]
    W_mtx.block<6, 6>(0, 0) = p_weight_mtx.block<6, 6>(0, 0);
    // W[21:24, 21:24] = p_weight_mtx[6:9, 6:9]
    W_mtx.block<3, 3>(21, 21) = p_weight_mtx.block<3, 3>(6, 6);
    // W[0:6, 21:24] = p_weight_mtx[0:6, 6:9]
    W_mtx.block<6, 3>(0, 21) = p_weight_mtx.block<6, 3>(0, 6);
    // W[21:24, 0:6] = p_weight_mtx[6:9, 0:6]
    W_mtx.block<3, 6>(21, 0) = p_weight_mtx.block<3, 6>(6, 0);

    // assign values: orientation
    // W[7:13, 7:13] = o_weight_mtx[0:6, 0:6]
    W_mtx.block<6, 6>(7, 7) = o_weight_mtx.block<6, 6>(0, 0);
    // W[24:27, 24:27] = o_weight_mtx[6:9, 6:9]
    W_mtx.block<3, 3>(24, 24) = o_weight_mtx.block<3, 3>(6, 6);
    // W[7:13, 24:27] = o_weight_mtx[0:6, 6:9]
    W_mtx.block<6, 3>(7, 24) = o_weight_mtx.block<6, 3>(0, 6);
    // W[24:27, 7:13] = o_weight_mtx[6:9, 0:6]
    W_mtx.block<3, 6>(24, 7) = o_weight_mtx.block<3, 6>(6, 0);

    W_ = std::vector<double>(W_mtx.data(), W_mtx.data() + W_mtx.size());
    setCostWeightMid(W_);

    if (is_set_WN)
    {
      Eigen::MatrixXd WN_mtx = W_mtx.block(0, 0, NX_, NX_);
      WN_ = std::vector<double>(WN_mtx.data(), WN_mtx.data() + WN_mtx.size());
      setCostWeightEnd(WN_);

      // std::cout << "W matrix:\n" << W_mtx << std::endl;
      // std::cout << "WN matrix:\n" << WN_mtx << std::endl;
    }
  }

  // acados functions that using multiple times
  inline int acadosUpdateParams(int stage, std::vector<double>& value) override
  {
    return tilt_qd_servo_thrust_dist_imp_mdl_acados_update_params(acados_ocp_capsule_, stage, value.data(), NP_);
  }

  inline int acadosUpdateParamsSparse(int stage, std::vector<int>& idx, std::vector<double>& p, int n_update) override
  {
    return tilt_qd_servo_thrust_dist_imp_mdl_acados_update_params_sparse(acados_ocp_capsule_, stage, idx.data(),
                                                                         p.data(), n_update);
  }

  inline int acadosSolve() override
  {
    return tilt_qd_servo_thrust_dist_imp_mdl_acados_solve(acados_ocp_capsule_);
  }

  inline void acadosPrintStats() override
  {
    tilt_qd_servo_thrust_dist_imp_mdl_acados_print_stats(acados_ocp_capsule_);
  }
};

}  // namespace mpc_solver

}  // namespace aerial_robot_control

#endif  // TILT_QD_SERVO_THRUST_DIST_IMP_MDL_NMPC_SOLVER_H
