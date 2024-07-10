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
  int NN_, NX_, NZ_, NU_, NP_, NBX_, NBX0_, NBU_, NSBX_, NSBU_, NSH_, NSH0_, NSG_, NSPHI_, NSHN_, NSGN_, NSPHIN_,
      NSPHI0_, NSBXN_, NS_, NS0_, NSN_, NG_, NBXN_, NGN_, NY0_, NY_, NYN_, NH_, NHM_, NH0_, NPHI0_, NPHI_, NHN_, NPHIN_,
      NR_;

  aerial_robot_msgs::PredXU x_u_out_;

  double* W_;
  double* WN_;

  MPCSolver()
  {
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
  };
  ~MPCSolver();
  void initialize();
  void reset(const aerial_robot_msgs::PredXU& x_u);
  int solve(const nav_msgs::Odometry& odom_now, double joint_angles[4], const aerial_robot_msgs::PredXU& x_u_ref,
            bool is_debug);

  /* Setters */
  void setCostWDiagElement(int index, double value, bool is_set_WN = true) const;
  void setCostWeight(bool is_update_W, bool is_update_WN);

  /* for debugging */
  void printStatus(double min_time);
  void printSolution();

  void initPredXU(aerial_robot_msgs::PredXU& x_u);

private:
  double* new_time_steps;
  tilt_qd_servo_mdl_solver_capsule* acados_ocp_capsule_;
  ocp_nlp_config* nlp_config_;
  ocp_nlp_dims* nlp_dims_;
  ocp_nlp_in* nlp_in_;
  ocp_nlp_out* nlp_out_;
  ocp_nlp_solver* nlp_solver_;
  void* nlp_opts_;

  void setReference(const aerial_robot_msgs::PredXU& x_u_ref, unsigned int x_stride, unsigned int u_stride);
  void setFeedbackConstraints(const nav_msgs::Odometry& odom_now, const double joint_angles[4]);
  double solveOCPOnce();
  void getSolution(unsigned int x_stride, unsigned int u_stride);

};

}  // namespace nmpc_over_act_full

}  // namespace aerial_robot_control
