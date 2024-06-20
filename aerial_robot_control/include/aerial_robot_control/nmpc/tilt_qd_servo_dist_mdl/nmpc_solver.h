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
#include "aerial_robot_control/nmpc/tilt_qd_servo_dist_mdl/c_generated_code/acados_solver_tilt_qd_servo_dist_mdl.h"

#define NN TILT_QD_SERVO_DIST_MDL_N
#define NX TILT_QD_SERVO_DIST_MDL_NX
#define NZ TILT_QD_SERVO_DIST_MDL_NZ
#define NU TILT_QD_SERVO_DIST_MDL_NU
#define NP TILT_QD_SERVO_DIST_MDL_NP
#define NBX TILT_QD_SERVO_DIST_MDL_NBX
#define NBX0 TILT_QD_SERVO_DIST_MDL_NBX0
#define NBU TILT_QD_SERVO_DIST_MDL_NBU
#define NSBX TILT_QD_SERVO_DIST_MDL_NSBX
#define NSBU TILT_QD_SERVO_DIST_MDL_NSBU
#define NSH TILT_QD_SERVO_DIST_MDL_NSH
#define NSG TILT_QD_SERVO_DIST_MDL_NSG
#define NSPHI TILT_QD_SERVO_DIST_MDL_NSPHI
#define NSHN TILT_QD_SERVO_DIST_MDL_NSHN
#define NSGN TILT_QD_SERVO_DIST_MDL_NSGN
#define NSPHIN TILT_QD_SERVO_DIST_MDL_NSPHIN
#define NSBXN TILT_QD_SERVO_DIST_MDL_NSBXN
#define NS TILT_QD_SERVO_DIST_MDL_NS
#define NSN TILT_QD_SERVO_DIST_MDL_NSN
#define NG TILT_QD_SERVO_DIST_MDL_NG
#define NBXN TILT_QD_SERVO_DIST_MDL_NBXN
#define NGN TILT_QD_SERVO_DIST_MDL_NGN
#define NY0 TILT_QD_SERVO_DIST_MDL_NY0
#define NY TILT_QD_SERVO_DIST_MDL_NY
#define NYN TILT_QD_SERVO_DIST_MDL_NYN
#define NH TILT_QD_SERVO_DIST_MDL_NH
#define NPHI TILT_QD_SERVO_DIST_MDL_NPHI
#define NHN TILT_QD_SERVO_DIST_MDL_NHN
#define NPHIN TILT_QD_SERVO_DIST_MDL_NPHIN
#define NR TILT_QD_SERVO_DIST_MDL_NR

namespace aerial_robot_control
{

namespace nmpc_over_act_full_i_term
{

struct Constraints
{
  double v_max;
  double v_min;
  double w_max;
  double w_min;
  double thrust_max;
  double thrust_min;
  double a_max;
  double a_min;
};

class MPCSolver
{
public:
  aerial_robot_msgs::PredXU x_u_out_;
  double* mtx_w_;
  double* mtx_wn_;

  MPCSolver();
  ~MPCSolver();
  void initialize();
  void reset(const aerial_robot_msgs::PredXU& x_u);
  int solve(const aerial_robot_msgs::PredXU& x_u_ref, const nav_msgs::Odometry& odom_now, double joint_angles[4],
            double f_disturb_w[3], double tau_disturb_cog[3], bool is_debug);
  // TODO: refactor the last arguments to states and parameters.

  /* Setters */
  void setCostWDiagElement(int index, double value, bool is_set_mtx_wn = true) const;
  void setCostWeight(bool is_update_W, bool is_update_WN);

  /* for debugging */
  void printStatus(double min_time);
  void printSolution();

private:
  double* new_time_steps;
  tilt_qd_servo_dist_mdl_solver_capsule* acados_ocp_capsule_;
  ocp_nlp_config* nlp_config_;
  ocp_nlp_dims* nlp_dims_;
  ocp_nlp_in* nlp_in_;
  ocp_nlp_out* nlp_out_;
  ocp_nlp_solver* nlp_solver_;
  void* nlp_opts_;

  void setReference(const aerial_robot_msgs::PredXU& x_u_ref, unsigned int x_stride, unsigned int u_stride,
                    double* params);
  void setFeedbackConstraints(const nav_msgs::Odometry& odom_now, const double joint_angles[4]);
  double solveOCPOnce();
  void getSolution(unsigned int x_stride, unsigned int u_stride);
};

void initPredXU(aerial_robot_msgs::PredXU& x_u);

}  // namespace nmpc_over_act_full_i_term

}  // namespace aerial_robot_control
