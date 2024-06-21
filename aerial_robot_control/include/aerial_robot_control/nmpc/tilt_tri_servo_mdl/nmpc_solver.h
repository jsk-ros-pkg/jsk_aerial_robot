//
// Created by lijinjie on 23/11/29.
//

#ifndef TILT_TRI_NMPC_SOLVER_H
#define TILT_TRI_NMPC_SOLVER_H

#endif  // TILT_TRI_NMPC_SOLVER_H

#include <ros/console.h>
#include <aerial_robot_msgs/PredXU.h>
#include <nav_msgs/Odometry.h>
#include <iostream>

// acados
#include "acados/utils/math.h"
#include "acados/utils/print.h"
#include "acados_c/external_function_interface.h"
#include "acados_c/ocp_nlp_interface.h"
#include "aerial_robot_control/nmpc/tilt_tri_servo_mdl/c_generated_code/acados_solver_tilt_tri_servo_mdl.h"

#define NN TILT_TRI_SERVO_MDL_N
#define NX TILT_TRI_SERVO_MDL_NX
#define NZ TILT_TRI_SERVO_MDL_NZ
#define NU TILT_TRI_SERVO_MDL_NU
#define NP TILT_TRI_SERVO_MDL_NP
#define NBX TILT_TRI_SERVO_MDL_NBX
#define NBX0 TILT_TRI_SERVO_MDL_NBX0
#define NBU TILT_TRI_SERVO_MDL_NBU
#define NSBX TILT_TRI_SERVO_MDL_NSBX
#define NSBU TILT_TRI_SERVO_MDL_NSBU
#define NSH TILT_TRI_SERVO_MDL_NSH
#define NSG TILT_TRI_SERVO_MDL_NSG
#define NSPHI TILT_TRI_SERVO_MDL_NSPHI
#define NSHN TILT_TRI_SERVO_MDL_NSHN
#define NSGN TILT_TRI_SERVO_MDL_NSGN
#define NSPHIN TILT_TRI_SERVO_MDL_NSPHIN
#define NSBXN TILT_TRI_SERVO_MDL_NSBXN
#define NS TILT_TRI_SERVO_MDL_NS
#define NSN TILT_TRI_SERVO_MDL_NSN
#define NG TILT_TRI_SERVO_MDL_NG
#define NBXN TILT_TRI_SERVO_MDL_NBXN
#define NGN TILT_TRI_SERVO_MDL_NGN
#define NY0 TILT_TRI_SERVO_MDL_NY0
#define NY TILT_TRI_SERVO_MDL_NY
#define NYN TILT_TRI_SERVO_MDL_NYN
#define NH TILT_TRI_SERVO_MDL_NH
#define NPHI TILT_TRI_SERVO_MDL_NPHI
#define NHN TILT_TRI_SERVO_MDL_NHN
#define NPHIN TILT_TRI_SERVO_MDL_NPHIN
#define NR TILT_TRI_SERVO_MDL_NR

namespace aerial_robot_control
{

namespace nmpc_tilt_tri_full
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
  int nx_;
  int nu_;
  double* W_;
  double* WN_;

  MPCSolver();
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

private:
  double* new_time_steps;
  tilt_tri_servo_mdl_solver_capsule* acados_ocp_capsule_;
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

void initPredXU(aerial_robot_msgs::PredXU& x_u);

}  // namespace nmpc_tilt_tri_full

}  // namespace aerial_robot_control
