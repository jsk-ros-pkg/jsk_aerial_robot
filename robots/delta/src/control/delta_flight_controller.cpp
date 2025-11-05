#include <delta/control/delta_controller.h>

using namespace aerial_robot_model;
using namespace aerial_robot_control;

void DeltaController::calcAccFromCog()
{
  tf::Quaternion cog2baselink_rot;
  tf::quaternionKDLToTF(robot_model_->getCogDesireOrientation<KDL::Rotation>(), cog2baselink_rot);
  tf::Matrix3x3 cog_rot = estimator_->getOrientation(Frame::BASELINK, estimate_mode_) *
                          tf::Matrix3x3(cog2baselink_rot).inverse();  // w_R_b * cog_R_b.inverse() = w_R_cog

  tf::Vector3 target_linear_acc_w(pid_controllers_.at(X).result(), pid_controllers_.at(Y).result(),
                                  pid_controllers_.at(Z).result());
  tf::Vector3 target_linear_acc_cog = cog_rot.inverse() * target_linear_acc_w;

  Eigen::VectorXd target_acc_cog = Eigen::VectorXd::Zero(6);

  target_acc_cog.head(3) =
      Eigen::Vector3d(target_linear_acc_cog.x(), target_linear_acc_cog.y(), target_linear_acc_cog.z());

  double target_ang_acc_x = pid_controllers_.at(ROLL).result();
  double target_ang_acc_y = pid_controllers_.at(PITCH).result();
  double target_ang_acc_z = pid_controllers_.at(YAW).result();

  Eigen::Vector3d omega;
  tf::vectorTFToEigen(omega_, omega);
  Eigen::Matrix3d inertia = robot_model_->getInertia<Eigen::Matrix3d>();
  Eigen::Vector3d gyro = omega.cross(inertia * omega);

  target_acc_cog.tail(3) =
      Eigen::Vector3d(target_ang_acc_x, target_ang_acc_y, target_ang_acc_z) + inertia.inverse() * gyro;

  target_acc_cog_ = target_acc_cog;
}

void DeltaController::forceLandingProcess()
{
  if (navigator_->getForceLandingFlag() &&
      pid_controllers_.at(Z).result() < 5.0)  // heuristic measures to avoid to large gimbal angles after force land
    start_rp_integration_ = false;
}

void DeltaController::wrenchAllocation()
{
  // calculate linear appropriated thrusts and gimbal angles
  if (first_run_)
  {
    Eigen::MatrixXd full_q_mat = robot_model_for_control_->getFullWrenchAllocationMatrixFromCog();
    Eigen::MatrixXd full_q_mat_inv = aerial_robot_model::pseudoinverse(full_q_mat);

    Eigen::VectorXd full_lambda = full_q_mat_inv * target_acc_cog_;
    for (int i = 0; i < motor_num_; i++)
    {
      lambda_all_.at(i) =
          std::clamp((float)full_lambda.segment(2 * i, 2).norm(), (float)robot_model_->getThrustLowerLimit(),
                     (float)robot_model_->getThrustUpperLimit());
      target_gimbal_angles_.at(i) = angles::normalize_angle(atan2(-full_lambda(2 * i + 0), full_lambda(2 * i + 1)));
    }
  }
  nonlinearWrenchAllocation();
}

void DeltaController::nonlinearWrenchAllocation()
{
  int n_variables = 2 * motor_num_;

  nlopt_log_.resize(n_variables, 0.0);
  nlopt_iterations_ = 0;

  nlopt::opt slsqp_solver(nlopt::LD_SLSQP, n_variables);
  slsqp_solver.set_min_objective(nonlinearWrenchAllocationMinObjective, this);
  slsqp_solver.add_equality_mconstraint(nonlinearWrenchAllocationEqConstraints, this,
                                        { 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6 });

  std::vector<double> lb(n_variables, -INFINITY);
  std::vector<double> ub(n_variables, INFINITY);
  for (int i = 0; i < motor_num_; i++)
  {
    lb.at(i) = 0;
    ub.at(i) = robot_model_->getThrustUpperLimit();
    lb.at(i + motor_num_) = -M_PI;
    ub.at(i + motor_num_) = M_PI;
  }

  slsqp_solver.set_lower_bounds(lb);
  slsqp_solver.set_upper_bounds(ub);
  slsqp_solver.set_xtol_rel(1e-6);
  slsqp_solver.set_ftol_rel(1e-6);
  slsqp_solver.set_maxeval(1000);

  /* set initial variable */
  std::vector<double> opt_x(n_variables, 0);
  for (int i = 0; i < motor_num_; i++)
  {
    opt_x.at(i) = std::clamp((double)lambda_all_.at(i), lb.at(i), ub.at(i));
    opt_x.at(i + motor_num_) = std::clamp((double)angles::normalize_angle(target_gimbal_angles_.at(i)),
                                          lb.at(i + motor_num_), ub.at(i + motor_num_));
  }

  double max_val;
  nlopt::result result;
  try
  {
    result = slsqp_solver.optimize(opt_x, max_val);
    nlopt_result_ = result;
  }
  catch (std::runtime_error error)
  {
  }

  if (result < 0)
    ROS_ERROR_STREAM("[nlopt] failed to solve. result is " << result);

  for (int i = 0; i < opt_x.size(); i++)
    nlopt_log_.at(i) = opt_x.at(i);

  /* set optimal variables to actuator input */
  for (int i = 0; i < motor_num_; i++)
  {
    lambda_all_.at(i) = opt_x.at(i);
    target_gimbal_angles_.at(i) = opt_x.at(i + motor_num_);
  }
}
