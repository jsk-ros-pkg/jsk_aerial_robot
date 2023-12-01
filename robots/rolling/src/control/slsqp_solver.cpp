#include <rolling/control/rolling_controller.h>

using namespace aerial_robot_model;
using namespace aerial_robot_control;

double slsqpMinObjective(const std::vector<double> &x, std::vector<double> &grad, void *ptr)
{
  if(!grad.empty())
    {
      for(int i = 0; i < x.size(); i++) grad.at(i) = 2.0 * x.at(i);
    }

  double ret = 0.0;
  for(int i = 0; i < x.size(); i++) ret += (x.at(i) * x.at(i));
  return ret;
}

double rollLowerBoundConstraint(const std::vector<double> &x, std::vector<double> &grad, void *ptr)
{
  RollingController *controller = reinterpret_cast<RollingController*>(ptr);

  double target_roll_acc = controller->getTargetWrenchAccTargetFrame()(ROLL);
  Eigen::MatrixXd full_q_mat_target_frame_roll = controller->getFullQTargetFrame().row(ROLL);

  double roll_acc = 0.0;
  for(int i = 0; i < x.size(); i++) roll_acc += (full_q_mat_target_frame_roll(i) * x.at(i));

  if(!grad.empty())
    {
      for(int i = 0; i < x.size(); i++) grad.at(i) = -full_q_mat_target_frame_roll(i);
    }

  return target_roll_acc - roll_acc;
}

double rollUpperBoundConstraint(const std::vector<double> &x, std::vector<double> &grad, void *ptr)
{
  RollingController *controller = reinterpret_cast<RollingController*>(ptr);

  double target_roll_acc = controller->getTargetWrenchAccTargetFrame()(ROLL);
  Eigen::MatrixXd full_q_mat_target_frame_roll = controller->getFullQTargetFrame().row(ROLL);

  double roll_acc = 0.0;
  for(int i = 0; i < x.size(); i++) roll_acc += (full_q_mat_target_frame_roll(i) * x.at(i));

  if(!grad.empty())
    {
      for(int i = 0; i < x.size(); i++) grad.at(i) = full_q_mat_target_frame_roll(i);
    }

  return roll_acc - target_roll_acc;
}

double pitchLowerBoundConstraint(const std::vector<double> &x, std::vector<double> &grad, void *ptr)
{
  RollingController *controller = reinterpret_cast<RollingController*>(ptr);

  double target_pitch_acc = controller->getTargetWrenchAccTargetFrame()(PITCH);
  Eigen::MatrixXd full_q_mat_target_frame_pitch = controller->getFullQTargetFrame().row(PITCH);

  double pitch_acc = 0.0;
  for(int i = 0; i < x.size(); i++) pitch_acc += (full_q_mat_target_frame_pitch(i) * x.at(i));

  if(!grad.empty())
    {
      for(int i = 0; i < x.size(); i++) grad.at(i) = -full_q_mat_target_frame_pitch(i);
    }

  return target_pitch_acc - pitch_acc;
}

double pitchUpperBoundConstraint(const std::vector<double> &x, std::vector<double> &grad, void *ptr)
{
  RollingController *controller = reinterpret_cast<RollingController*>(ptr);

  double target_pitch_acc = controller->getTargetWrenchAccTargetFrame()(PITCH);
  Eigen::MatrixXd full_q_mat_target_frame_pitch = controller->getFullQTargetFrame().row(PITCH);

  double pitch_acc = 0.0;
  for(int i = 0; i < x.size(); i++) pitch_acc += (full_q_mat_target_frame_pitch(i) * x.at(i));

  if(!grad.empty())
    {
      for(int i = 0; i < x.size(); i++) grad.at(i) = full_q_mat_target_frame_pitch(i);
    }

  return pitch_acc - target_pitch_acc;
}

double yawLowerBoundConstraint(const std::vector<double> &x, std::vector<double> &grad, void *ptr)
{
  RollingController *controller = reinterpret_cast<RollingController*>(ptr);

  double target_yaw_acc = controller->getTargetWrenchAccTargetFrame()(YAW);
  Eigen::MatrixXd full_q_mat_target_frame_yaw = controller->getFullQTargetFrame().row(YAW);

  double yaw_acc = 0.0;
  for(int i = 0; i < x.size(); i++) yaw_acc += (full_q_mat_target_frame_yaw(i) * x.at(i));

  if(!grad.empty())
    {
      for(int i = 0; i < x.size(); i++) grad.at(i) = -full_q_mat_target_frame_yaw(i);
    }

  return target_yaw_acc - yaw_acc;
}

double yawUpperBoundConstraint(const std::vector<double> &x, std::vector<double> &grad, void *ptr)
{
  RollingController *controller = reinterpret_cast<RollingController*>(ptr);

  double target_yaw_acc = controller->getTargetWrenchAccTargetFrame()(YAW);
  Eigen::MatrixXd full_q_mat_target_frame_yaw = controller->getFullQTargetFrame().row(YAW);

  double yaw_acc = 0.0;
  for(int i = 0; i < x.size(); i++) yaw_acc += (full_q_mat_target_frame_yaw(i) * x.at(i));

  if(!grad.empty())
    {
      for(int i = 0; i < x.size(); i++) grad.at(i) = full_q_mat_target_frame_yaw(i);
    }


  return yaw_acc - target_yaw_acc;
}

double zUpperBoundConstraint(const std::vector<double> &x, std::vector<double> &grad, void *ptr)
{
  RollingController *controller = reinterpret_cast<RollingController*>(ptr);

  Eigen::MatrixXd full_q_mat_target_frame_z = controller->getFullQTargetFrame().row(Z) * controller->getRobotMassForOpt();

  double z_force = 0.0;
  for(int i = 0; i < x.size(); i++) z_force += (full_q_mat_target_frame_z(i) * x.at(i));

  if(!grad.empty())
    {
      for(int i = 0; i < x.size(); i++) grad.at(i) = full_q_mat_target_frame_z(i);
    }

  return z_force - controller->getRobotMassForOpt() * controller->getGravityForOpt();
}

double xLowerBoundConstraint(const std::vector<double> &x, std::vector<double> &grad, void *ptr)
{
  RollingController *controller = reinterpret_cast<RollingController*>(ptr);

  Eigen::MatrixXd full_q_mat_target_frame_x = controller->getFullQTargetFrame().row(X) * controller->getRobotMassForOpt();
  Eigen::MatrixXd full_q_mat_target_frame_z = controller->getFullQTargetFrame().row(Z) * controller->getRobotMassForOpt();

  Eigen::MatrixXd lambda_coefficient = full_q_mat_target_frame_x + controller->getSteeringMu() * full_q_mat_target_frame_z;

  if(!grad.empty())
    {
      for(int i = 0; i < x.size(); i++) grad.at(i) = lambda_coefficient(i);
    }

  double ret = -controller->getSteeringMu() * controller->getRobotMassForOpt() * controller->getGravityForOpt();
  for(int i = 0; i < x.size(); i++) ret += (lambda_coefficient(i) * x.at(i));

  return ret;
}

double xUpperBoundConstraint(const std::vector<double> &x, std::vector<double> &grad, void *ptr)
{
  RollingController *controller = reinterpret_cast<RollingController*>(ptr);

  Eigen::MatrixXd full_q_mat_target_frame_x = controller->getFullQTargetFrame().row(X) * controller->getRobotMassForOpt();
  Eigen::MatrixXd full_q_mat_target_frame_z = controller->getFullQTargetFrame().row(Z) * controller->getRobotMassForOpt();

  Eigen::MatrixXd lambda_coefficient = -full_q_mat_target_frame_x + controller->getSteeringMu() * full_q_mat_target_frame_z;

  if(!grad.empty())
    {
      for(int i = 0; i < x.size(); i++) grad.at(i) = lambda_coefficient(i);
    }

  double ret = -controller->getSteeringMu() * controller->getRobotMassForOpt() * controller->getGravityForOpt();
  for(int i = 0; i < x.size(); i++) ret += (lambda_coefficient(i) * x.at(i));

  return ret;
}


double yLowerBoundConstraint(const std::vector<double> &x, std::vector<double> &grad, void *ptr)
{
  RollingController *controller = reinterpret_cast<RollingController*>(ptr);

  Eigen::MatrixXd full_q_mat_target_frame_y = controller->getFullQTargetFrame().row(Y) * controller->getRobotMassForOpt();
  Eigen::MatrixXd full_q_mat_target_frame_z = controller->getFullQTargetFrame().row(Z) * controller->getRobotMassForOpt();

  Eigen::MatrixXd lambda_coefficient = full_q_mat_target_frame_y + controller->getSteeringMu() * full_q_mat_target_frame_z;

  if(!grad.empty())
    {
      for(int i = 0; i < x.size(); i++) grad.at(i) = lambda_coefficient(i);
    }

  double ret = -controller->getSteeringMu() * controller->getRobotMassForOpt() * controller->getGravityForOpt();
  for(int i = 0; i < x.size(); i++) ret += (lambda_coefficient(i) * x.at(i));

  return ret;
}

double yUpperBoundConstraint(const std::vector<double> &x, std::vector<double> &grad, void *ptr)
{
  RollingController *controller = reinterpret_cast<RollingController*>(ptr);

  Eigen::MatrixXd full_q_mat_target_frame_y = controller->getFullQTargetFrame().row(Y) * controller->getRobotMassForOpt();
  Eigen::MatrixXd full_q_mat_target_frame_z = controller->getFullQTargetFrame().row(Z) * controller->getRobotMassForOpt();

  Eigen::MatrixXd lambda_coefficient = -full_q_mat_target_frame_y + controller->getSteeringMu() * full_q_mat_target_frame_z;

  if(!grad.empty())
    {
      for(int i = 0; i < x.size(); i++) grad.at(i) = lambda_coefficient(i);
    }

  double ret = -controller->getSteeringMu() * controller->getRobotMassForOpt() * controller->getGravityForOpt();
  for(int i = 0; i < x.size(); i++) ret += (lambda_coefficient(i) * x.at(i));

  return ret;
}



void RollingController::slsqpSolve()
{
  int n_variables = 2 * motor_num_;
  double epsilon = 0.000001;

  target_wrench_acc_target_frame_.resize(6);
  target_wrench_acc_target_frame_.tail(3) = Eigen::Vector3d(pid_controllers_.at(ROLL).result(),
                                                           pid_controllers_.at(PITCH).result(),
                                                           pid_controllers_.at(YAW).result());

  KDL::Frame cog = robot_model_->getCog<KDL::Frame>();
  KDL::Frame contact_point_alined_to_cog = contact_point_alined_.Inverse() * cog;
  Eigen::Vector3d contact_point_alined_to_cog_p = aerial_robot_model::kdlToEigen(contact_point_alined_to_cog.p);
  Eigen::Matrix3d contact_point_alined_to_cog_p_skew = aerial_robot_model::skew(contact_point_alined_to_cog_p);
  Eigen::VectorXd gravity = robot_model_->getGravity3d();
  Eigen::Vector3d gravity_ang_acc_from_contact_point_alined = robot_model_->getMass() * contact_point_alined_to_cog_p_skew * gravity;

  target_wrench_acc_target_frame_.tail(3) = target_wrench_acc_target_frame_.tail(3) + gravity_ang_acc_from_contact_point_alined;
  robot_mass_for_opt_ = robot_model_->getMass();
  gravity_for_opt_ = robot_model_->getGravity()(Z);

  nlopt::opt nl_solver(nlopt::LD_SLSQP, n_variables);

  nl_solver.set_min_objective(slsqpMinObjective, this);
  nl_solver.add_inequality_constraint(rollLowerBoundConstraint, this, epsilon);
  nl_solver.add_inequality_constraint(rollUpperBoundConstraint, this, epsilon);
  nl_solver.add_inequality_constraint(pitchLowerBoundConstraint, this, epsilon);
  nl_solver.add_inequality_constraint(pitchUpperBoundConstraint, this, epsilon);
  nl_solver.add_inequality_constraint(yawLowerBoundConstraint, this, epsilon);
  nl_solver.add_inequality_constraint(yawUpperBoundConstraint, this, epsilon);
  nl_solver.add_inequality_constraint(zUpperBoundConstraint, this, epsilon);
  nl_solver.add_inequality_constraint(xLowerBoundConstraint, this, epsilon);
  nl_solver.add_inequality_constraint(xUpperBoundConstraint, this, epsilon);
  nl_solver.add_inequality_constraint(yLowerBoundConstraint, this, epsilon);
  nl_solver.add_inequality_constraint(yUpperBoundConstraint, this, epsilon);

  nl_solver.set_xtol_rel(1e-4);

  std::vector<double> x(n_variables);
  for(int i = 0; i < x.size(); i++)
    {
      x.at(i) = full_lambda_all_(i) + 0.1;
    }

  double minf;
  try
    {
      nlopt::result result = nl_solver.optimize(x, minf);
    }
  catch(std::exception &e)
    {
      std::cout << "nlopt failed: " << e.what() << std::endl;
    }
}
