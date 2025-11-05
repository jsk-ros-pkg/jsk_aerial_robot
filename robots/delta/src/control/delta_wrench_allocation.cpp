#include <delta/control/delta_wrench_allocation.h>
#include <nlopt.hpp>

using namespace aerial_robot_control;

double nonlinearWrenchAllocationMinObjective(const std::vector<double>& x, std::vector<double>& grad, void* ptr)
{
  // 0 ~ motor_num : lambda
  // motor_num ~ 2 * motor_num : gimbal roll (phi)

  DeltaController* controller = reinterpret_cast<DeltaController*>(ptr);
  auto robot_model_for_control = controller->getRobotModelForControl();

  controller->setNLOptIterations(controller->getNLOptIterations() + 1);

  double ret = 0;
  int motor_num = robot_model_for_control->getRotorNum();

  /* compute result */
  // f = sum(lambda_i * lambda_i)
  for (int i = 0; i < motor_num; i++)
  {
    ret += x.at(i) * x.at(i);
  }

  if (grad.empty())
    return ret;

  /* make gradient */
  // df/dlambda_i = 2 * lambda_i
  // df/dphi_i = 0
  for (int i = 0; i < motor_num; i++)
  {
    grad.at(i) = 2 * x.at(i);
    grad.at(i + motor_num) = 0;
  }
  return ret;
}

void nonlinearWrenchAllocationEqConstraints(unsigned m, double* result, unsigned n, const double* x, double* grad,
                                            void* ptr)
{
  // 0 ~ 6 : Q * lambda - target_wrench_cog = 0

  DeltaController* controller = reinterpret_cast<DeltaController*>(ptr);
  auto robot_model_for_control = controller->getRobotModelForControl();

  int motor_num = robot_model_for_control->getRotorNum();

  std::vector<Eigen::Matrix3d> links_rotation_from_cog =
      robot_model_for_control->getLinksRotationFromCog<Eigen::Matrix3d>();
  std::vector<Eigen::Vector3d> rotors_origin_from_cog =
      robot_model_for_control->getRotorsOriginFromCog<Eigen::Vector3d>();
  std::vector<double> rotor_tilt = controller->getRotorTilt();

  Eigen::VectorXd target_acc_cog = controller->getTargetAccCog();
  double mass = robot_model_for_control->getMass();
  Eigen::Matrix3d inertia = robot_model_for_control->getInertia<Eigen::Matrix3d>();
  double m_f_rate = robot_model_for_control->getMFRate();
  std::map<int, int> rotor_direction = robot_model_for_control->getRotorDirection();

  Eigen::VectorXd target_wrench_cog = target_acc_cog;
  target_wrench_cog.head(3) = mass * target_wrench_cog.head(3);
  target_wrench_cog.tail(3) = inertia * target_wrench_cog.tail(3);

  Eigen::VectorXd lambda = Eigen::VectorXd::Zero(motor_num);
  for (int i = 0; i < motor_num; i++)
    lambda(i) = x[i];

  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(6, motor_num);
  for (int i = 0; i < motor_num; i++)
  {
    Eigen::Vector3d u_Li;
    u_Li << sin(rotor_tilt.at(i)), -sin(x[i + motor_num]) * cos(rotor_tilt.at(i)),
        cos(x[i + motor_num]) * cos(rotor_tilt.at(i));

    // cog_u_Li = cog_R_Li * u_Li
    Q.block(0, i, 3, 1) = links_rotation_from_cog.at(i) * u_Li;
    Q.block(3, i, 3, 1) = (aerial_robot_model::skew(rotors_origin_from_cog.at(i)) +
                           m_f_rate * rotor_direction.at(i + 1) * Eigen::MatrixXd::Identity(3, 3)) *
                          links_rotation_from_cog.at(i) * u_Li;
  }

  Eigen::VectorXd wrench_diff = Q * lambda - target_wrench_cog;
  for (int i = 0; i < m; i++)
    result[i] = wrench_diff(i);

  if (grad == NULL)
    return;

  /* make gradient */
  // thrust part. dwrench_dlambda = Q */
  for (int i = 0; i < 6; i++)
  {
    for (int j = 0; j < motor_num; j++)
    {
      grad[i * n + j] = Q(i, j);
    }
  }

  /* gimbal part */
  Eigen::MatrixXd df_dphi = Eigen::MatrixXd::Zero(3, motor_num);
  Eigen::MatrixXd dtau_dphi = Eigen::MatrixXd::Zero(3, motor_num);
  for (int i = 0; i < motor_num; i++)
  {
    Eigen::Vector3d du_Li_dphi;
    du_Li_dphi << 0, -cos(x[i + motor_num]) * cos(rotor_tilt.at(i)), -sin(x[i + motor_num]) * cos(rotor_tilt.at(i));

    df_dphi.block(0, i, 3, 1) = links_rotation_from_cog.at(i) * du_Li_dphi * x[i];
    dtau_dphi.block(0, i, 3, 1) = (aerial_robot_model::skew(rotors_origin_from_cog.at(i)) +
                                   m_f_rate * rotor_direction.at(i + 1) * Eigen::MatrixXd::Identity(3, 3)) *
                                  links_rotation_from_cog.at(i) * du_Li_dphi * x[i];
  }

  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < motor_num; j++)
    {
      grad[i * n + (j + motor_num)] = df_dphi(i, j);
      grad[(i + 3) * n + (j + motor_num)] = dtau_dphi(i, j);
    }
  }
}
