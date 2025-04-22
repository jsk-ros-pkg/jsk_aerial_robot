#include <aerial_robot_dynamics/robot_model_test.h>
#include <aerial_robot_dynamics/math_utils.h>
#include <chrono>

using namespace aerial_robot_dynamics;

bool PinocchioRobotModelTest::inverseDynamicsDerivativesTest(bool verbose)
{
  Eigen::VectorXd q = robot_model_->getResetConfiguration();
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot_model_->getModel()->nv);
  Eigen::VectorXd a = Eigen::VectorXd::Zero(robot_model_->getModel()->nv);

  addNoise(v, 0.1);
  addNoise(a, 0.1);

  Eigen::MatrixXd id_partial_dq_ana = Eigen::MatrixXd::Zero(robot_model_->getModel()->nv + robot_model_->getRotorNum(), robot_model_->getModel()->nv);
  Eigen::MatrixXd id_partial_dv_ana = Eigen::MatrixXd::Zero(robot_model_->getModel()->nv + robot_model_->getRotorNum(), robot_model_->getModel()->nv);
  Eigen::MatrixXd id_partial_da_ana = Eigen::MatrixXd::Zero(robot_model_->getModel()->nv + robot_model_->getRotorNum(), robot_model_->getModel()->nv);

  auto start = std::chrono::high_resolution_clock::now();
  pinocchio::computeRNEADerivatives(*(robot_model_->getModel()), *(robot_model_->getData()), q, v, a);
  auto end = std::chrono::high_resolution_clock::now();
  std::cout << "RNEA Derivatives time: " << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0 << " ms" << std::endl;

  start = std::chrono::high_resolution_clock::now();
  robot_model_->inverseDynamicsDerivatives(q, v, a, id_partial_dq_ana, id_partial_dv_ana, id_partial_da_ana);
  end = std::chrono::high_resolution_clock::now();
  std::cout << "ID Derivatives time: " << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count() / 1000.0 << " ms" << std::endl;

  if(verbose)
    {
      std::cout << "q: " << std::endl;
      std::cout << q.transpose() << std::endl;
      std::cout << "v: " << std::endl;
      std::cout << v.transpose() << std::endl;
      std::cout << "a: " << std::endl;
      std::cout << a.transpose() << std::endl;
    }

  // compare with numerical derivative
  double epsilon = 1e-6;
  Eigen::VectorXd original_q = q;
  Eigen::VectorXd original_v = v;
  Eigen::VectorXd original_a = a;
  Eigen::VectorXd id_original_solution = robot_model_->inverseDynamics(original_q, original_v, original_a);

  // check partial_dq
  Eigen::MatrixXd id_partial_dq_num = Eigen::MatrixXd::Zero(robot_model_->getModel()->nv + robot_model_->getRotorNum(), robot_model_->getModel()->nv);
  for(int i = 0; i < 3; i++) // root link position
    {
      q = original_q;
      q(i) += epsilon;
      Eigen::VectorXd id_solution_plus = robot_model_->inverseDynamics(q, original_v, original_a);
      id_partial_dq_num.col(i) = (id_solution_plus - id_original_solution) / epsilon;
    }

  for(int i = 0; i < 3; i++) // root link quaternion
    {
      q = original_q;
      double d_roll = i == 0 ? epsilon : 0; Eigen::AngleAxisd roll(Eigen::AngleAxisd(d_roll, Eigen::Vector3d::UnitX()));
      double d_pitch = i == 1 ? epsilon : 0; Eigen::AngleAxisd pitch(Eigen::AngleAxisd(d_pitch, Eigen::Vector3d::UnitY()));
      double d_yaw = i == 2 ? epsilon : 0; Eigen::AngleAxisd yaw(Eigen::AngleAxisd(d_yaw, Eigen::Vector3d::UnitZ()));

      Eigen::Matrix3d dR = pinocchio::exp3(Eigen::Vector3d(d_roll, d_pitch, d_yaw));
      Eigen::Quaterniond dQuat(dR);
      Eigen::Quaterniond original_quat = Eigen::Quaterniond(original_q(6), original_q(3), original_q(4), original_q(5)); // w, x, y, z
      Eigen::Quaterniond new_quat = dQuat * original_quat;
      new_quat.normalize();

      q(3) = new_quat.x();
      q(4) = new_quat.y();
      q(5) = new_quat.z();
      q(6) = new_quat.w();

      Eigen::VectorXd id_solution_plus = robot_model_->inverseDynamics(q, original_v, original_a);
      id_partial_dq_num.col(i + 3) = (id_solution_plus - id_original_solution) / epsilon;
    }

  for(int i = 7; i < robot_model_->getModel()->nq; i++) // joint position
    {
      q = original_q;
      q(i) += epsilon;
      Eigen::VectorXd id_solution_plus = robot_model_->inverseDynamics(q, original_v, original_a);
      id_partial_dq_num.col(i - 1) = (id_solution_plus - id_original_solution) / epsilon;
    }

  // check partial_dv
  robot_model_->inverseDynamics(original_q, original_v, original_a);
  Eigen::MatrixXd id_partial_dv_num = Eigen::MatrixXd::Zero(robot_model_->getModel()->nv + robot_model_->getRotorNum(), robot_model_->getModel()->nv);
  for(int i = 0; i < robot_model_->getModel()->nv; i++)
    {
      v = original_v;
      v(i) += epsilon;
      Eigen::VectorXd id_solution_plus = robot_model_->inverseDynamics(original_q, v, original_a);
      id_partial_dv_num.col(i) = (id_solution_plus - id_original_solution) / epsilon;
    }

  // check partial_da
  robot_model_->inverseDynamics(original_q, original_v, original_a);
  Eigen::MatrixXd id_partial_da_num = Eigen::MatrixXd::Zero(robot_model_->getModel()->nv + robot_model_->getRotorNum(), robot_model_->getModel()->nv);
  for(int i = 0; i < robot_model_->getModel()->nv; i++)
    {
      a = original_a;
      a(i) += epsilon;
      Eigen::VectorXd id_solution_plus = robot_model_->inverseDynamics(original_q, original_v, a);
      id_partial_da_num.col(i) = (id_solution_plus - id_original_solution) / epsilon;
    }

  if(verbose)
    {
      std::cout << "diff of id_partial_dq:" << std::endl;
      std::cout << id_partial_dq_ana - id_partial_dq_num << std::endl;
      std::cout << "norm: " << (id_partial_dq_ana - id_partial_dq_num).norm() << " max diff: " << (id_partial_dq_ana - id_partial_dq_num).cwiseAbs().maxCoeff() << std::endl;

      std::cout << "diff of id_partial_dv:" << std::endl;
      std::cout << id_partial_dv_ana - id_partial_dv_num << std::endl;
      std::cout << "norm: " << (id_partial_dv_ana - id_partial_dv_num).norm() << " max diff: " << (id_partial_dv_ana - id_partial_dv_num).cwiseAbs().maxCoeff() << std::endl;

      std::cout << "diff of id_partial_da:" << std::endl;
      std::cout << id_partial_da_ana - id_partial_da_num << std::endl;
      std::cout << "norm: " << (id_partial_da_ana - id_partial_da_num).norm() << " max diff: " << (id_partial_da_ana - id_partial_da_num).cwiseAbs().maxCoeff() << std::endl;
    }

  return (id_partial_dq_ana - id_partial_dq_num).cwiseAbs().maxCoeff() < 1e-4 &&
                                                                         (id_partial_dv_ana - id_partial_dv_num).cwiseAbs().maxCoeff() < 1e-4 &&
                                                                                                                                         (id_partial_da_ana - id_partial_da_num).cwiseAbs().maxCoeff() < 1e-4;
}
