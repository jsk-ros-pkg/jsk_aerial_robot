#include <aerial_robot_dynamics/robot_model_test.h>
#include <aerial_robot_dynamics/math_utils.h>
#include <chrono>

using namespace aerial_robot_dynamics;

bool PinocchioRobotModelTest::computeTauExtByThrustDerivativeQDerivativesTest(bool verbose)
{
  Eigen::VectorXd q = robot_model_->getResetConfiguration();

  std::vector<Eigen::MatrixXd> tauext_partial_thrust_partial_q_ana = robot_model_->computeTauExtByThrustDerivativeQDerivatives(q); // compute analytical derivatives
  std::vector<Eigen::MatrixXd> tauext_partial_thrust_partial_q_num = robot_model_->computeTauExtByThrustDerivativeQDerivativesNum(q); // compute numerical derivatives

  if(verbose)
    {
      for(int i = 0; i < tauext_partial_thrust_partial_q_ana.size(); i++)
        {
          std::cout << "tauext_partial_thrust_partial_q_ana[" << i << "]" << std::endl;
          std::cout << tauext_partial_thrust_partial_q_ana.at(i) << std::endl;
          std::cout << "tauext_partial_thrust_partial_q_num[" << i << "]" << std::endl;
          std::cout << tauext_partial_thrust_partial_q_num.at(i) << std::endl;
        }
    }

  bool ok = true;
  for(int i = 0; i < robot_model_->getModel()->nv; i++)
    {

      if((tauext_partial_thrust_partial_q_ana.at(i) - tauext_partial_thrust_partial_q_num.at(i)).cwiseAbs().maxCoeff() > 1e-4)
        {
          std::cout << "tauext_partial_thrust_partial_q_ana[" << i << "] is not equal to tauext_partial_thrust_partial_q_num[" << i << "]" << std::endl;
          ok = false;
        }
    }
  return ok;
}
