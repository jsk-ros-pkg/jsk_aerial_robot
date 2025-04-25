#include <aerial_robot_dynamics/robot_model_test.h>
#include <ros/ros.h>

int main (int argc, char** argv)
{
  ros::init(argc, argv, "test");
  ros::NodeHandle nh;

  bool verbose = false;
  if(argc > 1)
    {
      std::string arg = argv[1];
      if(arg != "0")
        verbose = true;
    }

  aerial_robot_dynamics::PinocchioRobotModelTest robot_model_test;

  // Test the robot model
  // FD with thrust and its test
  for(int i = 0; i < 3; i++)
    {
      std::cout << "-------------------------" << std::endl;
      bool fd_test = robot_model_test.forwardDynamicsTest(verbose);
      std::cout << "forwardDynamicsTest: " << fd_test << std::endl;
      std::cout << "-------------------------" << std::endl;
      std::cout << std::endl;
    }

  // FD derivative with thrust and its test
  for(int i = 0; i < 3; i++)
    {
      std::cout << "-------------------------" << std::endl;
      bool fd_derivarives_test = robot_model_test.forwardDynamicsDerivativesTest(verbose);
      std::cout << "forwardDynamicsDerivativesTest: " << fd_derivarives_test << std::endl;
      std::cout << "-------------------------" << std::endl;
      std::cout << std::endl;
    }


  for(int i = 0; i < 3; i++)
    {
      std::cout << "-------------------------" << std::endl;
      bool id_test = robot_model_test.inverseDynamicsTest(verbose);
      std::cout << "inverseDynamicsTest: " << id_test << std::endl;
      std::cout << "-------------------------" << std::endl;
      std::cout << std::endl;
    }


  for(int i = 0; i < 3; i++)
    {
      std::cout << "-------------------------" << std::endl;
      bool id_derivarives_test = robot_model_test.inverseDynamicsDerivativesTest(verbose);
      std::cout << "inverseDynamicsDerivativesTest: " << id_derivarives_test << std::endl;
      std::cout << "-------------------------" << std::endl;
      std::cout << std::endl;
    }

    for(int i = 0; i < 3; i++)
    {
      std::cout << "-------------------------" << std::endl;
      bool tauext_by_thrust_derivative_test = robot_model_test.computeTauExtByThrustDerivativeQDerivativesTest(verbose);
      std::cout << "computeTauExtPartialThrustPartialQTest: " << tauext_by_thrust_derivative_test << std::endl;
      std::cout << "-------------------------" << std::endl;
      std::cout << std::endl;
    }

  return 0;
}
