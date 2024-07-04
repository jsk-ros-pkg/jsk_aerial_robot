#include <ros/ros.h>
#include <aerial_robot_control/trajectory/trajectory_reference/polynomial.hpp>

int main(int argc, char **argv)
{
  ros::init (argc, argv, "trajectory_test");

  double start_time = 2.0, duration = 1.0;
  double dt = 0.02;

  agi::Polynomial poly = agi::Polynomial(11, agi::Vector<3>(0, 0, 1), 2);
  poly.scale(start_time, duration);
  poly.addConstraint(start_time, agi::Vector<3>(1, 0, 0));
  poly.addConstraint(start_time + duration, agi::Vector<3>(2, 0, 0));
  poly.solve();

  agi::Vector<> x = agi::Vector<>::Zero(3);
  for(double t = start_time; t <= start_time + duration; t+=dt)
    {
      poly.eval(t, x);
      std::cout << std::setw(6) << std::left << t << ": ";
      for(int i = 0; i < x.size(); i++)
        {
          std::cout << std::setw(12) << std::left << x(i);
        }
      std::cout << std::endl;
    }

  return 0;

}
