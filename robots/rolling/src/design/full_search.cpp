#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>
#include <math.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseArray.h>
#include <unistd.h>
#include <time.h>

int theta_count_max = 20;
int phi_count_max = 10;
int thrust_count_max = 10;

double mass = 3.3;
double max_thrust = 18.0;
double m_f_rate = -0.0143;
double gravity = 9.81;
int rotor_num = 3;
double theta_max = M_PI / 3.0;
double phi_max = M_PI / 3.0;

std::vector<double> theta(rotor_num);
std::vector<double> phi(rotor_num);
std::vector<double> max_theta(rotor_num);

double fc_t_min = 0.0;
double fc_t_min_max = 0.0;

std::ofstream ofs;

void calcRotorConfiguration(const std::vector<double>& phi, const std::vector<double>& theta, const double m_f_rate, std::vector<Eigen::Vector3d>& p, std::vector<Eigen::Vector3d>& u, std::vector<Eigen::Vector3d>& v, std::vector<double>& direct)
{
  p.clear();
  u.clear();
  v.clear();
  direct.clear();

  for(int i = 0; i < rotor_num; i++)
    {
      direct.push_back(2 * ((i + 1) % 2) - 1);
    }

  p.push_back(Eigen::Vector3d(-0.101, 0.137, 0.0));
  p.push_back(Eigen::Vector3d(-0.069, -0.153, 0.0));
  p.push_back(Eigen::Vector3d(0.168, 0.016, 0.0));

  Eigen::Matrix3d rot_mat;
  Eigen::Vector3d axis = Eigen::Vector3d(0.0, 0.0, 1.0);
  Eigen::Vector3d tmp;

  rot_mat = Eigen::AngleAxisd(-2.0 / 3.0 * M_PI, axis);
  tmp = Eigen::Vector3d(sin(theta.at(0)), cos(theta.at(0)) * sin(phi.at(0)), cos(theta.at(0)) * cos(phi.at(0)));
  u.push_back(rot_mat * tmp);

  rot_mat = Eigen::AngleAxisd(0, axis);
  tmp = Eigen::Vector3d(sin(theta.at(1)), cos(theta.at(1)) * sin(phi.at(1)), cos(theta.at(1)) * cos(phi.at(1)));
  u.push_back(rot_mat * tmp);


  rot_mat = Eigen::AngleAxisd(2.0 / 3.0 * M_PI, axis);
  tmp = Eigen::Vector3d(sin(theta.at(2)), cos(theta.at(2)) * sin(phi.at(2)), cos(theta.at(2)) * cos(phi.at(2)));
  u.push_back(rot_mat * tmp);

  v.push_back(p.at(0).cross(u.at(0)) + m_f_rate * direct.at(0) * u.at(0));
  v.push_back(p.at(1).cross(u.at(1)) + m_f_rate * direct.at(1) * u.at(1));
  v.push_back(p.at(2).cross(u.at(2)) + m_f_rate * direct.at(2) * u.at(2));

  // determine the origin point by the type of calclation

  // x: [ang1, ang2] * n/2

  // http://fnorio.com/0098spherical_trigonometry1/spherical_trigonometry1.html
}

void thrustSearch()
{
  std::vector<Eigen::Vector3d> p;
  std::vector<Eigen::Vector3d> u;
  std::vector<Eigen::Vector3d> v;
  std::vector<double> direct;
  calcRotorConfiguration(phi, theta, m_f_rate, p, u, v, direct);

  double min_tau_x = 1e6;
  double max_tau_x = -1e6;
  double min_tau_y = 1e6;
  double max_tau_y = -1e6;
  double min_tau_z = 1e6;
  double max_tau_z = -1e6;

  for(int i = 0; i < thrust_count_max + 1; i++)
    {
      for(int j = 0; j < thrust_count_max + 1; j++)
        {
          for(int k = 0; k < thrust_count_max + 1; k++)
            {
              double thrust0 = (double)i / (double)thrust_count_max * max_thrust;
              double thrust1 = (double)j / (double)thrust_count_max * max_thrust;
              double thrust2 = (double)k / (double)thrust_count_max * max_thrust;

              Eigen::Vector3d force  = u.at(0) * thrust0 + u.at(1) * thrust1 + u.at(2) * thrust2;
              Eigen::Vector3d torque = v.at(0) * thrust0 + v.at(1) * thrust1 + v.at(2) * thrust2;

              if(fabs(force(0)) < 0.5 && fabs(force(1)) < 0.5 && force(2) > mass * 9.8)
                {
                  min_tau_x = std::min(min_tau_x, torque(0));
                  max_tau_x = std::max(max_tau_x, torque(0));
                  min_tau_y = std::min(min_tau_y, torque(1));
                  max_tau_y = std::max(max_tau_y, torque(1));
                  min_tau_z = std::min(min_tau_z, torque(2));
                  max_tau_z = std::max(max_tau_z, torque(2));

                  double fc_t_x_min = std::min(fabs(min_tau_x), fabs(max_tau_x));
                  double fc_t_y_min = std::min(fabs(min_tau_y), fabs(max_tau_y));
                  double fc_t_z_min = std::min(fabs(min_tau_z), fabs(max_tau_z));
                  fc_t_min = std::max(fc_t_min, std::min(fc_t_x_min, std::min(fc_t_y_min, fc_t_z_min)));
                }
            }
        }
    }

if(fc_t_min > fc_t_min_max)
  {
    fc_t_min_max = fc_t_min;
    max_theta.at(0) = theta.at(0);
    max_theta.at(1) = theta.at(1);
    max_theta.at(2) = theta.at(2);
  }

}

int main(int argc, char **argv)
{
  time_t t = time(NULL);
  std::string filename = std::string("full_search") + std::to_string(t) + std::string(".txt");
  ofs.open(filename, std::ios::out);
  ofs << "theta0 theta1 theta2 fc_t_min" << std::endl;

  for(int i = 0; i < theta_count_max + 1; i++)
    {
      for(int j = 0; j < theta_count_max + 1; j++)
        {
          for(int k = 0; k < theta_count_max + 1; k++)
            {
              fc_t_min = 0.0;
              bool plus = false;
              bool minus = false;
              theta.at(0) = (double)i  / theta_count_max * theta_max;
              theta.at(1) = -(double)j / theta_count_max * theta_max;
              theta.at(2) = (double)k  / theta_count_max * theta_max;
              if(theta.at(0) > 0.0) plus = true;
              if(theta.at(0) < 0.0) minus = true;
              if(theta.at(1) > 0.0) plus = true;
              if(theta.at(1) < 0.0) minus = true;
              if(theta.at(2) > 0.0) plus = true;
              if(theta.at(2) < 0.0) minus = true;
              if(!plus || !minus) continue;

              for(int l = 0; l < phi_count_max + 1; l++)
                {
                  for(int m = 0; m < phi_count_max + 1; m++)
                    {
                      for(int n = 0; n < phi_count_max + 1; n++)
                        {
                          phi.at(0) = (double)l / phi_count_max * 2.0 * phi_max - phi_max;
                          phi.at(1) = (double)m / phi_count_max * 2.0 * phi_max - phi_max;
                          phi.at(2) = (double)n / phi_count_max * 2.0 * phi_max - phi_max;

                          thrustSearch();
                        }
                    }
                }
              std::cout << "theta = " << theta.at(0) << " " << theta.at(1) << " " << theta.at(2) << " fc_t_min: " << fc_t_min << std::endl;;
              ofs << theta.at(0) << " " << theta.at(1) << " " << theta.at(2) << " " << fc_t_min << std::endl;;
            }
        }
    }

  std::cout << "optimized theta = " << max_theta.at(0) << " " << max_theta.at(1) << " " << max_theta.at(2) << " fc_t_min: " << fc_t_min_max << std::endl;;
  ofs << "optimized theta = " << max_theta.at(0) << " " << max_theta.at(1) << " " << max_theta.at(2) << " fc_t_min: " << fc_t_min_max << std::endl;;

  return 0;
}
