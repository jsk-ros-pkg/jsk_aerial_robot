// -*- mode: c++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <delta/design/optimal_design.h>

int design_cnt = 0;
int modeling_cnt = 0;

double mass_ = 3.3;
double max_thrust_ = 19.0;
double fc_f_min_weight_ = 1.0;
double fc_t_min_weight_ = 1.0;
double m_f_rate_ = -0.0143;
int rotor_num_ = 3;
std::vector<double> theta_(rotor_num_);

void calcRotorConfiguration(const std::vector<double>& phi, const std::vector<double>& theta, const double m_f_rate, std::vector<Eigen::Vector3d>& p, std::vector<Eigen::Vector3d>& u, std::vector<Eigen::Vector3d>& v, std::vector<double>& direct)
{
  p.clear();
  u.clear();
  v.clear();
  direct.clear();


  for(int i = 0; i < rotor_num_; i++)
    {
      direct.at(i) = 2 * ((i + 1) % 2) - 1;
    }


  // determine the origin point by the type of calclation

  // x: [ang1, ang2] * n/2


  // http://fnorio.com/0098spherical_trigonometry1/spherical_trigonometry1.html
}


// temporarily copy https://github.com/jsk-ros-pkg/jsk_aerial_robot/blob/master/aerial_robot_model/src/transformable_aerial_robot_model/stability.cpp#L33-L40
double calcTripleProduct(const Eigen::Vector3d& ui, const Eigen::Vector3d& uj, const Eigen::Vector3d& uk)
{
  Eigen::Vector3d uixuj = ui.cross(uj);
  if (uixuj.norm() < 0.00001) {
    return 0.0;
  }
  return uixuj.dot(uk) / uixuj.norm();
}

double calcFeasibleControlFDists(const std::vector<Eigen::Vector3d>& u, const double thrust_max, const double mass)
{
  const int rotor_num = u.size();
  Eigen::VectorXd fc_f_dists = Eigen::VectorXd::Zero(rotor_num * (rotor_num - 1));
  Eigen::Vector3d gravity_force(0, 0, mass * 9.8); // considering the gravity

  int index = 0;
  for (int i = 0; i < rotor_num; ++i)
    {
      const Eigen::Vector3d& u_i = u.at(i);
      for (int j = 0; j < rotor_num; ++j)
        {
          if (i == j) continue;
          const Eigen::Vector3d& u_j = u.at(j);
          double dist_ij = 0.0;

          if(u.at(i).cross(u.at(j)).norm() < 1e-5)
            {
              // assume u_i and u_j has same direction, so this is not plane can generate
              dist_ij = 1e6;
            }

          for (int k = 0; k < rotor_num; ++k)
            {
              if (i == k || j == k) continue;
              const Eigen::Vector3d& u_k = u.at(k);
              double u_triple_product = calcTripleProduct(u_i, u_j, u_k);
              dist_ij += std::max(0.0, u_triple_product * thrust_max);
            }
          Eigen::Vector3d uixuj = u_i.cross(u_j);
          fc_f_dists(index) = fabs(dist_ij - (uixuj.dot(gravity_force) / uixuj.norm())); // considering the gravity
          index++;
        }
    }
  return  fc_f_dists.minCoeff();
}

double calcFeasibleControlTDists(const std::vector<Eigen::Vector3d>& v, const double thrust_max)
{
  const int rotor_num = v.size();
  Eigen::VectorXd fc_t_dists = Eigen::VectorXd::Zero(rotor_num * (rotor_num - 1));

  int index = 0;
  for (int i = 0; i < rotor_num; ++i)
    {
      const Eigen::Vector3d& v_i = v.at(i);
      for (int j = 0; j < rotor_num; ++j)
        {
          if (i == j) continue;
          const Eigen::Vector3d& v_j = v.at(j);
          double dist_ij = 0.0;

          if(v.at(i).cross(v.at(j)).norm() < 1e-5)
            {
              // assume v_i and v_j has same direction, so this is not plane can generate
              dist_ij = 1e6;
            }
          else
            {
              for (int k = 0; k < rotor_num; ++k)
                {
                  if (i == k || j == k) continue;
                  const Eigen::Vector3d& v_k = v.at(k);
                  double v_triple_product = calcTripleProduct(v_i, v_j, v_k);
                  dist_ij += std::max(0.0, v_triple_product * thrust_max);
                }
            }
          fc_t_dists(index) = dist_ij;
          index++;
        }
    }
  //std::cout << fc_t_dists.transpose() << std::endl;
  return fc_t_dists.minCoeff();
}

double objectiveDesignFunc(const std::vector<double> &x, std::vector<double> &grad, void *ptr)
{
  // x is theta that is tilt angle
  for(int i = 0; i < x.size(); i++)
    {
      theta_.at(i) = x.at(i);
    }

  // solve optimization problem for gimbal angles that maximize control wrench
  nlopt::opt optimizer_solver(nlopt::GN_ISRES, rotor_num_); // chose the proper optimization model
  optimizer_solver.set_max_objective(objectiveModelingFunc, NULL); // register "this" as the second arg in objectiveFunc
  //set bounds
  std::vector<double> lb(rotor_num_);
  std::vector<double> ub(rotor_num_);
  for(int i = 0; i < rotor_num_; i++)
    {
      lb.at(i) = -M_PI;
      ub.at(i) = M_PI;
    }
  optimizer_solver.set_lower_bounds(lb);
  optimizer_solver.set_upper_bounds(ub);

  optimizer_solver.add_inequality_constraint(fzConstraint, NULL, 1e-8);
  optimizer_solver.add_inequality_constraint(fxfyTConstraint, NULL, 1e-8);

  optimizer_solver.set_xtol_rel(1e-4); //1e-4

  int max_eval = 10000;
  optimizer_solver.set_maxeval(max_eval);

  std::vector<double> opt_x(rotor_num_, 0.01);
  double max_val;
  bool search_flag = true;
  if (search_flag)
    {
      nlopt::result result = optimizer_solver.optimize(opt_x, max_val);
      if (result != nlopt::SUCCESS) std::cout << "the optimize solution does not succeed, result is " << result << std::endl;;
    }

  std::vector<Eigen::Vector3d> p;
  std::vector<Eigen::Vector3d> u;
  std::vector<Eigen::Vector3d> v;
  std::vector<double> direct;

  calcRotorConfiguration(opt_x, theta_, m_f_rate_, p, u, v, direct);

  double fc_f_min = calcFeasibleControlFDists(u, max_thrust_, mass_);
  double fc_t_min = calcFeasibleControlTDists(v, max_thrust_);

  if(design_cnt % 10000 == 0)
    {
      std::stringstream ss;
      ss << "tilt angles: " << x[0] << ", " << x[1] << ", " << x[2] << "/n";
      std::cout << ss.str() << std::endl;
    }

  design_cnt++;

  return fc_f_min_weight_ * fc_f_min + fc_t_min_weight_ * fc_t_min;
}


double objectiveModelingFunc(const std::vector<double> &x, std::vector<double> &grad, void *ptr)
{
  // x is phi that is gimbal angle

  // get the p, u, v and direction
  std::vector<Eigen::Vector3d> p;
  std::vector<Eigen::Vector3d> u;
  std::vector<Eigen::Vector3d> v;
  std::vector<double> direct;

  calcRotorConfiguration(x, theta_, m_f_rate_, p, u, v, direct);

  // get the fc_f_min and fc_t_min
  double fc_f_min = calcFeasibleControlFDists(u, max_thrust_, mass_);
  double fc_t_min = calcFeasibleControlTDists(v, max_thrust_);

  // penalty against rotor angle
  double penalty_rate = 0.0;
  double angle_penalty =  0;
  for(int i; i < x.size(); i++)
    {
      double penalty_i =  abs(x[i]) * penalty_rate ;
      angle_penalty += penalty_i;
    }

  if (modeling_cnt % 10000 == 0) {
    std::stringstream ss;
    ss << "fc_f_min: " << fc_f_min << "; fc_t_min: " << fc_t_min << "/n";
    std::cout << ss.str() << std::endl;
    std::cout << "angle_penalty: " << angle_penalty << std::endl;
    std::cout << "value: " << (fc_f_min_weight_ * fc_f_min + fc_t_min_weight_ * fc_t_min + angle_penalty) << std::endl;
  }

  modeling_cnt++;

  return fc_f_min_weight_ * fc_f_min + fc_t_min_weight_ * fc_t_min + angle_penalty;
}

// set the  bounds by constrains of unit rotor fource
double fzConstraint(const std::vector<double> &x, std::vector<double> &grad, void *ptr)
{
  return (9.8 * mass_ - (cos(x[0]) + cos(x[2]) + cos(x[4]) + cos(x[6]))  * max_thrust_);
}

// set the  bounds by constrains of unit rotor torque
double fxfyTConstraint(const std::vector<double> &x, std::vector<double> &grad, void *ptr)
{
  // OptimalDesign *planner = reinterpret_cast<OptimalDesign*>(ptr);

  // get the p, u, v and direction
  std::vector<Eigen::Vector3d> p;
  std::vector<Eigen::Vector3d> u;
  std::vector<Eigen::Vector3d> v;
  std::vector<double> direct;

  calcRotorConfiguration(x, theta_, m_f_rate_, p, u, v, direct);

  double fx_sum = 0;
  double fy_sum = 0;
  double tx_sum = 0;
  double ty_sum = 0;
  double tz_sum = 0;
  for(int i = 0; i< rotor_num_; i++){
    fx_sum += u[i][0] * max_thrust_ ;
    fy_sum += u[i][1] * max_thrust_ ;
    tx_sum += v[i][0] * max_thrust_ ;
    ty_sum += v[i][1] * max_thrust_ ;
    tz_sum += v[i][2] * max_thrust_ ;
  }
  return ( (pow(fx_sum, 2) + pow(fy_sum,2) + pow(tx_sum,2) + pow(ty_sum,2) + pow(tz_sum,2)) - 0.01);
}


int main (int argc, char **argv)
{
  ros::init (argc, argv, "aerial_robot_base");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~"); // node handle with private namespace

  // design the optimiazation problem
  nlopt::opt optimizer_solver(nlopt::GN_ISRES, rotor_num_); // chose the proper optimization model
  optimizer_solver.set_max_objective(objectiveDesignFunc, NULL); // register "this" as the second arg in objectiveFunc
  //set bounds
  std::vector<double> lb(rotor_num_);
  std::vector<double> ub(rotor_num_);
  for(int i = 0; i < rotor_num_; i++)
    {
      lb.at(i) = -M_PI/3;
      ub.at(i) = M_PI/3;
    }
  optimizer_solver.set_lower_bounds(lb);
  optimizer_solver.set_upper_bounds(ub);
  optimizer_solver.set_xtol_rel(1e-4); //1e-4
  int max_eval;
  nhp.param("max_eval", max_eval, 100000);
  optimizer_solver.set_maxeval(max_eval);

  std::vector<double> opt_x(rotor_num_);
  opt_x.at(0) = 0.1;
  opt_x.at(1) = -0.1;
  opt_x.at(2) = 0.1;

  bool search_flag = true;
  double max_val;
  if (search_flag)
    {
      nlopt::result result = optimizer_solver.optimize(opt_x, max_val);
      if (result != nlopt::SUCCESS) ROS_WARN_STREAM("the optimize solution does not succeed, result is " << result);
    }

  std::cout << "result: ";
  for(int i = 0; i < opt_x.size(); i++)
    {
      std::cout << opt_x.at(0) << " ";
    }
  std::cout << std::endl;

  return 0;
}
