#include <delta/design/delta_optimal_design.h>

DeltaOptimalDesign::DeltaOptimalDesign(ros::NodeHandle nh, ros::NodeHandle nhp):
  nh_(nh),
  nhp_(nhp)
{
  getParam<int>(nhp_, "rotor_num", rotor_num_, 0);
  getParam<double>(nhp_, "mass", mass_, 0.0);
  getParam<double>(nhp_, "max_thrust", max_thrust_, 0.0);
  getParam<double>(nhp_, "theta_max", theta_max_, 0.0);
  getParam<double>(nhp_, "m_f_rate", m_f_rate_, 0.0);
  getParam<double>(nhp_, "fct_min_weight", fct_min_weight_, 0.0);
  getParam<double>(nhp_, "gimbal_penalty_weight", gimbal_penalty_weight_, 0.0);

  eval_cnt_ = 0;
  direction_.resize(rotor_num_);
  direction_.at(0) = 1;
  direction_.at(1) = -1;
  direction_.at(2) = 1;
}

std::vector<Eigen::Vector3d> DeltaOptimalDesign::calcRotorConfiguration(const std::vector<double>& theta)
{
  theta_.resize(theta.size());
  for(int i = 0; i < theta_.size(); i++)
    {
      theta_.at(i) = theta.at(i);
    }

  std::vector<Eigen::Vector3d> p(0), u(0), v(0);

  p.push_back(Eigen::Vector3d(-0.10356573848447463, 0.1420213347603282, -0.0067415938016600905));
  p.push_back(Eigen::Vector3d(-0.057934779219482105, -0.140915700598086, -0.0067415938016600905));
  p.push_back(Eigen::Vector3d(0.16690982019541284, 0.03679320895573279,   -0.0067415938016600905));

  Eigen::Matrix3d rot_mat;
  Eigen::Vector3d b3 = Eigen::Vector3d(0.0, 0.0, 1.0);
  Eigen::Vector3d x_axis, y_axis, z_axis;

  for(int i = 0; i < theta_.size(); i++)
    {
      rot_mat = Eigen::AngleAxisd((i - 1) * 2.0 / 3.0 * M_PI, b3); // -2/3pi, pi, 2/3pi
      x_axis = rot_mat * Eigen::Vector3d(sin(theta_.at(i)), 0, 0);
      y_axis = rot_mat * Eigen::Vector3d(0, cos(theta_.at(i)), 0);
      z_axis = rot_mat * Eigen::Vector3d(0, 0, cos(theta_.at(i)));

      u.push_back(x_axis);
      u.push_back(y_axis);
      u.push_back(z_axis);

      v.push_back(p.at(i).cross(x_axis) + m_f_rate_ * direction_.at(i) * x_axis);
      v.push_back(p.at(i).cross(x_axis) + m_f_rate_ * direction_.at(i) * x_axis);
      v.push_back(p.at(i).cross(y_axis) + m_f_rate_ * direction_.at(i) * y_axis);
      v.push_back(p.at(i).cross(z_axis) + m_f_rate_ * direction_.at(i) * z_axis);
      v.push_back(p.at(i).cross(-y_axis) + m_f_rate_ * direction_.at(i) * (-y_axis));
      v.push_back(p.at(i).cross(-z_axis) + m_f_rate_ * direction_.at(i) * (-z_axis));
    }
  return v;
}

double calcTripleProduct(const Eigen::Vector3d& ui, const Eigen::Vector3d& uj, const Eigen::Vector3d& uk)
{
  Eigen::Vector3d uixuj = ui.cross(uj);
  if (uixuj.norm() < 0.00001) {
    return 0.0;
  }
  return uixuj.dot(uk) / uixuj.norm();
}

double calcFeasibleControlTDists(const std::vector<Eigen::Vector3d>& v)
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
                  dist_ij += std::max(0.0, v_triple_product);
                }
            }
          fc_t_dists(index) = dist_ij;
          index++;
        }
    }
  return fc_t_dists.minCoeff();
}

double objectiveDesignFunc(const std::vector<double> &x, std::vector<double> &grad, void *ptr)
{
  /* x is gimbal angle */
  DeltaOptimalDesign *planner = reinterpret_cast<DeltaOptimalDesign*>(ptr);

  std::vector<Eigen::Vector3d> v = planner->calcRotorConfiguration(x);
  double gimbal_penalty = 0.0;
  for(int i = 0; i < x.size(); i++)
    {
      gimbal_penalty += x.at(i) * x.at(i);
    }

  double fct_min = calcFeasibleControlTDists(v);

  planner->eval_cnt_ = planner->eval_cnt_ + 1;
  if(planner->eval_cnt_ % 1000 == 0)
    {
      std::cout << "evaluation: " << planner->eval_cnt_ << " gimbal tilt: ";
      for(int i = 0; i < x.size(); i++)
        {
          std::cout << x.at(i) << " ";
        }
      std::cout << ". FCTmin: " << fct_min << ". gimbal penalty: " << gimbal_penalty << std::endl;
    }

  return planner->fct_min_weight_ * fct_min - planner->gimbal_penalty_weight_ * gimbal_penalty;
}

void DeltaOptimalDesign::run()
{
  // design the optimiazation problem
  nlopt::opt optimizer_solver(nlopt::GN_ISRES, rotor_num_); // chose the proper optimization model

  optimizer_solver.set_max_objective(objectiveDesignFunc, this); // register "this" as the second arg in objectiveFunc

  std::vector<double> lb(rotor_num_);
  std::vector<double> ub(rotor_num_);
  for(int i = 0; i < rotor_num_; i++)
    {
      lb.at(i) = -fabs(theta_max_);
      ub.at(i) = fabs(theta_max_);
    }
  optimizer_solver.set_lower_bounds(lb);
  optimizer_solver.set_upper_bounds(ub);
  optimizer_solver.set_xtol_rel(1e-4); //1e-4

  int max_eval;
  nhp_.param("max_eval", max_eval, 100000);
  optimizer_solver.set_maxeval(max_eval);

  std::vector<double> opt_x(rotor_num_, 0.0);
  opt_x.at(0) = -theta_max_;
  opt_x.at(1) =  theta_max_;
  opt_x.at(2) = -theta_max_;

  double max_val;
  nlopt::result result = optimizer_solver.optimize(opt_x, max_val);
  if (result != nlopt::SUCCESS) ROS_WARN_STREAM("the optimize solution does not succeed, result is " << result);
  std::cout << "evaluation: " << eval_cnt_ << ". gimbal tilt: ";
  for(int i = 0; i < rotor_num_; i++)
    {
      std::cout << opt_x.at(i) << " ";
    }
  std::cout << std::endl;
  std::cout << "objective: " << max_val << std::endl;
}

int main(int argc, char **argv)
{
  ros::init (argc, argv, "aerial_robot_base");
  ros::NodeHandle nh;
  ros::NodeHandle nhp("~"); // node handle with private namespace

  DeltaOptimalDesign* delta_optimal_design = new DeltaOptimalDesign(nh, nhp);
  delta_optimal_design->run();
  delete delta_optimal_design;
  return 0;
}
