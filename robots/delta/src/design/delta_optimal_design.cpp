#include <delta/design/delta_optimal_design.h>

DeltaOptimalDesign::DeltaOptimalDesign(ros::NodeHandle nh, ros::NodeHandle nhp):
  nh_(nh),
  nhp_(nhp)
{
  // initialize robot model
  delta_robot_model_ = boost::make_shared<RollingRobotModel>();
  delta_robot_model_->updateRobotModel();

  // update robot model with normal joint angles
  sensor_msgs::JointState joint_state;
  joint_state.name = {"joint1", "joint2", "gimbal1", "gimbal2", "gimbal3"};
  joint_state.position.resize(joint_state.name.size());
  for(int i = 0; i < delta_robot_model_->getJointNum() - delta_robot_model_->getRotorNum(); i++)
    joint_state.position.at(i) = 2 * M_PI / delta_robot_model_->getRotorNum();
  delta_robot_model_->updateRobotModel(joint_state);

  // get robot model info
  rotor_direction_ = delta_robot_model_->getRotorDirection();
  rotors_origin_from_cog_ = delta_robot_model_->getRotorsOriginFromCog<Eigen::Vector3d>();

  // get optimization parameter
  getParam<double>(nhp_, "theta_max", theta_max_, 0.0);
  getParam<double>(nhp_, "fct_min_weight", fct_min_weight_, 0.0);
  getParam<double>(nhp_, "gimbal_penalty_weight", gimbal_penalty_weight_, 0.0);

  feasible_control_torque_convex_pub_ = nh_.advertise<geometry_msgs::PoseArray>("feasible_control_torque_convex", 1);
  feasible_control_torque_radius_pub_ = nh_.advertise<std_msgs::Float32>("feasible_control_torque_radius", 1);

  eval_cnt_ = 0;
}

std::vector<Eigen::Vector3d> DeltaOptimalDesign::calcRotorConfiguration(const std::vector<double>& theta)
{
  std::vector<Eigen::Vector3d> v(0);
  std::vector<Eigen::Matrix3d> links_rotation_from_cog = delta_robot_model_->getLinksRotationFromCog<Eigen::Matrix3d>();
  double m_f_rate = delta_robot_model_->getMFRate();
  Eigen::Vector3d x_axis, y_axis, z_axis;

  for(int i = 0; i < delta_robot_model_->getRotorNum(); i++)
    {
      Eigen::Matrix3d rot_mat = links_rotation_from_cog.at(i);

      x_axis = rot_mat * Eigen::Vector3d(sin(theta.at(i)), 0, 0);
      y_axis = rot_mat * Eigen::Vector3d(0, cos(theta.at(i)), 0);
      z_axis = rot_mat * Eigen::Vector3d(0, 0, cos(theta.at(i)));

      v.push_back(rotors_origin_from_cog_.at(i).cross(x_axis) + m_f_rate * rotor_direction_.at(i + 1) * x_axis);
      v.push_back(rotors_origin_from_cog_.at(i).cross(y_axis) + m_f_rate * rotor_direction_.at(i + 1) * y_axis);
      v.push_back(rotors_origin_from_cog_.at(i).cross(z_axis) + m_f_rate * rotor_direction_.at(i + 1) * z_axis);
      v.push_back(rotors_origin_from_cog_.at(i).cross(-y_axis) + m_f_rate * rotor_direction_.at(i + 1) * (-y_axis));
      v.push_back(rotors_origin_from_cog_.at(i).cross(-z_axis) + m_f_rate * rotor_direction_.at(i + 1) * (-z_axis));
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
    gimbal_penalty += x.at(i) * x.at(i);

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
  int rotor_num = delta_robot_model_->getRotorNum();
  for(int i = 0; i < rotor_num; i++)
    std::cout << "rotor origin" << i + 1 << " " << rotors_origin_from_cog_.at(i).transpose() << std::endl;

  // design the optimiazation problem
  nlopt::opt optimizer_solver(nlopt::GN_ISRES, rotor_num); // chose the proper optimization model

  optimizer_solver.set_max_objective(objectiveDesignFunc, this); // register "this" as the second arg in objectiveFunc

  std::vector<double> lb(rotor_num);
  std::vector<double> ub(rotor_num);
  for(int i = 0; i < rotor_num; i++)
    {
      if(rotor_direction_.at(i + 1) < 0)
        {
          lb.at(i) = 0.0;
          ub.at(i) = fabs(theta_max_);
        }
      else
        {
          lb.at(i) = -fabs(theta_max_);
          ub.at(i) = 0.0;
        }
    }

  optimizer_solver.set_lower_bounds(lb);
  optimizer_solver.set_upper_bounds(ub);
  optimizer_solver.set_xtol_rel(1e-6);
  optimizer_solver.set_ftol_rel(1e-6);

  int max_eval;
  nhp_.param("max_eval", max_eval, 100000);
  optimizer_solver.set_maxeval(max_eval);

  std::vector<double> opt_x(rotor_num, 0.0);
  for(int i = 0; i < rotor_num; i++)
    opt_x.at(i) = - rotor_direction_.at(i + 1) * fabs(theta_max_);

  double max_val;
  nlopt::result result;
  try
    {
      result = optimizer_solver.optimize(opt_x, max_val);
    }
  catch(std::runtime_error error)
    {}

  if (result != nlopt::SUCCESS) ROS_WARN_STREAM("the optimize solution does not succeed, result is " << result);
  std::cout << "evaluation: " << eval_cnt_ << ". gimbal tilt: ";
  for(int i = 0; i < rotor_num; i++)
    std::cout << opt_x.at(i) << " ";
  std::cout << std::endl;
  std::cout << "objective: " << max_val << std::endl;

  // publish v in optimal state
  ros::Rate rate(1);
  while(ros::ok())
    {
      std::vector<Eigen::Vector3d> v = calcRotorConfiguration(opt_x);
      geometry_msgs::PoseArray feasible_control_torque_convex_msg;
      feasible_control_torque_convex_msg.poses.resize(0);
      for(int i = 0; i < v.size(); i++)
        {
          geometry_msgs::Pose pose_msg;
          pose_msg.position.x = delta_robot_model_->getThrustUpperLimit() * v.at(i)(0);
          pose_msg.position.y = delta_robot_model_->getThrustUpperLimit() * v.at(i)(1);
          pose_msg.position.z = delta_robot_model_->getThrustUpperLimit() * v.at(i)(2);
          feasible_control_torque_convex_msg.poses.push_back(pose_msg);
        }
      feasible_control_torque_convex_pub_.publish(feasible_control_torque_convex_msg);

      std_msgs::Float32 feasible_control_torque_radius_msg;
      feasible_control_torque_radius_msg.data = delta_robot_model_->getThrustUpperLimit() * calcFeasibleControlTDists(v);
      feasible_control_torque_radius_pub_.publish(feasible_control_torque_radius_msg);

      rate.sleep();
    }
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
