#include <hydrus/hydrus_robot_model.h>

HydrusRobotModel::HydrusRobotModel(bool init_with_rosparam, bool verbose, double thrust_max, double thrust_min, double m_f_rate, double stability_margin_thre, double p_det_thre, bool only_three_axis_mode):
  RobotModel(init_with_rosparam, verbose, thrust_max, thrust_min, m_f_rate),
  stability_margin_thre_(stability_margin_thre),
  p_det_thre_(p_det_thre),
  only_three_axis_mode_(only_three_axis_mode),
  p_det_(0),
  stability_margin_(0)
{
  if (init_with_rosparam)
    {
      getParamFromRos();
    }

  Q_f_ = Eigen::MatrixXd::Zero(3, getRotorNum());
  Q_tau_ = Eigen::MatrixXd::Zero(3, getRotorNum());
  P_ = Eigen::MatrixXd::Zero(6, getRotorNum());

  lqi_mode_ = LQI_FOUR_AXIS_MODE;
}

void HydrusRobotModel::getParamFromRos()
{
  ros::NodeHandle nhp("~");
  nhp.param("only_three_axis_mode", only_three_axis_mode_, false);
  nhp.param("stability_margin_thre", stability_margin_thre_, 0.01);
  nhp.param("p_determinant_thre", p_det_thre_, 1e-6);
}

bool HydrusRobotModel::modelling(bool verbose, bool control_verbose)
{
  const std::vector<Eigen::Vector3d> rotors_origin = getRotorsOriginFromCog<Eigen::Vector3d>();
  const std::vector<Eigen::Vector3d> rotors_normal = getRotorsNormalFromCog<Eigen::Vector3d>();

  const int rotor_num = getRotorNum();
  const auto rotor_direction = getRotorDirection();

  Eigen::VectorXd g(4);
  g << 0, 0, 0, 9.806650; // rotational motion (x,y,z) + translational motion (z)

  for (unsigned int i = 0; i < rotor_num; i++) {
    Q_f_.col(i) = rotors_normal.at(i);
    Q_tau_.col(i) = rotors_origin.at(i).cross(rotors_normal.at(i)) + rotor_direction.at(i + 1) * getMFRate() * rotors_normal.at(i);
  }

  P_.block(0, 0, 3, rotor_num) = getInertia<Eigen::Matrix3d>().inverse() * Q_tau_;
  P_.block(3, 0, 3, rotor_num) = Q_f_ / getMass();

  ros::Time start_time = ros::Time::now();
  /* lagrange mothod */
  // issue: min x_t * x; constraint: y = A_ * x  (stable point)
  //lamda: [4:0]
  // x = A_t * lamba
  // (A_  * A_t) * lamda = y
  // x = A_t * (A_ * A_t).inv * y
  Eigen::MatrixXd P_four_axis = P_.block(0, 0, 4, rotor_num);
  P_four_axis.row(3) = P_.row(5); // the forth element is z
  Eigen::FullPivLU<Eigen::MatrixXd> solver((P_four_axis * P_four_axis.transpose()));
  Eigen::VectorXd lamda, optimal_hovering_f;
  lamda = solver.solve(g);
  optimal_hovering_f_ = P_four_axis.transpose() * lamda;
  p_det_ = (P_four_axis * P_four_axis.transpose()).determinant();

  if(control_verbose)
    {
      std::cout << "P_:"  << std::endl << P_ << std::endl;
      std::cout << "P det:"  << std::endl << p_det_ << std::endl;
      std::cout << "optimal_hovering_f: " << optimal_hovering_f.transpose() << " vs " << optimal_hovering_f_.transpose() << std::endl;
      ROS_INFO("P solver is: %f\n", ros::Time::now().toSec() - start_time.toSec());
    }

  if(optimal_hovering_f_.maxCoeff() > getThrustUpperLimit() || optimal_hovering_f_.minCoeff() < getThrustLowerLimit() || p_det_ < p_det_thre_ || only_three_axis_mode_)
    {
      lqi_mode_ = LQI_THREE_AXIS_MODE;

      Eigen::MatrixXd P_three_axis = P_.block(0, 0, 3, rotor_num);
      P_three_axis.row(2) = P_.row(5); // the third element is z

      /* calculate the P_orig peusdo inverse */
      Eigen::MatrixXd P_dash = Q_tau_; // caution: without inverse inertia to calculate P_dash_pseudo_inverse
      P_dash.row(2) = P_.row(5);
      Eigen::MatrixXd P_dash_pseudo_inverse = aerial_robot_model::pseudoinverse(P_dash);
      P_orig_pseudo_inverse_ = Eigen::MatrixXd::Zero(rotor_num, 4);
      P_orig_pseudo_inverse_.block(0, 0, rotor_num, 2) = P_dash_pseudo_inverse.block(0, 0, rotor_num, 2);
      P_orig_pseudo_inverse_.col(3) = P_dash_pseudo_inverse.col(2);
      if(control_verbose)
        std::cout << "P orig pseudo inverse for three axis mode:"  << std::endl << P_orig_pseudo_inverse_ << std::endl;

      /* if we do the 4dof underactuated control */
      if(!only_three_axis_mode_) return false;

      Eigen::FullPivLU<Eigen::MatrixXd> solver((P_three_axis * P_three_axis.transpose()));
      Eigen::VectorXd lamda;
      lamda = solver.solve(g.tail(3));
      optimal_hovering_f_ = P_three_axis.transpose() * lamda;
      p_det_ = (P_three_axis * P_three_axis.transpose()).determinant();

      if(control_verbose)
        std::cout << "three axis mode: optimal_hovering_f_:"  << std::endl << optimal_hovering_f_ << std::endl;

      /* if we only do the 3dof control, we still need to check the steady state validation */
      if(optimal_hovering_f_.maxCoeff() > getThrustUpperLimit() || optimal_hovering_f_.minCoeff() < getThrustLowerLimit() || p_det_ < p_det_thre_)
        return false;

      return true;
    }

  /* calculate the P_orig pseudo inverse */
  Eigen::MatrixXd P_dash = P_four_axis;
  P_dash.block(0, 0, 3, rotor_num) = Q_tau_; // without inverse inertia!
  P_orig_pseudo_inverse_ = aerial_robot_model::pseudoinverse(P_dash);

  if(control_verbose)
    std::cout << "P orig pseudo inverse for four axis mode:" << std::endl << P_orig_pseudo_inverse_ << std::endl;

  if(control_verbose || verbose)
    std::cout << "four axis mode optimal_hovering_f_:"  << std::endl << optimal_hovering_f_ << std::endl;

  lqi_mode_ = LQI_FOUR_AXIS_MODE;

  return true;
}

bool HydrusRobotModel::stabilityMarginCheck(bool verbose)
{
  double average_x = 0, average_y = 0;

  const std::vector<Eigen::Vector3d> rotors_origin_from_cog = getRotorsOriginFromCog<Eigen::Vector3d>();
  const int rotor_num = getRotorNum();

  /* calcuate the average */
  for(int i = 0; i < rotor_num; i++)
    {
      average_x += rotors_origin_from_cog[i](0);
      average_y += rotors_origin_from_cog[i](1);
      if(verbose)
        ROS_INFO("rotor%d x: %f, y: %f", i + 1, rotors_origin_from_cog[i](0), rotors_origin_from_cog[i](1));
    }
  average_x /= rotor_num;
  average_y /= rotor_num;

  double s_xy = 0, s_xx = 0, s_yy = 0;
  for(const auto& rotor_pos : rotors_origin_from_cog)
    {
      double x_diff = rotor_pos(0) - average_x;
      double y_diff = rotor_pos(1) - average_y;
      s_xy += (x_diff * y_diff);
      s_xx += (x_diff * x_diff);
      s_yy += (y_diff * y_diff);
    }

  Eigen::Matrix2d S;
  S << s_xx / rotor_num, s_xy / rotor_num, s_xy / rotor_num, s_yy / rotor_num;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(S);

  assert(getLinkLength() > 0);
  stability_margin_ = sqrt(es.eigenvalues()[0]) / getLinkLength();
  if(verbose) ROS_INFO("stability_margin: %f", stability_margin_);
  if(stability_margin_ < stability_margin_thre_) return false;
  return true;
}
