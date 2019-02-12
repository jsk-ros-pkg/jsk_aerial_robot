#include <hydrus/hydrus_robot_model.h>

HydrusRobotModel::HydrusRobotModel(bool init_with_rosparam, bool verbose, std::string baselink, std::string thrust_link, double stability_margin_thre, double p_det_thre, double f_max, double f_min, double m_f_rate, bool only_three_axis_mode):
  RobotModel(init_with_rosparam, verbose, baselink, thrust_link),
  stability_margin_thre_(stability_margin_thre),
  p_det_thre_(p_det_thre),
  f_max_(f_max),
  f_min_(f_min),
  m_f_rate_(m_f_rate_),
  only_three_axis_mode_(only_three_axis_mode),
  p_det_(0),
  stability_margin_(0)
{
  if (init_with_rosparam)
    {
      getParamFromRos();
    }
  P_ = Eigen::MatrixXd::Zero(6, getRotorNum());

  lqi_mode_ = LQI_FOUR_AXIS_MODE;
}

void HydrusRobotModel::getParamFromRos()
{
  ros::NodeHandle nhp("~");
  nhp.param("only_three_axis_mode", only_three_axis_mode_, false);
  nhp.param ("stability_margin_thre", stability_margin_thre_, 0.01);
  if(getVerbose()) std::cout << "stability margin thre: " << std::setprecision(3) << stability_margin_thre_ << std::endl;
  nhp.param ("p_determinant_thre", p_det_thre_, 1e-6);
  if(getVerbose()) std::cout << "p determinant thre: " << std::setprecision(3) << p_det_thre_ << std::endl;
  nhp.param ("f_max", f_max_, 8.6);
  if(getVerbose()) std::cout << "f_max: " << std::setprecision(3) << f_max_ << std::endl;
  nhp.param ("f_min", f_min_, 2.0);
  if(getVerbose()) std::cout << "f_min: " << std::setprecision(3) << f_min_ << std::endl;
  ros::NodeHandle control_node("/motor_info");
  control_node.param("m_f_rate", m_f_rate_, 0.01);
  if(getVerbose()) std::cout << "m_f_rate: " << std::setprecision(3) << m_f_rate_ << std::endl;
}

bool HydrusRobotModel::modelling(bool verbose, bool control_verbose)
{
  const std::vector<Eigen::Vector3d> rotors_origin = getRotorsOriginFromCogDash<Eigen::Vector3d>();
  const std::vector<Eigen::Vector3d> rotors_normal = getRotorsNormalFromCogDash<Eigen::Vector3d>();

  const int rotor_num = getRotorNum();
  const auto rotor_direction = getRotorDirection();

  Eigen::VectorXd g(4);
  g << 0, 0, 0, 9.806650 * getMass(); // rotational motion (x,y,z) + translational motion (z)

  Eigen::MatrixXd Q_tau(3, rotor_num), Q_f(3, rotor_num);
  for (unsigned int i = 0; i < rotor_num; i++) {
    Q_f.col(i) = rotors_normal.at(i);
    Q_tau.col(i) = rotors_origin.at(i).cross(rotors_normal.at(i)) + rotor_direction.at(i + 1) * m_f_rate_ * rotors_normal.at(i);
  }

  Eigen::MatrixXd Q_four_axis = Eigen::MatrixXd::Zero(4, rotor_num);
  Q_four_axis.block(0, 0, 3, rotor_num) = Q_tau;
  Q_four_axis.row(3) = Q_f.row(2);

  ros::Time start_time = ros::Time::now();
  /* lagrange mothod */
  // issue: min x_t * x; constraint: g = P_ * x  (stable point)
  //lamda: [4:0]
  // x = P_t * lamba
  // (P_  * P_t) * lamda = g
  // x = P_t * (P_ * P_t).inv * g
  Eigen::FullPivLU<Eigen::MatrixXd> solver((Q_four_axis * Q_four_axis.transpose()));
  Eigen::VectorXd lamda, optimal_hovering_f;
  lamda = solver.solve(g);
  optimal_hovering_f = Q_four_axis.transpose() * lamda;

  //std::cout << "tau : \n" << (Q_tau * optimal_hovering_f).transpose() << std::endl;
  //std::cout << "force : \n" << (Q_f * optimal_hovering_f).transpose() << std::endl;

  Eigen::VectorXd f = Q_f * optimal_hovering_f;
  double f_norm_roll = atan2(f(1), f(2));
  double f_norm_pitch = atan2(-f(0), sqrt(f(1)*f(1) + f(2)*f(2)));
  setCogDesireOrientation(f_norm_roll, f_norm_pitch, 0); // set the hoverable frame as CoG to do the control
  //std::cout << "hovering force : \n" << (Eigen::AngleAxisd(f_norm_pitch, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(f_norm_roll, Eigen::Vector3d::UnitX()) * f).transpose() << std::endl;
  //ROS_INFO("f_norm_pitch: %f, f_norm_roll: %f", f_norm_pitch, f_norm_roll);

  Eigen::Matrix3d r;
  r = Eigen::AngleAxisd(f_norm_pitch, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(f_norm_roll, Eigen::Vector3d::UnitX());
  P_.block(0, 0, 3, rotor_num) = getInertia<Eigen::Matrix3d>().inverse() * r * Q_tau;
  P_.block(3, 0, 3, rotor_num) = r * Q_f / getMass();

  if(control_verbose) std::cout << "P_:"  << std::endl << P_ << std::endl;

  Eigen::MatrixXd P_four_axis = Q_four_axis;
  P_four_axis.block(0, 0, 3, rotor_num) = P_.block(0, 0, 3, rotor_num);
  P_four_axis.row(3) = P_.row(5);
  p_det_ = (P_four_axis * P_four_axis.transpose()).determinant();
  if(control_verbose) std::cout << "P det:"  << std::endl << p_det_ << std::endl;

  optimal_hovering_f_ = optimal_hovering_f * (9.806650 * getMass() / f.norm());

  //std::cout << "optimal_hovering_f: " << optimal_hovering_f.transpose() << " vs " << optimal_hovering_f_.transpose() << std::endl;

  if(control_verbose)
    ROS_INFO("P solver is: %f\n", ros::Time::now().toSec() - start_time.toSec());

  if(optimal_hovering_f_.maxCoeff() > f_max_ || optimal_hovering_f_.minCoeff() < f_min_ || p_det_ < p_det_thre_ || only_three_axis_mode_)
    {
      lqi_mode_ = LQI_THREE_AXIS_MODE;

      Eigen::MatrixXd Q_three_axis = Eigen::MatrixXd::Zero(3, rotor_num);;
      Q_three_axis.row(0) = Q_tau.row(0);
      Q_three_axis.row(1) = Q_tau.row(1);
      Q_three_axis.row(2) = Q_f.row(2);
      Eigen::VectorXd g3(3);
      g3 << 0, 0, 9.8 * getMass();

      Eigen::FullPivLU<Eigen::MatrixXd> solver((Q_three_axis * Q_three_axis.transpose()));
      Eigen::VectorXd lamda;
      lamda = solver.solve(g3);
      optimal_hovering_f_ = Q_three_axis.transpose() * lamda;
      if(control_verbose)
        std::cout << "three axis mode: optimal_hovering_f_:"  << std::endl << optimal_hovering_f_ << std::endl;

      /* calculate the P_orig(without inverse inertia) peusdo inverse */
      Eigen::MatrixXd P_dash = Q_three_axis;
      P_dash.row(2) = Q_f.row(2) / getMass();
      Eigen::MatrixXd P_dash_pseudo_inverse = P_dash.transpose() * (P_dash * P_dash.transpose()).inverse();
      P_orig_pseudo_inverse_ = Eigen::MatrixXd::Zero(rotor_num, 4);
      P_orig_pseudo_inverse_.block(0, 0, rotor_num, 2) = P_dash_pseudo_inverse.block(0, 0, rotor_num, 2);
      P_orig_pseudo_inverse_.col(3) = P_dash_pseudo_inverse.col(2);
      if(control_verbose)
        std::cout << "P orig_pseudo inverse for three axis mode:"  << std::endl << P_orig_pseudo_inverse_ << std::endl;

      /* if we do the 4dof underactuated control */
      if(!only_three_axis_mode_) return false;

      P_dash.block(0, 0, 2, rotor_num) = P_.block(0, 0, 2, rotor_num);
      p_det_ = (P_dash * P_dash.transpose()).determinant();

      /* if we only do the 3dof control, we still need to check the steady state validation */
      if(optimal_hovering_f_.maxCoeff() > f_max_ || optimal_hovering_f_.minCoeff() < f_min_ || p_det_ < p_det_thre_)
        return false;

      if (f(0) != 0 || f(1) != 0)
        {
          ROS_ERROR("the three axis control mode does not support for the tilted rotor model");
          return false;
        }
      return true;
    }

  /* calculate the P_orig(without inverse inertia) pseudo inverse */
  Eigen::MatrixXd P_dash = Q_four_axis;
  P_dash.row(3) = Q_f.row(2) / getMass();
  if(control_verbose)
    std::cout << "P dash:" << std::endl << P_dash << std::endl;

  P_orig_pseudo_inverse_ = P_dash.transpose() * (P_dash * P_dash.transpose()).inverse();
  if(control_verbose)
    std::cout << "P orig_pseudo inverse for four axis mode:" << std::endl << P_orig_pseudo_inverse_ << std::endl;

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
