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
  //P : mapping from thrust(u) to acceleration(y) y = Pu-G
  P_ = Eigen::MatrixXd::Zero(4, getRotorNum());

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
  const std::vector<Eigen::Vector3d> rotors_origin_from_cog = getRotorsOriginFromCog<Eigen::Vector3d>();
  const int rotor_num = getRotorNum();
  const auto rotor_direction = getRotorDirection();

  Eigen::VectorXd g(4);
  g << 0, 0, 9.80665, 0;
  Eigen::VectorXd p_x(rotor_num), p_y(rotor_num), p_c(rotor_num), p_m(rotor_num);

  for(int i = 0; i < getRotorNum(); i++)
    {
      p_y(i) =  rotors_origin_from_cog[i](1);
      p_x(i) = -rotors_origin_from_cog[i](0);
      p_c(i) =  rotor_direction.at(i + 1) * m_f_rate_ ;
      p_m(i) =  1.0 / getMass();
      if(verbose)
        std::cout << "link" << i + 1 <<"origin :\n" << rotors_origin_from_cog[i] << std::endl;
    }

  Eigen::MatrixXd P_att = Eigen::MatrixXd::Zero(3, rotor_num);
  P_att.row(0) = p_y;
  P_att.row(1) = p_x;
  P_att.row(2) = p_c;

  P_att = getInertia<Eigen::Matrix3d>().inverse() * P_att;
  if(verbose)
    std::cout << "links_inertia inverse:"  << std::endl << getInertia<Eigen::Matrix3d>().inverse() << std::endl;

  /* roll, pitch, alt, yaw */
  P_.row(0) = P_att.row(0);
  P_.row(1) = P_att.row(1);
  P_.row(2) = p_m;
  P_.row(3) = P_att.row(2);

  if(control_verbose)
    std::cout << "P_:"  << std::endl << P_ << std::endl;

  ros::Time start_time = ros::Time::now();
  /* lagrange mothod */
  // issue: min x_t * x; constraint: g = P_ * x  (stable point)
  //lamda: [4:0]
  // x = P_t * lamba
  // (P_  * P_t) * lamda = g
  // x = P_t * (P_ * P_t).inv * g
  Eigen::FullPivLU<Eigen::MatrixXd> solver((P_ * P_.transpose()));
  Eigen::VectorXd lamda;
  lamda = solver.solve(g);
  optimal_hovering_f_ = P_.transpose() * lamda;

  p_det_ = (P_ * P_.transpose()).determinant();
  if(control_verbose)
    std::cout << "P det:"  << std::endl << p_det_ << std::endl;

  if(control_verbose)
    ROS_INFO("P solver is: %f\n", ros::Time::now().toSec() - start_time.toSec());

  if(optimal_hovering_f_.maxCoeff() > f_max_ || optimal_hovering_f_.minCoeff() < f_min_ || p_det_ < p_det_thre_ || only_three_axis_mode_)
    {
      lqi_mode_ = LQI_THREE_AXIS_MODE;

      //no yaw constraint
      Eigen::MatrixXd P_dash = Eigen::MatrixXd::Zero(3, rotor_num);
      P_dash.row(0) = P_.row(0);
      P_dash.row(1) = P_.row(1);
      P_dash.row(2) = P_.row(2);
      Eigen::VectorXd g3(3);
      g3 << 0, 0, 9.8;
      Eigen::FullPivLU<Eigen::MatrixXd> solver((P_dash * P_dash.transpose()));
      Eigen::VectorXd lamda;
      lamda = solver.solve(g3);
      optimal_hovering_f_ = P_dash.transpose() * lamda;
      if(control_verbose)
        std::cout << "three axis mode: optimal_hovering_f_:"  << std::endl << optimal_hovering_f_ << std::endl;

      /* calculate the P_orig(without inverse inertia) peusdo inverse */
      P_dash.row(0) = p_y;
      P_dash.row(1) = p_x;
      P_dash.row(2) = p_m;
      Eigen::MatrixXd P_dash_pseudo_inverse = P_dash.transpose() * (P_dash * P_dash.transpose()).inverse();
      P_orig_pseudo_inverse_ = Eigen::MatrixXd::Zero(rotor_num, 4);
      P_orig_pseudo_inverse_.block(0, 0, rotor_num, 3) = P_dash_pseudo_inverse;
      if(control_verbose)
        std::cout << "P orig_pseudo inverse for three axis mode:"  << std::endl << P_orig_pseudo_inverse_ << std::endl;

      /* if we do the 4dof underactuated control */
      if(!only_three_axis_mode_) return false;

      p_det_ = (P_dash * P_dash.transpose()).determinant();

      /* if we only do the 3dof control, we still need to check the steady state validation */
      if(optimal_hovering_f_.maxCoeff() > f_max_ || optimal_hovering_f_.minCoeff() < f_min_
         || p_det_ < p_det_thre_)
        return false;

      return true;
    }

  /* calculate the P_orig(without inverse inertia) pseudo inverse */
  Eigen::MatrixXd P_dash = Eigen::MatrixXd::Zero(4, rotor_num);
  P_dash.row(0) = p_y;
  P_dash.row(1) = p_x;
  P_dash.row(2) = p_m;
  P_dash.row(3) = p_c;

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
