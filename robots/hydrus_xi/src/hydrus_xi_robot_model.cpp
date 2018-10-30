#include <hydrus_xi/hydrus_xi_robot_model.h>

HydrusXiRobotModel::HydrusXiRobotModel(bool init_with_rosparam, bool verbose, std::string baselink, std::string thrust_link, double stability_margin_thre, double p_det_thre, double f_max, double f_min, double m_f_rate, bool only_three_axis_mode) :
  HydrusRobotModel(init_with_rosparam, verbose, baselink, thrust_link, stability_margin_thre, p_det_thre, f_max, f_min, m_f_rate, only_three_axis_mode)
{
}

bool HydrusXiRobotModel::modelling(bool verbose, bool control_verbose) //override
{
  const std::vector<Eigen::Vector3d> rotors_origin = getRotorsOriginFromCog<Eigen::Vector3d>();
  const std::vector<Eigen::Vector3d> rotors_normal = getRotorsNormalFromCog<Eigen::Vector3d>();
  const int rotor_num = getRotorNum();
  const auto rotor_direction = getRotorDirection();

  Eigen::VectorXd g(4);
  g << 0, 0, 9.80665, 0;
  Eigen::VectorXd p_m(rotor_num);

  Eigen::MatrixXd Q_tau(3, rotor_num);
  for (unsigned int i = 0; i < rotor_num; i++) {
    p_m(i) = rotors_normal.at(i)(2) / getMass();
    Q_tau.col(i) = rotors_origin.at(i).cross(rotors_normal.at(i)) + rotor_direction.at(i + 1) * m_f_rate_ * rotors_normal.at(i);
    if(verbose)
      std::cout << "link" << i + 1 <<"origin :\n" << rotors_origin.at(i) << std::endl;
  }

  Eigen::MatrixXd P_att = getInertia<Eigen::Matrix3d>().inverse() * Q_tau;

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

  /* calculate the P_orig(without inverse inertia) pseudo inverse */
  Eigen::MatrixXd P_dash = Eigen::MatrixXd::Zero(4, rotor_num);
  P_dash.row(0) = Q_tau.row(0);
  P_dash.row(1) = Q_tau.row(1);
  P_dash.row(2) = p_m;
  P_dash.row(3) = Q_tau.row(2);

  P_orig_pseudo_inverse_ = P_dash.transpose() * (P_dash * P_dash.transpose()).inverse();
  if(control_verbose)
    std::cout << "P orig_pseudo inverse for four axis mode:" << std::endl << P_orig_pseudo_inverse_ << std::endl;

  if(control_verbose || verbose)
    std::cout << "four axis mode optimal_hovering_f_:"  << std::endl << optimal_hovering_f_ << std::endl;

  lqi_mode_ = LQI_FOUR_AXIS_MODE;

  if(optimal_hovering_f_.maxCoeff() > f_max_ || optimal_hovering_f_.minCoeff() < f_min_ || p_det_ < p_det_thre_)
     return false;

  return true;
}

//bool HydrusXiRobotModel::stabilityMarginCheck(bool verbose) //override
//{
  //implement for future
//  return true;
//}
