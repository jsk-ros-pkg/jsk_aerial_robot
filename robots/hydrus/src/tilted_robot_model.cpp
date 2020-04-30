#include <hydrus/tilted_robot_model.h>

HydrusTiltedRobotModel::HydrusTiltedRobotModel(bool init_with_rosparam, bool verbose, double epsilon, double control_margin_thre, double p_det_thre):
  HydrusRobotModel(init_with_rosparam, verbose, epsilon, control_margin_thre, p_det_thre, false)
{
}

void HydrusTiltedRobotModel::updateStatics(bool verbose)
{
  const int rotor_num = getRotorNum();
  const auto rotor_direction = getRotorDirection();

  std::vector<Eigen::Vector3d> rotors_origin(rotor_num);
  std::vector<Eigen::Vector3d> rotors_normal(rotor_num);

  auto rotors_origin_from_cog = getRotorsOriginFromCog<KDL::Vector>();
  auto rotors_normal_from_cog = getRotorsNormalFromCog<KDL::Vector>();
  auto cog_desire_orientation = getCogDesireOrientation<KDL::Rotation>();
  for(int i = 0; i < rotor_num; ++i)
    {
      /* {CoG'}: same origin with {CoG}, but the orientation (axes alignment) is same with {B} (baselink) */
      rotors_origin.at(i) = aerial_robot_model::kdlToEigen(cog_desire_orientation.Inverse() * rotors_origin_from_cog.at(i));
      rotors_normal.at(i) = aerial_robot_model::kdlToEigen(cog_desire_orientation.Inverse() * rotors_normal_from_cog.at(i));
    }
  auto link_inertia_cog_dash = (cog_desire_orientation.Inverse() * KDL::RigidBodyInertia(getMass(), KDL::Vector::Zero(), getInertia<KDL::RotationalInertia>())).getRotationalInertia();

  Eigen::VectorXd g(4);
  g << 0, 0, 0, 9.806650; // rotational motion (x,y,z) + translational motion (z)

  for (unsigned int i = 0; i < rotor_num; i++) {
    Q_f_.col(i) = rotors_normal.at(i);
    Q_tau_.col(i) = rotors_origin.at(i).cross(rotors_normal.at(i)) + rotor_direction.at(i + 1)  * getMFRate() * rotors_normal.at(i);
  }

  P_.block(0, 0, 3, rotor_num) = aerial_robot_model::kdlToEigen(link_inertia_cog_dash).inverse() * Q_tau_;
  P_.block(3, 0, 3, rotor_num) = Q_f_ / getMass();

  ros::Time start_time = ros::Time::now();
  /* lagrange mothod
   issue: min x_t * x; constraint: y = A_ * x  (stable point)
   lamda: [4:0]
   x = A_t * lamba
   (A_  * A_t) * lamda = y
   x = A_t * (A_ * A_t).inv * y */

  Eigen::MatrixXd P_four_axis = P_.block(0, 0, 4, rotor_num);
  P_four_axis.row(3) = P_.row(5); // the forth element is z
  Eigen::FullPivLU<Eigen::MatrixXd> solver((P_four_axis * P_four_axis.transpose()));
  Eigen::VectorXd lamda, optimal_hovering_f;
  lamda = solver.solve(g);
  optimal_hovering_f = P_four_axis.transpose() * lamda;
  p_det_ = (P_four_axis * P_four_axis.transpose()).determinant();

  if(verbose)
    {
      std::cout << "P_:"  << std::endl << P_ << std::endl;
      std::cout << "P_four_axis det:"  << std::endl << p_det_ << std::endl;
      std::cout << "optimal_hovering_f: " << optimal_hovering_f.transpose() << " vs " << optimal_hovering_f_.transpose() << std::endl;
      ROS_INFO("P solver is: %f\n", ros::Time::now().toSec() - start_time.toSec());
    }

  /* special process to find the hovering axis for tilt model */
  Eigen::VectorXd f = Q_f_ * optimal_hovering_f;
  double f_norm_roll = atan2(f(1), f(2));
  double f_norm_pitch = atan2(-f(0), sqrt(f(1)*f(1) + f(2)*f(2)));
  /* set the hoverable frame as CoG for the flight control */
  setCogDesireOrientation(f_norm_roll, f_norm_pitch, 0);
  Eigen::Matrix3d r;
  r = Eigen::AngleAxisd(f_norm_pitch, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(f_norm_roll, Eigen::Vector3d::UnitX());

  if(verbose)
    {
      std::cout << "f_norm_pitch: " << f_norm_pitch << "; f_norm_roll: " << f_norm_roll << std::endl;
      std::cout << "transformed force: \n" << (r * f).transpose() << std::endl;
    }

  P_.block(0, 0, 3, rotor_num) = r * P_.block(0, 0, 3, rotor_num);
  P_.block(3, 0, 3, rotor_num) = r * P_.block(3, 0, 3, rotor_num);

  if(verbose) std::cout << "transformed P_:"  << std::endl << P_ << std::endl;

  P_four_axis.block(0, 0, 3, rotor_num) = P_.block(0, 0, 3, rotor_num);
  P_four_axis.row(3) = P_.row(5);
  p_det_ = (P_four_axis * P_four_axis.transpose()).determinant();
  if(verbose) std::cout << "P four axis det:"  << std::endl << p_det_ << std::endl;

  /* rescaling hovering force */
  optimal_hovering_f_ = optimal_hovering_f * (9.806650 * getMass() / f.norm());

  if(verbose)
    std::cout << "rescaled optimal_hovering_f: " << optimal_hovering_f_.transpose() << std::endl;

  if(p_det_ < p_det_thre_)
    {
      ROS_ERROR("the determinant of P four axis is too small: %f [%f]", p_det_, p_det_thre_);
      lqi_mode_ = 0;
      return;
    }

  /* calculate the P_orig pseudo inverse */
  Eigen::MatrixXd P_dash = P_four_axis;
  P_dash.block(0, 0, 3, rotor_num) = r * Q_tau_; // without inverse inertia!
  P_orig_pseudo_inverse_ = aerial_robot_model::pseudoinverse(P_dash);
  if(verbose)
    std::cout << "P orig pseudo inverse for four axis mode:" << std::endl << P_orig_pseudo_inverse_ << std::endl;

  lqi_mode_ = LQI_THREE_AXIS_MODE;
}




