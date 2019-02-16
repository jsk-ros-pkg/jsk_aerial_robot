#include <hydrus_xi/hydrus_xi_robot_model.h>

HydrusXiRobotModel::HydrusXiRobotModel(bool init_with_rosparam, bool verbose, std::string baselink, std::string thrust_link, double stability_margin_thre, double p_det_thre, double f_max, double f_min, double m_f_rate, bool only_three_axis_mode) :
  HydrusRobotModel(init_with_rosparam, verbose, baselink, thrust_link, stability_margin_thre, p_det_thre, f_max, f_min, m_f_rate, only_three_axis_mode)
{
  makeJointThrustMap();
  link_joint_num_ = 6;
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

void HydrusXiRobotModel::addCogSegments()
{
  tree_with_cog_ = getTree();
  
}

Eigen::MatrixXd HydrusXiRobotModel::getCOGJacobian(const sensor_msgs::JointState& joint_state)
{

  
  std::vector<std::string> links;
  auto segments = tree.getSegments();

  for (int i = 0; i < link_joint_num_; ++i) {
    links.push_back(std::string("link") + std::to_string(i + 1));
  }
  for (int i = 0; i < getRotorNum(); ++i) {
    links.push_back(std::string("gimbal_link") + std::to_string(i + 1));
  }

  std::vector<Eigen::MatrixXd> jacobian_mass;
  for (int i = 0; i < links.size(); ++i) {
    auto seg = GetTreeElementSegment(segments.at(links[i]));
    auto jacobian = getJacobian(joint_state, links[i]);
    KDL::Vector cog = seg.getInertia().getCOG();
    double mass = seg.getInertia().getMass();
    KDL::Vector joint_axis = seg.getJoint().JointAxis();
    
    //    jacobian_mass.push_back(
  }



  auto seg = GetTreeElementSegment(segments.at("gimbal_link1"));
  
  std::cout << cog.x() << " " << cog.y() << " " << cog.z() << std::endl;

  KDL::Vector vec = seg.getJoint().JointAxis();
  std::cout << vec.x() << " " << vec.y() << " " << vec.z() << std::endl;

}

Eigen::MatrixXd HydrusXiRobotModel::getJacobian(const sensor_msgs::JointState& joint_state, std::string segment_name)
{
  const auto& tree = getTree();
  KDL::JntArray joint_positions = jointMsgToKdl(joint_state);
  KDL::TreeJntToJacSolver solver(tree);
  KDL::Jacobian jac(tree.getNrOfJoints());
  int status = solver.JntToJac(joint_positions, jac, segment_name);
  solver.JntToJac(joint_positions, jac, segment_name);
  return convertJacobian(jac.data);
}

inline Eigen::MatrixXd HydrusXiRobotModel::convertJacobian(const Eigen::MatrixXd& in)
{
  Eigen::MatrixXd out(6, link_joint_num_ + getRotorNum());
  const auto& actuator_map = getActuatorMap();
  for (int i = 0; i < link_joint_num_; ++i) {
    out.col(i) = in.col(actuator_map.at(std::string("joint") + std::to_string(i + 1)));
  }

  for (int i = 0; i < getRotorNum(); ++i) {
    out.col(i + link_joint_num_) = in.col(actuator_map.at(std::string("gimbal") + std::to_string(i + 1)));
  }
  return out;
}

Eigen::MatrixXd HydrusXiRobotModel::calcWrenchAllocationMatrix()
{
  const std::vector<Eigen::Vector3d> rotors_origin = getRotorsOriginFromCog<Eigen::Vector3d>();
  const std::vector<Eigen::Vector3d> rotors_normal = getRotorsNormalFromCog<Eigen::Vector3d>();
  const int rotor_num = getRotorNum();

  //Q : WrenchAllocationMatrix
  Eigen::MatrixXd Q(6, rotor_num);
  double uav_mass_inv = 1.0 / getMass();
  Eigen::Matrix3d inertia_inv = getInertia<Eigen::Matrix3d>().inverse();
  for (unsigned int i = 0; i < rotor_num; ++i) {
    Q.block(0, i, 3, 1) = rotors_normal.at(i) * uav_mass_inv;
    Q.block(3, i, 3, 1) = inertia_inv * (rotors_origin.at(i).cross(rotors_normal.at(i)));
  }

  return Q;
}

void HydrusXiRobotModel::makeJointThrustMap()
{
  joint_thrust_map_.clear();
  std::vector<std::string> thrust_list;
  auto segment_map = getTree().getSegments();
  auto actuator_map = getActuatorMap();
  for (const auto actuator : actuator_map) {
    std::vector<std::string> empty_vec;
    joint_thrust_map_[actuator.first] = empty_vec;
  }
  const auto root_tree_elem = getTree().getRootSegment();

  for (int i = 1; ; ++i) {
    std::string thrust_s = std::string("thrust") + std::to_string(i);
    auto thrust_tree_elem = segment_map.find(thrust_s);
    if (thrust_tree_elem == segment_map.end()) break;

    KDL::TreeElement& current_tree_elem = thrust_tree_elem->second;

    while (true) {
      auto parent_seg_map = GetTreeElementParent(thrust_tree_elem->second);
      if (parent_seg_map == root_tree_elem) break;
      auto parent_segment = GetTreeElementSegment(parent_seg_map->second);

      if (joint_thrust_map_.find(parent_segment.getJoint().getName()) != joint_thrust_map_.end()) {
        joint_thrust_map_.at(parent_segment.getJoint().getName()).push_back(thrust_s);
      }
            current_tree_elem = parent_seg_map->second;
    }
  }
}

std::vector<double> HydrusXiRobotModel::calcJointTorque()
{
  auto seg_tf_map = getSegmentsTf();

  auto Q_inv = aerial_robot_model::pseudoinverse(calcWrenchAllocationMatrix());
  Eigen::VectorXd g(6); g << 0, 0, 9.80665, 0, 0, 0;
  auto static_thrust = Q_inv * g; //[thrust1, thrust2, ..., thrustN]
  const auto rotors_normal_from_cog = getRotorsNormalFromCog<KDL::Vector>();
  const auto rotor_direction = getRotorDirection();

  std::vector<double> joint_torque; //[joint1, joint2, ..., jointN]

  for (unsigned int i = 1; ; ++i) {
    std::string joint_s = std::string("joint") + std::to_string(i);
    auto thrust_itr = joint_thrust_map_.find(joint_s);
    if (thrust_itr == joint_thrust_map_.end()) break;
    double trq = 0.0;
    for (auto t : thrust_itr->second) {
      auto r = seg_tf_map.at(t).p - seg_tf_map.at(std::string("link") + std::to_string(i)).p;
      t.erase(t.begin(), t.begin() + std::string("thrust").length());
      int idx = std::stoi(t) - 1;
      auto thrust = rotors_normal_from_cog.at(idx) * static_thrust[idx];
      double counter_moment = rotor_direction.at(idx + 1) * m_f_rate_ * thrust.z();
      trq += (-(r * thrust).z() - counter_moment);
    }
    joint_torque.push_back(trq);
  }

  return joint_torque;
}

//bool HydrusXiRobotModel::stabilityMarginCheck(bool verbose) //override
//{
  //implement for future
//  return true;
//}
