#include <aerial_robot_model/transformable_aerial_robot_model.h>

using namespace aerial_robot_model::transformable;

void RobotModel::resolveLinkLength()
{
  KDL::JntArray joint_positions(getTree().getNrOfJoints());
  //hard coding
  KDL::Frame f_link2 = forwardKinematics<KDL::Frame>("link2", joint_positions);
  KDL::Frame f_link3 = forwardKinematics<KDL::Frame>("link3", joint_positions);
  link_length_ = (f_link3.p - f_link2.p).Norm();
}


void RobotModel::calcBasicKinematicsJacobian()
{
  const std::vector<Eigen::Vector3d> p = getRotorsOriginFromCog<Eigen::Vector3d>();
  const std::vector<Eigen::Vector3d> u = getRotorsNormalFromCog<Eigen::Vector3d>();
  const auto& joint_positions = getJointPositions();
  const auto& sigma = getRotorDirection();
  const int rotor_num = getRotorNum();
  const double m_f_rate = getMFRate();

  //calc jacobian of u(thrust direction, force vector), p(thrust position)
  for (int i = 0; i < rotor_num; ++i) {
    std::string seg_name = std::string("thrust") + std::to_string(i + 1);
    Eigen::MatrixXd thrust_coord_jacobian = RobotModel::getJacobian(joint_positions, seg_name);
    thrust_coord_jacobians_.at(i) = thrust_coord_jacobian;
    u_jacobians_.at(i) = -skew(u.at(i)) * thrust_coord_jacobian.bottomRows(3);
    p_jacobians_.at(i) = thrust_coord_jacobian.topRows(3) - cog_jacobian_;
  }
}

void RobotModel::calcCoGMomentumJacobian()
{
  double mass_all = getMass();
  const auto cog_all = getCog<KDL::Frame>().p;
  const auto& segment_map = getTree().getSegments();
  const auto seg_frames = getSegmentsTf();
  const auto& inertia_map = getInertiaMap();
  const auto& joint_names = getJointNames();
  const auto& joint_segment_map = getJointSegmentMap();
  const auto& joint_parent_link_names = getJointParentLinkNames();
  const auto joint_num = getJointNum();

  Eigen::MatrixXd root_rot = aerial_robot_model::kdlToEigen(getCogDesireOrientation<KDL::Rotation>() * seg_frames.at(getBaselinkName()).M.Inverse());
  /*
    Note: the jacobian about the cog velocity (linear momentum) and angular momentum.

    Please refer to Eq.13 ~ 22 of following paper:
    ============================================================================
    S. Kajita et al., "Resolved momentum control: humanoid motion planning based on the linear and angular momentum,".
    ============================================================================

    1. the cog velocity is w.r.t in the root link frame.
    2. the angular momentum is w.r.t in cog frame
  */


  int col_index = 0;
  /* fix bug: the joint_segment_map is reordered, which is not match the order of  joint_indeices_ or joint_names_ */
  // joint part
  for (const auto& joint_name : joint_names){
    std::string joint_child_segment_name = joint_segment_map.at(joint_name).at(0);
    KDL::Segment joint_child_segment = GetTreeElementSegment(segment_map.at(joint_child_segment_name));
    KDL::Vector a = seg_frames.at(joint_parent_link_names.at(col_index)).M * joint_child_segment.getJoint().JointAxis();

    KDL::Vector r = seg_frames.at(joint_child_segment_name).p;
    KDL::RigidBodyInertia inertia = KDL::RigidBodyInertia::Zero();
    for (const auto& seg : joint_segment_map.at(joint_name)) {
      if (seg.find("thrust") == std::string::npos) {
        KDL::Frame f = seg_frames.at(seg);
        inertia = inertia + f * inertia_map.at(seg);
      }
    }
    KDL::Vector c = inertia.getCOG();
    double m = inertia.getMass();

    KDL::Vector p_momentum_jacobian_col = a * (c - r) * m;
    KDL::Vector l_momentum_jacobian_col = (c - cog_all) * p_momentum_jacobian_col + inertia.RefPoint(c).getRotationalInertia() * a;

    cog_jacobian_.col(6 + col_index) = aerial_robot_model::kdlToEigen(p_momentum_jacobian_col / mass_all);
    l_momentum_jacobian_.col(6 + col_index) = aerial_robot_model::kdlToEigen(l_momentum_jacobian_col);
    col_index++;
  }

  // virtual 6dof root
  cog_jacobian_.leftCols(3) = Eigen::MatrixXd::Identity(3, 3);
  cog_jacobian_.middleCols(3, 3) = - aerial_robot_model::skew(aerial_robot_model::kdlToEigen(cog_all));
  cog_jacobian_ = root_rot * cog_jacobian_;

  l_momentum_jacobian_.leftCols(3) = Eigen::MatrixXd::Zero(3, 3);
  l_momentum_jacobian_.middleCols(3, 3) = getInertia<Eigen::Matrix3d>() * root_rot; // aready converted
  l_momentum_jacobian_.rightCols(joint_num) = root_rot * l_momentum_jacobian_.rightCols(joint_num);
}
