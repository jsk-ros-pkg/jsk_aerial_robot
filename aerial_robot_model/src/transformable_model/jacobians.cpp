#include <aerial_robot_model/transformable_aerial_robot_model.h>

using namespace aerial_robot_model::transformable;

Eigen::MatrixXd RobotModel::getJacobian(const KDL::JntArray& joint_positions, std::string segment_name, KDL::Vector offset)
{
  const auto& tree = getTree();
  const auto seg_frames = getSegmentsTf();
  
  Eigen::MatrixXd root_rot = aerial_robot_model::kdlToEigen(getCogDesireOrientation<KDL::Rotation>() * seg_frames.at(getBaselinkName()).M.Inverse());

  KDL::TreeJntToJacSolver solver(tree);
  KDL::Jacobian jac(tree.getNrOfJoints());
  int status = solver.JntToJac(joint_positions, jac, segment_name);
  jac.changeRefPoint(seg_frames.at(segment_name).M * offset);

  // joint part
  Eigen::MatrixXd jac_joint = convertJacobian(jac.data);
  // return jac_joint; // only joint jacobian

  // add virtual 6dof root
  Eigen::MatrixXd jac_all = Eigen::MatrixXd::Identity(6, 6 + getJointNum());
  jac_all.rightCols(getJointNum()) = jac_joint;
  Eigen::Vector3d p = aerial_robot_model::kdlToEigen(seg_frames.at(segment_name).p + seg_frames.at(segment_name).M * offset);
  jac_all.block(0,3,3,3) = -aerial_robot_model::skew(p);

  jac_all.topRows(3) = root_rot * jac_all.topRows(3);
  jac_all.bottomRows(3) = root_rot * jac_all.bottomRows(3);
  return jac_all;

}

Eigen::MatrixXd RobotModel::convertJacobian(const Eigen::MatrixXd& in)
{
  const auto& joint_indices = getJointIndices();
  Eigen::MatrixXd out(6, getJointNum());

  int col_index = 0;
  for (const auto& joint_index : joint_indices) { // fix bug of order
    out.col(col_index) = in.col(joint_index);
    col_index++;
  }
  return out;
}


// @deprecated
Eigen::VectorXd RobotModel::getHessian(std::string ref_frame, int joint_i, int joint_j, KDL::Vector offset)
{
  const auto& segment_map = getTree().getSegments();
  const auto seg_frames = getSegmentsTf();
  const auto& joint_hierachy = getJointHierachy();
  const auto& joint_segment_map = getJointSegmentMap();
  const auto& joint_names = getJointNames();
  const auto& joint_parent_link_names = getJointParentLinkNames();

  Eigen::MatrixXd root_rot = aerial_robot_model::kdlToEigen(getCogDesireOrientation<KDL::Rotation>() * seg_frames.at(getBaselinkName()).M.Inverse());

  Eigen::Vector3d p_e = aerial_robot_model::kdlToEigen(seg_frames.at(ref_frame).p + seg_frames.at(ref_frame).M * offset);

  std::string joint_i_name = joint_names.at(joint_i);
  std::string joint_j_name = joint_names.at(joint_j);

  if (joint_hierachy.at(joint_i_name) > joint_hierachy.at(joint_j_name)) {
    std::swap(joint_i_name, joint_j_name);
  }

  std::vector<std::string> joint_i_child_segments = joint_segment_map.at(joint_i_name);
  std::vector<std::string> joint_j_child_segments = joint_segment_map.at(joint_j_name);

  if (std::find(joint_i_child_segments.begin(), joint_i_child_segments.end(), ref_frame) == joint_i_child_segments.end() ||  std::find(joint_j_child_segments.begin(), joint_j_child_segments.end(), ref_frame) == joint_j_child_segments.end()) {
    return Eigen::VectorXd::Zero(6);
  }

  std::string joint_i_child_segment_name = joint_i_child_segments.at(0);
  std::string joint_j_child_segment_name = joint_j_child_segments.at(0);
  const KDL::Segment& joint_i_child_segment = GetTreeElementSegment(segment_map.at(joint_i_child_segment_name));
  const KDL::Segment& joint_j_child_segment = GetTreeElementSegment(segment_map.at(joint_j_child_segment_name));
  Eigen::Vector3d a_i = aerial_robot_model::kdlToEigen(seg_frames.at(joint_parent_link_names.at(joint_i)).M * joint_i_child_segment.getJoint().JointAxis());
  Eigen::Vector3d a_j = aerial_robot_model::kdlToEigen(seg_frames.at(joint_parent_link_names.at(joint_j)).M * joint_j_child_segment.getJoint().JointAxis());
  Eigen::Vector3d p_j = aerial_robot_model::kdlToEigen(seg_frames.at(joint_j_child_segment_name).p); //joint pos

  Eigen::VectorXd derivative(6);
  derivative.topRows(3) = root_rot * a_i.cross(a_j.cross(p_e - p_j));
  derivative.bottomRows(3) = root_rot * a_i.cross(a_j);

  return derivative;
}

Eigen::MatrixXd RobotModel::getSecondDerivative(std::string ref_frame, int joint_i, KDL::Vector offset)
{
  const auto& segment_map = getTree().getSegments();
  const auto seg_frames = getSegmentsTf();
  const auto& joint_hierachy = getJointHierachy();
  const auto& joint_segment_map = getJointSegmentMap();
  const auto& joint_names = getJointNames();
  const auto& joint_parent_link_names = getJointParentLinkNames();
  const auto& joint_num = getJointNum();
  const int full_body_ndof = 6 + joint_num;
  Eigen::MatrixXd root_rot = aerial_robot_model::kdlToEigen(getCogDesireOrientation<KDL::Rotation>() * seg_frames.at(getBaselinkName()).M.Inverse());

  Eigen::Vector3d p_e = aerial_robot_model::kdlToEigen(seg_frames.at(ref_frame).p + seg_frames.at(ref_frame).M * offset);

  std::string joint_i_name = joint_names.at(joint_i);
  std::vector<std::string> joint_i_child_segments = joint_segment_map.at(joint_i_name);
  std::string joint_i_child_segment_name = joint_i_child_segments.at(0);
  const KDL::Segment& joint_i_child_segment = GetTreeElementSegment(segment_map.at(joint_i_child_segment_name));
  Eigen::Vector3d a_i = aerial_robot_model::kdlToEigen(seg_frames.at(joint_parent_link_names.at(joint_i)).M * joint_i_child_segment.getJoint().JointAxis());
  Eigen::Vector3d p_i = aerial_robot_model::kdlToEigen(seg_frames.at(joint_i_child_segment_name).p);

  Eigen::MatrixXd jacobian_i = Eigen::MatrixXd::Zero(6, full_body_ndof);

  if (std::find(joint_i_child_segments.begin(), joint_i_child_segments.end(), ref_frame) == joint_i_child_segments.end()) {
    return jacobian_i;
  }

  // joint part
  for(int j = 0; j < joint_num; j++)
    {
      std::string joint_j_name = joint_names.at(j);
      std::vector<std::string> joint_j_child_segments = joint_segment_map.at(joint_j_name);
      if (std::find(joint_j_child_segments.begin(), joint_j_child_segments.end(), ref_frame) == joint_j_child_segments.end()) {
        continue;
      }

      std::string joint_j_child_segment_name = joint_j_child_segments.at(0);
      const KDL::Segment& joint_j_child_segment = GetTreeElementSegment(segment_map.at(joint_j_child_segment_name));
      Eigen::Vector3d a_j = aerial_robot_model::kdlToEigen(seg_frames.at(joint_parent_link_names.at(j)).M * joint_j_child_segment.getJoint().JointAxis());
      Eigen::Vector3d p_j = aerial_robot_model::kdlToEigen(seg_frames.at(joint_j_child_segment_name).p);

      Eigen::Vector3d a_i_j(0,0,0);
      Eigen::Vector3d p_i_j(0,0,0);
      Eigen::Vector3d p_e_j = a_j.cross(p_e - p_j);
      if ( joint_hierachy.at(joint_j_name) <= joint_hierachy.at(joint_i_name)){
        // joint_j is close to root, or i = j

        // both i and j is revolute jont
        a_i_j = a_j.cross(a_i);
        p_i_j = a_j.cross(p_i - p_j);
      }

      jacobian_i.block(0, 6 + j, 3, 1) = root_rot * a_i_j.cross(p_e - p_i) + root_rot * a_i.cross(p_e_j - p_i_j); // force
      jacobian_i.block(3, 6 + j, 3, 1) = root_rot * a_i_j; // torque
    }

  // virtual 6dof root
  jacobian_i.block(0, 3, 3, 3) = - root_rot * aerial_robot_model::skew(a_i.cross(p_e - p_i));
  jacobian_i.block(3, 3, 3, 3) = - root_rot * aerial_robot_model::skew(a_i);
  return jacobian_i;
}

Eigen::MatrixXd RobotModel::getSecondDerivativeRoot(std::string ref_frame, KDL::Vector offset)
{
  const int joint_num = getJointNum();
  const int full_body_ndof = 6 + joint_num;
  const auto& joint_parent_link_names = getJointParentLinkNames();
  const auto& segment_map = getTree().getSegments();
  const auto seg_frames = getSegmentsTf();
  Eigen::MatrixXd root_rot = aerial_robot_model::kdlToEigen(getCogDesireOrientation<KDL::Rotation>() * seg_frames.at(getBaselinkName()).M.Inverse());

  Eigen::Vector3d p_e = aerial_robot_model::kdlToEigen(seg_frames.at(ref_frame).p + seg_frames.at(ref_frame).M * offset);
  Eigen::MatrixXd out = Eigen::MatrixXd::Zero(3, full_body_ndof);

  // joint part
  for (int i = 0; i < joint_num; ++i) {

    std::string joint_name = getJointNames().at(i);
    std::vector<std::string> joint_child_segments = getJointSegmentMap().at(joint_name);

    if (std::find(joint_child_segments.begin(), joint_child_segments.end(), ref_frame) == joint_child_segments.end())
      {
        continue;
      }

    std::string joint_child_segment_name = joint_child_segments.at(0);
    const KDL::Segment& joint_child_segment = GetTreeElementSegment(segment_map.at(joint_child_segment_name));
    Eigen::Vector3d a = aerial_robot_model::kdlToEigen(seg_frames.at(joint_parent_link_names.at(i)).M * joint_child_segment.getJoint().JointAxis()); // fix bug
    Eigen::Vector3d p = aerial_robot_model::kdlToEigen(seg_frames.at(joint_child_segment_name).p);

    out.col(6 + i) = root_rot * a.cross(p_e - p);
  }

  // virtual root 6dof
  out.leftCols(3) = root_rot;
  out.middleCols(3,3) = - root_rot * aerial_robot_model::skew(p_e);

  return out;
}

