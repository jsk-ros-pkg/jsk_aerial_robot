#include <aerial_robot_model/model/transformable_aerial_robot_model.h>

using namespace aerial_robot_model::transformable;

RobotModel::RobotModel(bool init_with_rosparam, bool verbose, double fc_f_min_thre, double fc_t_min_thre, double epsilon):
  aerial_robot_model::RobotModel(init_with_rosparam, verbose, fc_f_min_thre, fc_t_min_thre, epsilon)
{
  std::function<void (const KDL::TreeElement&) > recursiveSegmentCheck = [&recursiveSegmentCheck, this](const KDL::TreeElement& tree_element)
    {
      const KDL::Segment current_seg = GetTreeElementSegment(tree_element);

      if (current_seg.getJoint().getType() != KDL::Joint::None &&
          current_seg.getJoint().getName().find("joint") == 0) {
        link_joint_names_.push_back(current_seg.getJoint().getName());
        link_joint_indices_.push_back(tree_element.q_nr);
      }

      // recursive process
      for (const auto& elem: GetTreeElementChildren(tree_element)) {
        recursiveSegmentCheck(elem->second);
      }

    };
  recursiveSegmentCheck(getTree().getRootSegment()->second);


  for(auto itr : link_joint_names_) {
    auto joint_ptr = getUrdfModel().getJoint(itr);
    link_joint_lower_limits_.push_back(joint_ptr->limits->lower);
    link_joint_upper_limits_.push_back(joint_ptr->limits->upper);
  }

  resolveLinkLength();

  // jacobian
  const int rotor_num = getRotorNum();
  const int joint_num = getJointNum();
  const int full_body_dof = 6 + joint_num;
  u_jacobians_.resize(rotor_num);
  p_jacobians_.resize(rotor_num);
  thrust_coord_jacobians_.resize(rotor_num);
  cog_coord_jacobians_.resize(getInertiaMap().size());
  cog_jacobian_.resize(3, full_body_dof);
  l_momentum_jacobian_.resize(3, full_body_dof);
  approx_fc_f_dists_.resize(rotor_num * (rotor_num - 1));
  approx_fc_t_dists_.resize(rotor_num * (rotor_num - 1));
  fc_f_dists_jacobian_.resize(rotor_num * (rotor_num - 1), full_body_dof);
  fc_t_dists_jacobian_.resize(rotor_num * (rotor_num - 1), full_body_dof);
  lambda_jacobian_.resize(rotor_num, full_body_dof);
  joint_torque_.resize(joint_num);
  joint_torque_jacobian_.resize(joint_num, full_body_dof);
}

void RobotModel::updateJacobians()
{
  updateJacobians(getJointPositions(), false);
}

void RobotModel::updateJacobians(const KDL::JntArray& joint_positions, bool update_model)
{
  if(update_model) updateRobotModel(joint_positions);

  calcCoGMomentumJacobian(); // should be processed first

  calcBasicKinematicsJacobian(); // need cog_jacobian_

  calcLambdaJacobian();

  calcJointTorque(false);

  calcJointTorqueJacobian();

  calcFeasibleControlJacobian();
}
