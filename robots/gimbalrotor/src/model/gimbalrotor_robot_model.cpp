#include <gimbalrotor/model/gimbalrotor_robot_model.h>

GimbalrotorRobotModel::GimbalrotorRobotModel(bool init_with_rosparam, bool verbose, double fc_t_min_thre, double epsilon) :
  RobotModel(init_with_rosparam, verbose, fc_t_min_thre, epsilon)
{
  const int rotor_num = getRotorNum();
  const int joint_num = getJointNum();

  links_rotation_from_cog_.resize(rotor_num);
  mask_calc_flag_ = false;
}

void GimbalrotorRobotModel::calcThrustMask()
{
  if(mask_calc_flag_) return;
  /* this process is only for untransformable robots */
  const auto& segment_map = getTree().getSegments();
  const auto& joint_names = getJointNames();
  const auto& joint_segment_map = getJointSegmentMap();
  rotor_masks_.resize(RobotModel::getRotorNum());
  for (int i = 0; i < joint_names.size(); i++)
    {
      Eigen::MatrixXd mask(3, 2);
      std::vector<std::string> joint_i_child_segments = joint_segment_map.at(joint_names[i]);
      std::string joint_i_child_segment_name = joint_i_child_segments.at(0);
      const KDL::Segment& joint_i_child_segment = GetTreeElementSegment(segment_map.at(joint_i_child_segment_name));
      Eigen::Vector3d gimbal_i_axis = aerial_robot_model::kdlToEigen(joint_i_child_segment.getJoint().JointAxis()); //supposing that the axis of gimbal is along coordinate axis
      double m_00 = 1.0 - std::abs(gimbal_i_axis[0]);
      double m_10 = 1.0 - std::abs(gimbal_i_axis[1]);
      mask << m_00, 0, m_10, 0, 0, 1;
      rotor_masks_[i] = mask;
    }
  mask_calc_flag_ = true;
}

void GimbalrotorRobotModel::updateRobotModelImpl(const KDL::JntArray& joint_positions)
{
  KDL::TreeFkSolverPos_recursive fk_solver(getTree());
  /* special process */
  KDL::Frame f_baselink;
  fk_solver.JntToCart(joint_positions, f_baselink, getBaselinkName());
  const KDL::Rotation cog_frame = f_baselink.M * getCogDesireOrientation<KDL::Rotation>().Inverse();
  const auto joint_index_map = getJointIndexMap();
  gimbal_processed_joint_ = joint_positions;
  /* link based on COG */
  for(int i = 0; i < getRotorNum(); ++i)
    {
      KDL::Frame f;
      links_rotation_from_cog_[i] = cog_frame.Inverse() * f.M;
    }
  /* normal robot model update */
  RobotModel::updateRobotModelImpl(gimbal_processed_joint_);

  calcThrustMask();
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(GimbalrotorRobotModel, aerial_robot_model::RobotModel);
