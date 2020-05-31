#include <hydrus/hydrus_tilted_robot_model.h>

HydrusTiltedRobotModel::HydrusTiltedRobotModel(bool init_with_rosparam, bool verbose, double fc_t_min_thre, double epsilon):
  HydrusRobotModel(init_with_rosparam, verbose, fc_t_min_thre, 0, epsilon, 4)
{
}


void HydrusTiltedRobotModel::calcStaticThrust()
{
  calcWrenchMatrixOnRoot(); // update Q matrix

  /* calculate the static thrust on CoG frame */
  /* note: can not calculate in root frame, sine the projected f_x, f_y is different in CoG and root */
  Eigen::MatrixXd wrench_mat_on_cog = calcWrenchMatrixOnCoG();

  Eigen::VectorXd static_thrust = aerial_robot_model::pseudoinverse(wrench_mat_on_cog.middleRows(2, 4)) * getGravity().segment(2,4) * getMass();
  setStaticThrust(static_thrust);
}

void HydrusTiltedRobotModel::updateRobotModelImpl(const KDL::JntArray& joint_positions)
{
  aerial_robot_model::RobotModel::updateRobotModelImpl(joint_positions);

  if(getStaticThrust().minCoeff() < 0)
    {
      setCogDesireOrientation(0, 0, 0);
      return; // invalid robot state
    }

  /* special process to find the hovering axis for tilt model */
  Eigen::MatrixXd cog_rot_inv = aerial_robot_model::kdlToEigen(getCogDesireOrientation<KDL::Rotation>().Inverse());
  Eigen::MatrixXd wrench_mat_on_cog = calcWrenchMatrixOnCoG();
  Eigen::VectorXd f = cog_rot_inv * wrench_mat_on_cog.topRows(3) * getStaticThrust();

  double f_norm_roll = atan2(f(1), f(2));
  double f_norm_pitch = atan2(-f(0), sqrt(f(1)*f(1) + f(2)*f(2)));

  /* set the hoverable frame as CoG and reupdate model */
  setCogDesireOrientation(f_norm_roll, f_norm_pitch, 0);
  HydrusRobotModel::updateRobotModelImpl(joint_positions);

  if(getVerbose())
  {
    ROS_INFO_STREAM("f_norm_pitch: " << f_norm_pitch << "; f_norm_roll: " << f_norm_roll);
    ROS_INFO_STREAM("rescaled static thrust: " << getStaticThrust().transpose());
  }
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(HydrusTiltedRobotModel, aerial_robot_model::RobotModel);
