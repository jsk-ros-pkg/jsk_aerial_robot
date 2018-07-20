#include <dragon/dragon_robot_model.h>

DragonRobotModel::DragonRobotModel(bool init_with_rosparam, bool verbose, std::string baselink, std::string thrust_link, double stability_margin_thre, double p_det_thre, double f_max, double f_min, double m_f_rate, bool only_three_axis_mode, double edf_radius, double edf_max_tilt) :
  HydrusRobotModel(init_with_rosparam, verbose, baselink, thrust_link, stability_margin_thre, p_det_thre, f_max, f_min, m_f_rate, only_three_axis_mode),
  edf_radius_(edf_radius),
  edf_max_tilt_(edf_max_tilt)
{
  if (init_with_rosparam)
    {
      getParamFromRos();
    }

  links_rotation_from_cog_.resize(getRotorNum());
  edfs_origin_from_cog_.resize(getRotorNum() * 2);
  gimbal_nominal_angles_.resize(getRotorNum() * 2);
}

void DragonRobotModel::getParamFromRos()
{
  ros::NodeHandle nhp("~");
  nhp.param("edf_radius", edf_radius_, 0.035); //70mm EDF
  nhp.param("edf_max_tilt", edf_max_tilt_, 0.26); //15 [deg]
}

KDL::JntArray DragonRobotModel::gimbalProcess(const KDL::JntArray& joint_positions)
{
  KDL::JntArray modified_joint_positions = joint_positions;
  const KDL::Frame f = forwardKinematics<KDL::Frame>(getBaselinkName(), joint_positions);
  const KDL::Rotation cog_frame = f.M * getCogDesireOrientation<KDL::Rotation>().Inverse();
  const auto actuator_map = getActuatorMap();

  /* link based on COG */
  for(int i = 0; i < getRotorNum(); ++i)
    {
      std::string s = std::to_string(i + 1);
      KDL::Frame f = forwardKinematics<KDL::Frame>(std::string("link") + s, joint_positions);

      links_rotation_from_cog_[i] = cog_frame.Inverse() * f.M;
      double r, p, y;
      links_rotation_from_cog_[i].GetRPY(r, p, y);

      modified_joint_positions(actuator_map.find(std::string("gimbal") + s + std::string("_roll"))->second) = -r;
      modified_joint_positions(actuator_map.find(std::string("gimbal") + s + std::string("_pitch"))->second) = -p;

      gimbal_nominal_angles_[i * 2] = -r;
      gimbal_nominal_angles_[i * 2 + 1] = -p;
    }
  return modified_joint_positions;
}

bool DragonRobotModel::overlapCheck(bool verbose) const
{
  const std::vector<Eigen::Vector3d> edfs_origin_from_cog = getEdfsOriginFromCog<Eigen::Vector3d>();
  const int rotor_num = getRotorNum();

  for(int i = 0; i < rotor_num * 2; ++i)
    {
      for(int j = i + 1; j < rotor_num * 2; ++j)
        {
          Eigen::Vector3d diff = edfs_origin_from_cog[i] - edfs_origin_from_cog[j]; //dual
          double projected_dist = sqrt(diff(0) * diff(0) + diff(1) * diff(1));
          //approximated, the true one should be (edf_radius_ / cos(tilt) + diff(2) * tan(tilt)
          double dist_thre = edf_radius_ + fabs(diff(2)) * tan(edf_max_tilt_) + edf_radius_;
          /* special for dual rotor */
          if(i / 2 == j / 2) continue;

          /* debug */
          if(dist_thre > projected_dist)
            {
              if(verbose) ROS_WARN("overlap!: %d and %d, projectd_dist: %f, thre: %f",
                                  i + 1, j + 1, projected_dist, dist_thre);
              return false;
            }
        }
    }
  return true;
}

void DragonRobotModel::updateRobotModelImpl(const KDL::JntArray& joint_positions)
{
  /* special process */
  gimbal_processed_joint_ = gimbalProcess(joint_positions);
  HydrusRobotModel::updateRobotModelImpl(joint_positions);

  /* special process for dual edf gimbal */
  /* set the edf position w.r.t CoG frame */
  std::vector<KDL::Vector> f_edfs;
  for(int i = 0; i < getRotorNum(); ++i)
    {
      std::string s = std::to_string(i + 1);
      std::string edf = std::string("edf") + s + std::string("_left");
      KDL::Frame f = forwardKinematics<KDL::Frame>(edf, gimbal_processed_joint_);

      f_edfs.push_back((getCog<KDL::Frame>().Inverse() * f).p);

      edf = std::string("edf") + s + std::string("_right");
      f = forwardKinematics<KDL::Frame>(edf, gimbal_processed_joint_);
      f_edfs.push_back((getCog<KDL::Frame>().Inverse() * f).p);
    }

  edfs_origin_from_cog_ = f_edfs;
}
