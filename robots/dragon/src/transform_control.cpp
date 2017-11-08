#include <dragon/transform_control.h>

using namespace std;

DragonTransformController::DragonTransformController(ros::NodeHandle nh, ros::NodeHandle nh_private, bool callback_flag): TransformController(nh, nh_private, callback_flag)
{
  initParam();

  links_frame_from_cog_.resize(rotor_num_);
  edfs_origin_from_cog_.resize(rotor_num_ * 2); // special for dual edf rotos

  string pub_name;
  nh_private_.param("gimbal_control_topic_name", pub_name, string("gimbals_ctrl"));
  gimbal_control_pub_ = nh_.advertise<sensor_msgs::JointState>(pub_name, 1);

}


/* to retreive the gimbal angles at the hovering state,
   which is necessary to calculate the nominal inertia for LQI control
 */
void DragonTransformController::gimbalProcess(sensor_msgs::JointState& state)
{
  KDL::TreeFkSolverPos_recursive fk_solver(tree_);
  unsigned int nj = tree_.getNrOfJoints();
  KDL::JntArray jointpositions(nj);

  std::map<std::string, uint32_t> gimbal_map;

  unsigned int j = 0;
  for(unsigned int i = 0; i < state.position.size(); i++)
    {
      std::map<std::string, uint32_t>::iterator itr = joint_map_.find(state.name[i]);
      if(itr != joint_map_.end())  jointpositions(joint_map_.find(state.name[i])->second) = state.position[i];

      if(state.name[i].find("gimbal") != string::npos)
        gimbal_map.insert(std::make_pair(state.name[i], i));
    }

  KDL::Frame f;
  fk_solver.JntToCart(jointpositions, f, baselink_);
  KDL::Rotation cog_frame = f.M * cog_desire_orientation_.Inverse();

  /* link based on COG */
  sensor_msgs::JointState gimbal_control_msg;
  gimbal_nominal_angles_.clear();
  for(int i = 0; i < rotor_num_; i++)
    {
      std::stringstream ss;
      ss << i + 1;
      KDL::Frame f;
      fk_solver.JntToCart(jointpositions, f, std::string("link") + ss.str());

      links_frame_from_cog_[i] = cog_frame.Inverse() * f.M;
      double r, p, y;
      links_frame_from_cog_[i].GetRPY(r, p, y);

      state.position[gimbal_map.find(std::string("gimbal") + ss.str() + std::string("_roll"))->second] = -r;
      state.position[gimbal_map.find(std::string("gimbal") + ss.str() + std::string("_pitch"))->second] = -p;
      //f_rotors.push_back(Eigen::Map<const Eigen::Vector3d>((cog_frame.Inverse() * f).p.data));
      //std::cout << "link" << i + 1 << ": \n"<< f_from_cog << std::endl;

      gimbal_control_msg.position.push_back(-r);
      gimbal_control_msg.position.push_back(-p);

      gimbal_nominal_angles_.push_back(-r);
      gimbal_nominal_angles_.push_back(-p);
    }

  if(gimbal_control_) gimbal_control_pub_.publish(gimbal_control_msg);
}

void DragonTransformController::initParam()
{
  nh_private_.param("gimbal_control", gimbal_control_, false);

  nh_private_.param("edf_radius", edf_radius_, 0.035); //70mm EDF
  nh_private_.param("edf_max_tilt", edf_max_tilt_, 0.26); //15 [deg]
}


void DragonTransformController::kinematics(sensor_msgs::JointState state)
{
  /* special process */
  gimbalProcess(state);

  TransformController::kinematics(state);

  /* special process for dual edf gimbal */
  KDL::TreeFkSolverPos_recursive fk_solver(tree_);
  unsigned int nj = tree_.getNrOfJoints();
  KDL::JntArray jointpositions(nj);

  unsigned int j = 0;
  for(unsigned int i = 0; i < state.position.size(); i++)
    {
      std::map<std::string, uint32_t>::iterator itr = joint_map_.find(state.name[i]);
      if(itr != joint_map_.end())  jointpositions(joint_map_.find(state.name[i])->second) = state.position[i];
    }

  KDL::Frame cog_frame;
  tf::transformTFToKDL(cog_, cog_frame);
  std::vector<Eigen::Vector3d> f_edfs;
  for(int i = 0; i < rotor_num_; i++)
    {
      std::stringstream ss;
      ss << i + 1;
      std::string edf = std::string("edf") + ss.str() + std::string("_left");
      KDL::Frame f;
      int status = fk_solver.JntToCart(jointpositions, f, edf);

      f_edfs.push_back(Eigen::Map<const Eigen::Vector3d>((cog_frame.Inverse() * f).p.data));
      //ROS_WARN(" %s status is : %d, [%f, %f, %f]", edf.c_str(), status, f.p.x(), f.p.y(), f.p.z());
      //std::cout << "edf " << i + 1 << "left: \n"<< f_edfs[i] << std::endl;

      edf = std::string("edf") + ss.str() + std::string("_right");
      status = fk_solver.JntToCart(jointpositions, f, edf);
      f_edfs.push_back(Eigen::Map<const Eigen::Vector3d>((cog_frame.Inverse() * f).p.data));
      //std::cout << "edf " << i + 1 << "right: \n"<< f_edfs[i] << std::endl;
      //ROS_WARN(" %s status is : %d, [%f, %f, %f]", edf.c_str(), status, f.p.x(), f.p.y(), f.p.z());
    }

  setEdfsFromCog(f_edfs);
}

bool DragonTransformController::overlapCheck(bool verbose)
{
  std::vector<Eigen::Vector3d> edfs_origin_from_cog(rotor_num_ * 2);
  getEdfsFromCog(edfs_origin_from_cog);

  for(int i = 0; i < rotor_num_ * 2; i++)
    {
      for(int j =  i + 1; j < rotor_num_ * 2; j++)
        {
          Eigen::Vector3d diff = edfs_origin_from_cog[i] - edfs_origin_from_cog[j]; //dual
          double projected_dist = sqrt(diff(0) * diff(0) + diff(1) * diff(1));
          //approximated, the true one should be (edf_radius_ / cos(tilt) + diff(2) * tan(tilt)
          double dist_thre = edf_radius_  + fabs(diff(2)) * tan(edf_max_tilt_) + edf_radius_;

          /* special for dual rotor */
          if(i / 2 == j / 2) continue;

          /* debug */
          //ROS_INFO(" %d and %d, projectd_dist: %f, thre: %f", i + 1, j + 1, projected_dist, dist_thre);

          if(dist_thre > projected_dist)
            {
              if(verbose)ROS_WARN("overlap!: %d and %d, projectd_dist: %f, thre: %f",
                                  i + 1, j + 1, projected_dist, dist_thre);
              return false;
            }
        }
    }

  return true;
}
