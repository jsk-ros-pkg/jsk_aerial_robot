#include <dragon/transform_control.h>

using namespace std;

DragonTransformController::DragonTransformController(ros::NodeHandle nh, ros::NodeHandle nh_private, bool callback_flag): TransformController(nh, nh_private, callback_flag)
{
  initParam();

  links_frame_from_cog_.resize(rotor_num_);

  string pub_name;
  nh_private_.param("gimbal_control_topic_name", pub_name, string("gimbals_ctrl"));
  gimbal_control_pub_ = nh_.advertise<sensor_msgs::JointState>(pub_name, 1);
}


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
    }

  if(gimbal_control_) gimbal_control_pub_.publish(gimbal_control_msg);
}

void DragonTransformController::jointStateCallback(const sensor_msgs::JointStateConstPtr& state)
{
  sensor_msgs::JointState js = *state;
  gimbalProcess(js);
  TransformController::jointStateCallback(sensor_msgs::JointStateConstPtr(new sensor_msgs::JointState(js)));
}


void DragonTransformController::initParam()
{
  nh_private_.param("gimbal_control", gimbal_control_, false);
}
