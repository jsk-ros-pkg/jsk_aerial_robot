#include <aerial_robot_dynamics/robot_model.h>

using namespace aerial_robot_dynamics;

PinocchioRobotModel::PinocchioRobotModel()
{
  // Initialize the model and data
  model_ = std::make_shared<pinocchio::Model>();

  // Initialize model with URDF file
  std::string robot_model_string = getRobotModelXml("robot_description");
  pinocchio::urdf::buildModelFromXML(robot_model_string, pinocchio::JointModelFreeFlyer(), *model_);
  model_->lowerPositionLimit.segment<3>(0).setConstant(-100); // position
  model_->upperPositionLimit.segment<3>(0).setConstant(100);  // position
  model_->lowerPositionLimit.segment<4>(3).setConstant(-1.0); // quaternion
  model_->upperPositionLimit.segment<4>(3).setConstant(1.0);  // quaternion

  // Initialize the data structure
  data_ = std::make_shared<pinocchio::Data>(*model_);

  std::cout << "model nq: " << model_->nq << std::endl;
  std::cout << "model nv: " << model_->nv << std::endl;
  std::cout << "model njoints: " << model_->njoints << std::endl;
  std::cout << "model nframes: " << model_->nframes << std::endl;

  // Parse the URDF string to xml
  TiXmlDocument robot_model_xml;
  robot_model_xml.Parse(robot_model_string.c_str());

  // get baselink name from urdf
  TiXmlElement* baselink_attr = robot_model_xml.FirstChildElement("robot")->FirstChildElement("baselink");
  std::string baselink;
  if(!baselink_attr)
    ROS_DEBUG("Can not get baselink attribute from urdf model");
  else
    baselink = std::string(baselink_attr->Attribute("name"));
  std::cout << "Baselink name: " << baselink << std::endl;

  // get rotor property
  TiXmlElement* m_f_rate_attr = robot_model_xml.FirstChildElement("robot")->FirstChildElement("m_f_rate");
  if(!m_f_rate_attr)
    ROS_ERROR("Can not get m_f_rate attribute from urdf model");
  else
    m_f_rate_attr->Attribute("value", &m_f_rate_);
  std::cout << "m_f_rate: " << m_f_rate_ << std::endl;

  // get rotor number
  rotor_num_ = 0;
  for(int i = 0; i < model_->nframes; i++)
    {
      std::string frame_name = model_->frames[i].name;
      if(frame_name.find("rotor") != std::string::npos)
        {
          rotor_num_++;
        }
    }
  std::cout << "Rotor number: " << rotor_num_ << std::endl;
  std::cout << std::endl;

  // Print joint information
  std::vector<int> q_dims(model_->njoints);
  int joint_index = 0;
  for(int i = 0; i < model_->njoints; i++)
    {
      std::string joint_type = model_->joints[i].shortname();
      std::cout << model_->names[i] << " " << joint_type <<  " " << model_->joints[model_->getJointId(model_->names[i])].idx_q() << std::endl;
    }
  std::cout << std::endl;

  // Print frame information
  for(int i = 0; i < model_->nframes; i++)
    {
      std::string frame_name = model_->frames[i].name;
      std::cout << frame_name << std::endl;
    }
}

Eigen::VectorXd PinocchioRobotModel::forwardDynamics(const Eigen::VectorXd& q, const Eigen::VectorXd& v, const Eigen::VectorXd& tau, Eigen::VectorXd& thrust)
{
  // make thrust vector as external wrench
  pinocchio::container::aligned_vector<pinocchio::Force> fext(model_->njoints, pinocchio::Force::Zero());
  for(int i = 0; i < rotor_num_; i++)
    {
      std::string rotor_frame_name = "rotor" + std::to_string(i + 1);
      pinocchio::FrameIndex rotor_frame_index = model_->getFrameId(rotor_frame_name);
      pinocchio::JointIndex rotor_parent_joint_index = model_->frames[rotor_frame_index].parent;
      pinocchio::Force rotor_wrench;

      // LOCAL
      rotor_wrench.linear() = Eigen::Vector3d(0, 0, thrust(i));
      rotor_wrench.angular() = Eigen::Vector3d(0, 0, m_f_rate_ * thrust(i));
      fext.at(rotor_parent_joint_index) = rotor_wrench;
    }

  // Compute the forward dynamics with external forces
  Eigen::VectorXd a = pinocchio::aba(*model_, *data_, q, v, tau, fext, pinocchio::Convention::LOCAL);

  return a;
}

std::string PinocchioRobotModel::getRobotModelXml(const std::string& param_name, ros::NodeHandle nh)
{
  // This function should retrieve the robot model XML string from the parameter server
  std::string robot_model_string = "";
  if (!nh.getParam(param_name, robot_model_string))
    ROS_ERROR("Failed to get robot model XML from parameter server");
  return robot_model_string;
}
