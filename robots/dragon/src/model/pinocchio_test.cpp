#include "dragon/model/pinocchio_test.h"

PinocchioRobotModel::PinocchioRobotModel(ros::NodeHandle nh, ros::NodeHandle nhp):
  nh_(nh),
  nhp_(nhp)
{
  modelInit();
  kinematicsInit();
  inertialInit();
  rotorInit();

  joint_state_sub_ = nh_.subscribe("joint_states", 1, &PinocchioRobotModel::jointStateCallback, this);
}


void PinocchioRobotModel::modelInit()
{
  // Load the urdf model from ros parameter server
  std::string robot_model_string = getRobotModelXml("robot_description");
  pinocchio::urdf::buildModelFromXML(robot_model_string, pinocchio::JointModelFreeFlyer(), model_dbl_);
  ROS_WARN_STREAM("[model][pinocchio] model name: " << model_dbl_.name);

  // Create data required by the algorithms
  pinocchio::Data data_dbl_(model_dbl_);

  // declaraion of model and data with casadi
  model_ = model_dbl_.cast<casadi::SX>();
  data_ = pinocchio::DataTpl<casadi::SX>(model_);

  // get baselink name from urdf
  TiXmlDocument robot_model_xml;
  robot_model_xml.Parse(robot_model_string.c_str());
  TiXmlElement* baselink_attr = robot_model_xml.FirstChildElement("robot")->FirstChildElement("baselink");
  if(!baselink_attr)
    ROS_DEBUG("Can not get baselink attribute from urdf model");
  else
    baselink_ = std::string(baselink_attr->Attribute("name"));

  ROS_WARN_STREAM("[model][pinocchio] model_.nq: " << model_.nq);
  ROS_WARN_STREAM("[model][pinocchio] model_.nv: " << model_.nv);
  ROS_WARN_STREAM("[model][pinocchio] model_.njoints: " << model_.njoints);

  // make map for joint position and index
  std::vector<int> q_dims(model_.njoints);
  for(int i = 0; i < model_.njoints; i++)
  {
    std::string joint_type = model_.joints[i].shortname();
    if(joint_type == "JointModelFreeFlyer")  // floating joint is expressed by seven variables in joint position space (position and quaternion)
      q_dims.at(i) = 7;
    else if(joint_type == "JointModelRUBX" || joint_type == "JointModelRUBY" || joint_type == "JointModelRUBZ")  // continuous joint is expressed by two variables in joint position space (cos and sin)
      q_dims.at(i) = 2;
    else //  revolute joint is expressed by one variable in joint position space 
      q_dims.at(i) = 1;
  }

  int joint_index = 0;
  rotor_num_ = 0;
  for(pinocchio::JointIndex joint_id = 0; joint_id < (pinocchio::JointIndex)model_.njoints; ++joint_id)
  {
    if(model_.names[joint_id] != "universe")
    {
      joint_index_map_[model_.names[joint_id]] = joint_index;
      joint_index += q_dims.at(joint_id);

      // special process for rotor
      if(model_.names[joint_id].find("rotor") != std::string::npos)
      {
        rotor_num_++;
      }
    }
  }
}


void PinocchioRobotModel::kinematicsInit()
{
  // init q
  q_cs_ = casadi::SX::sym("q", model_.nq);
  q_dbl_ = casadi::DM(model_.nq, 1);
  q_.resize(model_.nq, 1);
  for(int i = 0; i < model_.nq; i++)
  {
    q_dbl_(i) = 0.0;
    q_(i) = q_cs_(i);
  }

  // solve FK
  pinocchio::forwardKinematics(model_, data_, q_);
  pinocchio::updateFramePlacements(model_, data_);
}


void PinocchioRobotModel::inertialInit()
{
  // setup cog info  
  oMcog_.translation() = pinocchio::centerOfMass(model_, data_, q_, true);
  oMcog_.rotation() = data_.oMf[model_.getFrameId(baselink_)].rotation();

  mass_ = pinocchio::computeTotalMass(model_);
  ROS_WARN_STREAM("[model][pinocchio] robot mass: " << mass_);


  // TODO: inertia https://github.com/stack-of-tasks/pinocchio/issues/980
  // for(pinocchio::JointIndex joint_id = 0; joint_id < (pinocchio::JointIndex)model_.njoints; ++joint_id)
  //   std::cout << std::setw(24) << std::left
  //             << model_.names[joint_id] << ": "
  //             << std::setw(24) << std::left
  //             << model_.joints[joint_id].shortname() << ": "
  //             << std::fixed << std::setprecision(5)
  //             << model_.inertias[joint_id]
  //             << std::endl;

  std::cout << std::endl;
  // pinocchio::computeCentroidalMap(model_, data_, q_); 
  // std::cout << pinocchio::computeCentroidalMap(model_, data_, q_) << std::endl;
  // std::cout << pinocchio::computeCentroidalMap(model_, data_, q_).rows() << " " << pinocchio::computeCentroidalMap(model_, data_, q_).cols() << std::endl;

  pinocchio::crba(model_, data_, q_);
  data_.M.triangularView<Eigen::StrictlyLower>() = data_.M.transpose().triangularView<Eigen::StrictlyLower>();
  data_.Ycrb[0] = data_.liMi[1].act(data_.Ycrb[1]);

  pinocchio::InertiaTpl<casadi::SX> Ig(oMcog_.inverse().act(data_.Ycrb[0]));
  pinocchio::Symmetric3Tpl<casadi::SX> I = Ig.inertia();
  // inertia_ = I.matrix();
  inertia_ = casadi::SX(3, 3);
  for(int i = 0; i < 3; i++)
  {
    for(int j = 0; j < 3; j++)
    {
      inertia_(i, j) = I.matrix()(i, j);
    }
  }

  // std::cout << I.matrix() << std::endl;
  // std::cout << I.matrix().rows() << " " << I.matrix().cols() << std::endl;

  // std::cout << data_.hg << std::endl;
  // std::cout << data_.Ag << std::endl;
  // std::cout << data_.Ig << std::endl;


}

void PinocchioRobotModel::rotorInit()
{
  // get rotor origin and normal from root and cog 
  rotor_origin_root_.resize(rotor_num_);
  rotor_origin_cog_.resize(rotor_num_);
  rotor_normal_root_.resize(rotor_num_);
  rotor_normal_cog_.resize(rotor_num_);

  for(int i = 0; i < rotor_num_; i++)
  {
    std::string rotor_name = "rotor" + std::to_string(i + 1);
    int joint_id =  model_.getJointId(rotor_name);

    // origin
    casadi::SX rotor_origin_root = casadi::SX::zeros(3);
    casadi::SX rotor_origin_cog = casadi::SX::zeros(3);
    for(int j = 0; j < 3; j++)
    {
      rotor_origin_root(j) = data_.oMi[joint_id].translation()(j);
      rotor_origin_cog(j) = (oMcog_.inverse() * data_.oMi[joint_id]).translation()(j);
    }

    rotor_origin_root_.at(i) = rotor_origin_root;
    rotor_origin_cog_.at(i) = rotor_origin_cog;

    // normal
    casadi::SX rotor_normal_root = casadi::SX::zeros(3);
    casadi::SX rotor_normal_cog = casadi::SX::zeros(3);
    int rotor_axis_type;
    if(model_.joints[joint_id].shortname() == "JointModelRX" || model_.joints[joint_id].shortname() == "JointModelRUBX")
      rotor_axis_type = 0;
    else if(model_.joints[joint_id].shortname() == "JointModelRY" || model_.joints[joint_id].shortname() == "JointModelRUBY")
      rotor_axis_type = 1;
    else if(model_.joints[joint_id].shortname() == "JointModelRZ" || model_.joints[joint_id].shortname() == "JointModelRUBZ")
      rotor_axis_type = 2;

    for(int j = 0; j < 3; j++)
    {
      rotor_normal_root(j) = data_.oMi[joint_id].rotation()(j, rotor_axis_type);
      rotor_normal_cog(j) = (oMcog_.inverse() * data_.oMi[joint_id]).rotation()(j, rotor_axis_type);
    }
    rotor_normal_root_.at(i) = rotor_normal_root;
    rotor_normal_cog_.at(i) = rotor_normal_cog;
  }
}

void PinocchioRobotModel::jointStateCallback(const sensor_msgs::JointStateConstPtr msg)
{
  std::vector<std::string> joint_names = msg->name;
  std::vector<double> joint_positions = msg->position;

  for(int i = 0; i < joint_names.size(); i++)
  {
    q_dbl_(joint_index_map_[joint_names.at(i)]) = joint_positions.at(i);
  }

  // std::cout << "real cog: " << computeRealValue(cog_pos_, q_cs_, q_dbl_).transpose() << std::endl;
  // for(int i = 0; i < rotor_num_; i++)
  // {
  //   std::cout << "origin " << i + 1 << ": " << computeRealValue(rotor_origin_cog_.at(i), q_cs_, q_dbl_).transpose() << std::endl;
  //   std::cout << "normal " << i + 1 << ": " << computeRealValue(rotor_normal_cog_.at(i), q_cs_, q_dbl_).transpose() << std::endl;    
  // }
  // std::cout << std::endl;
  // std::cout << computeRealValue(inertia_, q_cs_, q_dbl_) << std::endl;
  // std::cout << std::endl;
}


int main(int argc, char ** argv)
{
  ros::init (argc, argv, "pinocchio_robot_model");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  PinocchioRobotModel* pinocchio_robot_model = new PinocchioRobotModel(nh, nh_private);
  ros::spin();

  delete pinocchio_robot_model;
  return 0;

  // // zero configuration
  // Eigen::VectorXd q = randomConfiguration(model);
  // q = Eigen::VectorXd::Zero(q.rows());
  // std::cout << "q: " << q.transpose() << std::endl;

  // // joint space with casadi
  // auto q_cs = casadi::SX::sym("q", model.nq);
  // Eigen::Matrix<casadi::SX, Eigen::Dynamic, 1> q_casadi;
  // q_casadi.resize(model.nq, 1);
  // for(int i = 0; i < model.nq; i++)
  //   q_casadi(i) = q_cs(i);

  // // Perform the forward kinematics over the kinematic tree
  // pinocchio::forwardKinematics(model, data, q);

  // // Print out the placement of each joint of the kinematic tree
  // for(pinocchio::JointIndex joint_id = 0; joint_id < (pinocchio::JointIndex)model.njoints; ++joint_id)
  //   std::cout << std::setw(24) << std::left
  //             << model.names[joint_id] << ": "
  //             << std::setw(15) << std::left
  //             << model.joints[joint_id].shortname() << ": "
  //             << std::fixed << std::setprecision(5)
  //             << data.oMi[joint_id].translation().transpose()
  //             << std::endl;

  // std::cout << std::endl;
  // std::cout << std::endl;

  // std::cout << "q_casadi: " << q_casadi.transpose() << std::endl;
  // for(pinocchio::JointIndex joint_id = 0; joint_id < (pinocchio::JointIndex)model_casadi.njoints; ++joint_id)
  //   std::cout << std::setw(24) << std::left
  //             << model_casadi.names[joint_id] << ": "
  //             << std::setw(15) << std::left
  //             << model_casadi.joints[joint_id].shortname() << ": "
  //             << std::fixed << std::setprecision(5)
  //             << data_casadi.oMi[joint_id].translation().transpose()
  //             << std::endl;

  // std::cout << std::endl;

  // // check the position of each joint
  // casadi::DM casadi_dbl = casadi::DM({0, 0, 0, 0, 0, 1.57, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0});
  // std::cout << "casadi_dbl: " << casadi_dbl << std::endl;
  // for(int i = 0; i < model.nv; i++)
  // {
  //   for(int j = 0; j < 3; j++)
  //   {
  //     casadi::Function f = casadi::Function("f", {q_cs}, {data_casadi.oMi[i].translation()(j)});
  //     casadi::DM pos_j = f(casadi_dbl);
  //     std::cout << pos_j << " ";
  //   }
  //   std::cout << std::endl;
  // }
  // std::cout << std::endl;

  // // check jacobian
  // pinocchio::computeJointJacobians(model, data, q);
  // std::cout << "data.J: \n" << data.J << std::endl;
  // std::cout << std::endl;

  // pinocchio::computeJointJacobians(model_casadi, data_casadi, q_casadi);
  // std::cout << "data.J: \n" << data_casadi.J << std::endl;
  // std::cout << std::endl;

  // // kinematics derivatives
  // Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
  // Eigen::VectorXd a = Eigen::VectorXd::Zero(model.nv);

  // // Computes the kinematics derivatives for all the joints of the robot
  // pinocchio::computeForwardKinematicsDerivatives(model, data, q, v, a);

  // // Retrieve the kinematics derivatives of a specific joint, expressed in the LOCAL frame of the joints.
  // pinocchio::JointIndex joint_id = (pinocchio::JointIndex)(model.njoints-1);
  // pinocchio::Data::Matrix6x v_partial_dq(6, model.nv), a_partial_dq(6, model.nv), a_partial_dv(6, model.nv), a_partial_da(6, model.nv);
  // v_partial_dq.setZero();
  // a_partial_dq.setZero(); a_partial_dv.setZero(); a_partial_da.setZero();
  // pinocchio::getJointAccelerationDerivatives(model, data, joint_id, pinocchio::LOCAL, v_partial_dq,
  //                                            a_partial_dq, a_partial_dv, a_partial_da);

  // std::cout << "v partial dq: \n" << v_partial_dq << std::endl;
  // std::cout << std::endl;
  // // Remark: we are not directly computing the quantity v_partial_dv as it is also equal to a_partial_da.

  // // But we can also expressed the same quantities in the frame centered on the end-effector joint, but expressed in the axis aligned with the world frame.
  // pinocchio::getJointAccelerationDerivatives(model,data, joint_id, pinocchio::WORLD, v_partial_dq,
  //                                            a_partial_dq, a_partial_dv, a_partial_da);
  // std::cout << "v partial dq: \n" << v_partial_dq << std::endl;

}
