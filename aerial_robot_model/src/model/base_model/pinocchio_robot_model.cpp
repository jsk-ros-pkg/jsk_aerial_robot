#include <aerial_robot_model/model/pinocchio_robot_model.h>

namespace aerial_robot_model {
  PinocchioRobotModel::PinocchioRobotModel()
  {
    modelInit();
    kinematicsInit();
    inertialInit();
    rotorInit();

    updateRobotModel();
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
        else if(joint_type == "JointModelRUBX" || joint_type == "JointModelRUBY" || joint_type == "JointModelRUBZ" || joint_type == "JointModelRevoluteUnboundedUnaligned")  // continuous joint is expressed by two variables in joint position space (cos and sin)
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

    for(pinocchio::JointIndex joint_id = 0; joint_id < (pinocchio::JointIndex)model_.njoints; ++joint_id)
      ROS_WARN_STREAM("[model][pinocchio] joint " << std::setw(4) << joint_id << ": " << std::setw(24) << std::left << model_.names[joint_id] << ": " << std::setw(24) << std::left << model_.joints[joint_id].shortname());
    for(int i = 0; i < model_.frames.size(); i++)
      ROS_WARN_STREAM("[model][pinocchio] frame " << std::setw(4) << i << ": " << std::setw(24) << std::left << model_.frames[i].name);
  }


  void PinocchioRobotModel::kinematicsInit()
  {
    // init q
    q_cs_ = casadi::SX::sym("q", model_.nq);
    for(int i = 0; i < 7; i++)  // set 0 to free flyer joint
      q_cs_(i) = 0;

    q_dbl_ = casadi::DM(model_.nq, 1);
    q_.resize(model_.nq, 1);
    for(int i = 0; i < model_.nq; i++)
      {
        q_dbl_(i) = 0.0;
        q_(i) = q_cs_(i);
      }
  }

  void PinocchioRobotModel::inertialInit()
  {
    // get mass (caution: type of this variable is casadi::SX)
    mass_ = pinocchio::computeTotalMass(model_);
    ROS_WARN_STREAM("[model][pinocchio] robot mass: " << mass_);
  }

  void PinocchioRobotModel::rotorInit()
  {
    rotors_origin_root_.resize(rotor_num_);
    rotors_origin_cog_.resize(rotor_num_);
    rotors_normal_root_.resize(rotor_num_);
    rotors_normal_cog_.resize(rotor_num_);
  }

  void PinocchioRobotModel::inertialUpdate()
  {
    // set cog frame
    oMcog_.translation() = pinocchio::centerOfMass(model_, data_, q_, true);
    oMcog_.rotation() = data_.oMf[model_.getFrameId(baselink_)].rotation();
    pinocchio::casadi::copy(oMcog_.translation(), opcog_);

    // get inertia matrix expressed in cog frame. (Hint: pinocchio/unittest/centroidal.cpp)
    pinocchio::crba(model_, data_, q_);    // Composite Rigid Body Algorithm
    data_.M.triangularView<Eigen::StrictlyLower>() = data_.M.transpose().triangularView<Eigen::StrictlyLower>();
    data_.Ycrb[0] = data_.liMi[1].act(data_.Ycrb[1]);

    pinocchio::InertiaTpl<casadi::SX> Ig(oMcog_.inverse().act(data_.Ycrb[0]));
    pinocchio::Symmetric3Tpl<casadi::SX> I = Ig.inertia();
    pinocchio::casadi::copy(I.matrix(), inertia_);
  }

  void PinocchioRobotModel::rotorUpdate()
  {
    // get rotor origin and normal from root and cog
    for(int i = 0; i < rotor_num_; i++)
      {
        std::string rotor_name = "rotor" + std::to_string(i + 1);
        int joint_id =  model_.getJointId(rotor_name);

        // origin
        pinocchio::casadi::copy(data_.oMi[joint_id].translation(), rotors_origin_root_.at(i));
        pinocchio::casadi::copy((oMcog_.inverse() * data_.oMi[joint_id]).translation(), rotors_origin_cog_.at(i));

        // normal (assume rotational axis is corresponding to z axis)
        pinocchio::casadi::copy(data_.oMi[joint_id].rotation().middleCols(2, 1), rotors_normal_root_.at(i));
        pinocchio::casadi::copy((oMcog_.inverse() * data_.oMi[joint_id]).rotation().middleCols(2, 1), rotors_normal_cog_.at(i));
      }
  }

  void PinocchioRobotModel::updateRobotModel()
  {
    Eigen::Matrix<casadi::SX, Eigen::Dynamic, 1> q;
    q.resize(model_.nq, 1);
    pinocchio::casadi::copy(q_cs_, q);

    updateRobotModelImpl(q);
  }

  void PinocchioRobotModel::updateRobotModelImpl(Eigen::Matrix<casadi::SX, Eigen::Dynamic, 1> q)
  {
    q_ = q;

    // solve FK
    pinocchio::forwardKinematics(model_, data_, q_);
    pinocchio::updateFramePlacements(model_, data_);

    inertialUpdate();
    rotorUpdate();
  }

  std::string PinocchioRobotModel::getRobotModelXml(const std::string param, ros::NodeHandle nh)
  {
    std::string xml_string;

  if(!nh.hasParam(param))
    {
      ROS_ERROR("Could not find parameter %s on parameter server with namespace '%s'", param.c_str(), nh.getNamespace().c_str());
      return xml_string;
    }
  nh.getParam(param, xml_string);
  return xml_string;
  }
}

Eigen::MatrixXd computeRealValue(casadi::SX y, casadi::SX x, Eigen::VectorXd x_dbl)
{
  casadi::DM ret = casadi::DM(y.size1(), y.size2());
  for(int i =  0; i < y.size1(); i++)
    {
      for(int j = 0; j < y.size2(); j++)
        {
          casadi::Function f = casadi::Function("f", {x}, {y(i, j)});
          casadi::DM y_dbl = f(eigenVectorToCasadiDm(x_dbl));
          ret(i, j) = y_dbl;
        }
    }
  return casadiDmToEigenMatrix(ret);
}

Eigen::MatrixXd computeRealValue(casadi::SX y, casadi::SX x, casadi::DM x_dbl)
{
  casadi::DM ret = casadi::DM(y.size1(), y.size2());
  for(int i =  0; i < y.size1(); i++)
    {
      for(int j = 0; j < y.size2(); j++)
        {
          casadi::Function f = casadi::Function("f", {x}, {y(i, j)});
          casadi::DM y_dbl = f(x_dbl);
          ret(i, j) = y_dbl;
        }
    }
  return casadiDmToEigenMatrix(ret);
}
