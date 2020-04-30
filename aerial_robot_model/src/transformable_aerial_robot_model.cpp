#include <aerial_robot_model/transformable_aerial_robot_model.h>

namespace aerial_robot_model {

  RobotModel::RobotModel(bool init_with_rosparam, bool verbose, double epsilon):
    verbose_(verbose),
    epsilon_(epsilon),
    baselink_("fc"),
    thrust_link_("thrust"),
    rotor_num_(0),
    joint_num_(0),
    thrust_max_(0),
    thrust_min_(0)
  {
    /* robot model */
    if (!model_.initParam("robot_description"))
      {
        ROS_ERROR("Failed to extract urdf model from rosparam");
        return;
      }
    if (!kdl_parser::treeFromUrdfModel(model_, tree_))
      {
        ROS_ERROR("Failed to extract kdl tree from xml robot description");
        return;
      }
    /* get baselink and thrust_link from robot model */
    auto robot_model_xml = getRobotModelXml("robot_description");
    TiXmlElement* baselink_attr = robot_model_xml.FirstChildElement("robot")->FirstChildElement("baselink");
    if(!baselink_attr)
      ROS_DEBUG("Can not get baselink attribute from urdf model");
    else
      baselink_ = std::string(baselink_attr->Attribute("name"));

    TiXmlElement* thrust_link_attr = robot_model_xml.FirstChildElement("robot")->FirstChildElement("thrust_link");
    if(!thrust_link_attr)
      ROS_DEBUG("Can not get thrust_link attribute from urdf model");
    else
      thrust_link_ = std::string(thrust_link_attr->Attribute("name"));

    if(!model_.getLink(baselink_))
      {
        ROS_ERROR_STREAM("Can not find the link named '" << baselink_ << "' in urdf model");
        return;
      }
    bool found_thrust_link = false;
    std::vector<urdf::LinkSharedPtr> urdf_links;
    model_.getLinks(urdf_links);
    for(const auto& link: urdf_links)
      {
        if(link->name.find(thrust_link_.c_str()) != std::string::npos)
          found_thrust_link = true;
      }
    if(!found_thrust_link)
      {
        ROS_ERROR_STREAM("Can not find the link named '" << baselink_ << "' in urdf model");
        return;
      }

    /* set rotor property */
    TiXmlElement* m_f_rate_attr = robot_model_xml.FirstChildElement("robot")->FirstChildElement("m_f_rate");
    if(!m_f_rate_attr)
      ROS_ERROR("Can not get m_f_rate attribute from urdf model");
    else
      m_f_rate_attr->Attribute("value", &m_f_rate_);

    for(const auto& link: urdf_links)
      {
        if(link->parent_joint)
          {
            if(link->parent_joint->name == "rotor1")
              {
                thrust_max_ = link->parent_joint->limits->upper;
                thrust_min_ = link->parent_joint->limits->lower;
                break;
              }
          }
      }


    if (init_with_rosparam)
      {
        getParamFromRos();
      }

    inertialSetup(tree_.getRootSegment()->second);
    makeJointSegmentMap();
    resolveLinkLength();

    rotors_origin_from_cog_.resize(rotor_num_);
    rotors_normal_from_cog_.resize(rotor_num_);

    for(auto itr : link_joint_names_)
      {
        auto joint_ptr = model_.getJoint(itr);
        link_joint_lower_limits_.push_back(joint_ptr->limits->lower);
        link_joint_upper_limits_.push_back(joint_ptr->limits->upper);
      }


    // jacobian
    full_body_ndof_ = 6 + joint_num_;
    q_mat_.resize(6, rotor_num_);
    v_.resize(rotor_num_);
    u_jacobian_.resize(rotor_num_);
    v_jacobian_.resize(rotor_num_);
    p_jacobian_.resize(rotor_num_);
    cog_jacobian_.resize(3, full_body_ndof_);
    l_momentum_jacobian_.resize(3, full_body_ndof_);
    gravity_.resize(6);
    gravity_ <<  0, 0, 9.80665, 0, 0, 0;
    gravity_3d_.resize(3);
    gravity_3d_ << 0, 0, 9.80665;
    lambda_jacobian_.resize(rotor_num_, full_body_ndof_);
    f_min_ij_.resize(rotor_num_ * (rotor_num_ - 1));
    t_min_ij_.resize(rotor_num_ * (rotor_num_ - 1));
    f_min_jacobian_.resize(rotor_num_ * (rotor_num_ - 1), full_body_ndof_);
    t_min_jacobian_.resize(rotor_num_ * (rotor_num_ - 1), full_body_ndof_);
    thrust_coord_jacobians_.resize(rotor_num_);
    cog_coord_jacobians_.resize(getInertiaMap().size());
    joint_torque_.resize(joint_num_);
    joint_torque_jacobian_.resize(joint_num_, full_body_ndof_);
    static_thrust_.resize(rotor_num_);
    thrust_wrench_units_.resize(rotor_num_);
    thrust_wrench_allocations_.resize(rotor_num_);

    u_triple_product_jacobian_.resize(rotor_num_);
    for (auto& j : u_triple_product_jacobian_) {
      j.resize(rotor_num_);
      for (auto& k : j) {
        k.resize(rotor_num_);
        for (auto& vec : k) {
          vec.resize(full_body_ndof_);
        }
      }
    }

    v_triple_product_jacobian_.resize(rotor_num_);
    for (auto& j : v_triple_product_jacobian_) {
      j.resize(rotor_num_);
      for (auto& k : j) {
        k.resize(rotor_num_);
        for (auto& vec : k) {
          vec.resize(full_body_ndof_);
        }
      }
    }
   }

  bool RobotModel::addExtraModule(std::string module_name, std::string parent_link_name, KDL::Frame transform, KDL::RigidBodyInertia inertia)
  {
    if(extra_module_map_.find(module_name) == extra_module_map_.end())
      {
        if(inertia_map_.find(parent_link_name) == inertia_map_.end())
          {
            ROS_WARN("[extra module]: fail to add new extra module %s, because its parent link (%s) does not exist", module_name.c_str(), parent_link_name.c_str());
            return false;
          }

        if(!aerial_robot_model::isValidRotation(transform.M))
          {
            ROS_WARN("[extra module]: fail to add new extra module %s, because its orientation is invalid", module_name.c_str());
            return false;
          }

        if(inertia.getMass() <= 0)
          {
            ROS_WARN("[extra module]: fail to add new extra module %s, becuase its mass %f is invalid", module_name.c_str(), inertia.getMass());
            return false;
          }

        KDL::Segment extra_module(parent_link_name, KDL::Joint(KDL::Joint::None), transform, inertia);
        extra_module_map_.insert(std::make_pair(module_name, extra_module));
        ROS_INFO("[extra module]: succeed to add new extra module %s", module_name.c_str());
        return true;
      }
    else
      {
        ROS_WARN("[extra module]: fail to add new extra module %s, becuase it already exists", module_name.c_str());
        return false;
      }
  }

  std::map<std::string, KDL::Frame> RobotModel::fullForwardKinematicsImpl(const KDL::JntArray& joint_positions)
  {
    if (joint_positions.rows() != tree_.getNrOfJoints())
      throw std::runtime_error("joint num is invalid");

    std::map<std::string, KDL::Frame> seg_tf_map;
    std::function<void (const KDL::TreeElement&, const KDL::Frame&) > recursiveFullFk = [&recursiveFullFk, &seg_tf_map, &joint_positions](const KDL::TreeElement& tree_element, const KDL::Frame& parrent_f)
      {
        for (const auto& elem: GetTreeElementChildren(tree_element))
          {
            const KDL::TreeElement& curr_element = elem->second;
            KDL::Frame curr_f = parrent_f * GetTreeElementSegment(curr_element).pose(joint_positions(GetTreeElementQNr(curr_element)));
            seg_tf_map.insert(std::make_pair(GetTreeElementSegment(curr_element).getName(), curr_f));
            recursiveFullFk(curr_element, curr_f);
          }
      };

    recursiveFullFk(tree_.getRootSegment()->second, KDL::Frame::Identity());

    return seg_tf_map;
  }

  void RobotModel::getParamFromRos()
  {
    ros::NodeHandle nhp("~");
    nhp.param("kinematic_verbose", verbose_, false);
    nhp.param("epslion", epsilon_, 10.0);
  }

  KDL::RigidBodyInertia RobotModel::inertialSetup(const KDL::TreeElement& tree_element)
  {
    const KDL::Segment current_seg = GetTreeElementSegment(tree_element);

    KDL::RigidBodyInertia current_seg_inertia = current_seg.getInertia();
    if(verbose_) ROS_WARN_STREAM("segment " <<  current_seg.getName() << ", mass is: " << current_seg_inertia.getMass());

    /* check whether this can be a base inertia segment (i.e. link) */
    /* 1. for the "root" parent link (i.e. link1) */
    if(current_seg.getName().find("root") != std::string::npos)
      {
        assert(inertia_map_.size() == 0);
        assert(GetTreeElementChildren(tree_element).size() == 1);

        const KDL::Segment& child_seg = GetTreeElementSegment(GetTreeElementChildren(tree_element).at(0)->second);
        inertia_map_.insert(std::make_pair(child_seg.getName(), child_seg.getInertia()));
        if(verbose_) ROS_WARN("Add root link: %s", child_seg.getName().c_str());

      }
    /* 2. for segment that has joint with parent segment */
    if (current_seg.getJoint().getType() != KDL::Joint::None)
      {
        /* add the new inertia base (child) link if the joint is not a rotor */
        if(current_seg.getJoint().getName().find("rotor") == std::string::npos)
          {
            /* create a new inertia base link */
            inertia_map_.insert(std::make_pair(current_seg.getName(), current_seg_inertia));
            joint_index_map_.insert(std::make_pair(current_seg.getJoint().getName(), tree_element.q_nr));
            joint_names_.push_back(current_seg.getJoint().getName());
            joint_indices_.push_back(tree_element.q_nr);
            joint_parent_link_names_.push_back(GetTreeElementParent(tree_element)->first);

            /* extract link joint */
            if(current_seg.getJoint().getName().find("joint") == 0)
              {
                link_joint_names_.push_back(current_seg.getJoint().getName());
                link_joint_indices_.push_back(tree_element.q_nr);
              }

            if(verbose_) ROS_WARN("Add new inertia base link: %s", current_seg.getName().c_str());
          }
      }
    /* special process for rotor */
    if(current_seg.getJoint().getName().find("rotor") != std::string::npos)
      {
        /* add the rotor direction */
        if(verbose_) ROS_WARN("%s, rototation is %f", current_seg.getJoint().getName().c_str(), current_seg.getJoint().JointAxis().z());
        rotor_direction_.insert(std::make_pair(std::atoi(current_seg.getJoint().getName().substr(5).c_str()), current_seg.getJoint().JointAxis().z()));
      }

    /* recursion process for children segment */
    for (const auto& elem: GetTreeElementChildren(tree_element))
      {
        const KDL::Segment& child_seg = GetTreeElementSegment(elem->second);
        KDL::RigidBodyInertia child_seg_inertia = child_seg.getFrameToTip() *  inertialSetup(elem->second);
        KDL::RigidBodyInertia current_seg_inertia_old = current_seg_inertia;
        current_seg_inertia = current_seg_inertia_old + child_seg_inertia;

        if(verbose_) ROS_WARN("Add new child segment %s to direct segment: %s", child_seg.getName().c_str(), current_seg.getName().c_str());
      }

    /* count the rotor */
    if(current_seg.getName().find(thrust_link_.c_str()) != std::string::npos) rotor_num_++;
    /* update the inertia if the segment is base */
    if (inertia_map_.find(current_seg.getName()) != inertia_map_.end())
      {
        inertia_map_.at(current_seg.getName()) = current_seg_inertia;

        if(verbose_) ROS_WARN("Total mass of base segment %s is %f", current_seg.getName().c_str(),
                              inertia_map_.at(current_seg.getName()).getMass());
        current_seg_inertia = KDL::RigidBodyInertia::Zero();
      }

    return current_seg_inertia;
  }

  void RobotModel::makeJointSegmentMap()
  {
    joint_segment_map_.clear();
    for (const auto joint_index : joint_index_map_) {
      std::vector<std::string> empty_vec;
      joint_segment_map_[joint_index.first] = empty_vec;
    }

    std::vector<std::string> current_joints;
    jointSegmentSetupRecursive(getTree().getRootSegment()->second, current_joints);
  }

  void RobotModel::jointSegmentSetupRecursive(const KDL::TreeElement& tree_element, std::vector<std::string> current_joints)
  {
    const auto inertia_map = getInertiaMap();
    const KDL::Segment current_seg = GetTreeElementSegment(tree_element);
    bool add_joint_flag = false;

    // if this segment has a real joint except rotor
    if (current_seg.getJoint().getType() != KDL::Joint::None && current_seg.getJoint().getName().find("rotor") == std::string::npos) {
      std::string focused_joint = current_seg.getJoint().getName();
      joint_hierachy_.insert(std::make_pair(focused_joint, current_joints.size()));
      current_joints.push_back(focused_joint);
      bool add_joint_flag = true;
      joint_num_++;
    }

    // if this segment is a real segment (= not having fixed joint)
    if (inertia_map.find(current_seg.getName()) != inertia_map.end() || current_seg.getName().find("thrust") != std::string::npos) {
      for (const auto& cj : current_joints) {
        joint_segment_map_.at(cj).push_back(current_seg.getName());
      }
    }

    // recursive process
    for (const auto& elem: GetTreeElementChildren(tree_element)) {
      jointSegmentSetupRecursive(elem->second, current_joints);
    }

    return;
  }

  KDL::JntArray RobotModel::jointMsgToKdl(const sensor_msgs::JointState& state) const
  {
    KDL::JntArray joint_positions(tree_.getNrOfJoints());
    for(unsigned int i = 0; i < state.position.size(); ++i)
      {
        auto itr = joint_index_map_.find(state.name[i]);
        if(itr != joint_index_map_.end()) joint_positions(itr->second) = state.position[i];
      }
    return joint_positions;
  }

  sensor_msgs::JointState RobotModel::kdlJointToMsg(const KDL::JntArray& joint_positions) const
  {
    sensor_msgs::JointState state;
    state.name.reserve(joint_index_map_.size());
    state.position.reserve(joint_index_map_.size());
    for(const auto& actuator : joint_index_map_)
      {
        state.name.push_back(actuator.first);
        state.position.push_back(joint_positions(actuator.second));
      }
    return state;
  }

  TiXmlDocument RobotModel::getRobotModelXml(const std::string& param)
  {
    ros::NodeHandle nh;
    std::string xml_string;
    nh.getParam(param, xml_string);
    TiXmlDocument xml_doc;
    xml_doc.Parse(xml_string.c_str());

    return xml_doc;
  }

  bool RobotModel::removeExtraModule(std::string module_name)
  {
    const auto it = extra_module_map_.find(module_name);
    if(it == extra_module_map_.end())
      {
        ROS_WARN("[extra module]: fail to remove the extra module %s, because it does not exists", module_name.c_str());
        return false;
      }
    else
      {
        extra_module_map_.erase(module_name);
        ROS_INFO("[extra module]: succeed to remove the extra module %s", module_name.c_str());
        return true;
      }
  }

  void RobotModel::resolveLinkLength() //TODO hard coding
  {
    KDL::JntArray joint_positions(tree_.getNrOfJoints());
    //hard coding //TODO
    KDL::Frame f_link2 = forwardKinematics<KDL::Frame>("link2", joint_positions);
    KDL::Frame f_link3 = forwardKinematics<KDL::Frame>("link3", joint_positions);
    link_length_ = (f_link3.p - f_link2.p).Norm();
  }

  void RobotModel::updateRobotModelImpl(const KDL::JntArray& joint_positions)
  {
    joint_positions_ = joint_positions;

    KDL::RigidBodyInertia link_inertia = KDL::RigidBodyInertia::Zero();
    seg_tf_map_ = fullForwardKinematics(joint_positions);

    for(const auto& inertia : inertia_map_)
      {
        KDL::Frame f = seg_tf_map_[inertia.first];
        link_inertia = link_inertia + f * inertia.second;

        /* process for the extra module */
        for(const auto& extra : extra_module_map_)
          {
            if(extra.second.getName() == inertia.first)
              {
                link_inertia = link_inertia + f * (extra.second.getFrameToTip() * extra.second.getInertia());
              }
          }
      }

    /* CoG */
    KDL::Frame f_baselink = seg_tf_map_[baselink_];
    cog_.M = f_baselink.M * cog_desire_orientation_.Inverse();
    cog_.p = link_inertia.getCOG();
    mass_ = link_inertia.getMass();
    cog2baselink_transform_ = cog_.Inverse() * f_baselink;

    /* thrust point based on COG */
    for(int i = 0; i < rotor_num_; ++i)
      {
        std::string rotor = thrust_link_ + std::to_string(i + 1);
        KDL::Frame f = seg_tf_map_[rotor];
        if(verbose_) ROS_WARN(" %s : [%f, %f, %f]", rotor.c_str(), f.p.x(), f.p.y(), f.p.z());
        rotors_origin_from_cog_.at(i) = (cog_.Inverse() * f).p;
        rotors_normal_from_cog_.at(i) = (cog_.Inverse() * f).M * KDL::Vector(0, 0, 1);
      }

    link_inertia_cog_ = (cog_.Inverse() * link_inertia).getRotationalInertia();

    /* statics */
    calcStaticThrust();
  }

} //namespace aerial_robot_model
