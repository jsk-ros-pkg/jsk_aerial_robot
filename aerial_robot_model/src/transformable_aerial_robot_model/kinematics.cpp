#include <aerial_robot_model/transformable_aerial_robot_model.h>

namespace aerial_robot_model {

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

  void RobotModel::calcBasicKinematicsJacobian()
  {
    const std::vector<Eigen::Vector3d> p = getRotorsOriginFromCog<Eigen::Vector3d>();
    const std::vector<Eigen::Vector3d> u = getRotorsNormalFromCog<Eigen::Vector3d>();
    const auto& joint_positions = getJointPositions();
    const auto& sigma = getRotorDirection();
    const int rotor_num = getRotorNum();
    const double m_f_rate = getMFRate();

    //calc jacobian of u(thrust direction, force vector), p(thrust position)
    for (int i = 0; i < rotor_num; ++i) {
      std::string seg_name = std::string("thrust") + std::to_string(i + 1);
      Eigen::MatrixXd thrust_coord_jacobian = RobotModel::getJacobian(joint_positions, seg_name);
      thrust_coord_jacobians_.at(i) = thrust_coord_jacobian;
      u_jacobians_.at(i) = -skew(u.at(i)) * thrust_coord_jacobian.bottomRows(3);
      p_jacobians_.at(i) = thrust_coord_jacobian.topRows(3) - cog_jacobian_;
    }
  }

  void RobotModel::calcCoGMomentumJacobian()
  {
    double mass_all = getMass();
    const auto cog_all = getCog<KDL::Frame>().p;
    const auto& segment_map = getTree().getSegments();
    const auto& seg_frames = getSegmentsTf();
    const auto& inertia_map = getInertiaMap();
    const auto& joint_names = getJointNames();
    const auto& joint_segment_map = getJointSegmentMap();
    const auto& joint_parent_link_names = getJointParentLinkNames();
    const auto joint_num = getJointNum();

    Eigen::MatrixXd root_rot = aerial_robot_model::kdlToEigen(getCogDesireOrientation<KDL::Rotation>() * seg_frames.at(baselink_).M.Inverse());
    /*
      Note: the jacobian about the cog velocity (linear momentum) and angular momentum.

      Please refer to Eq.13 ~ 22 of following paper:
      ============================================================================
      S. Kajita et al., "Resolved momentum control: humanoid motion planning based on the linear and angular momentum,".
      ============================================================================

      1. the cog velocity is w.r.t in the root link frame.
      2. the angular momentum is w.r.t in cog frame
    */


    int col_index = 0;
    /* fix bug: the joint_segment_map is reordered, which is not match the order of  joint_indeices_ or joint_names_ */
    // joint part
    for (const auto& joint_name : joint_names){
      std::string joint_child_segment_name = joint_segment_map.at(joint_name).at(0);
      KDL::Segment joint_child_segment = GetTreeElementSegment(segment_map.at(joint_child_segment_name));
      KDL::Vector a = seg_frames.at(joint_parent_link_names.at(col_index)).M * joint_child_segment.getJoint().JointAxis();

      KDL::Vector r = seg_frames.at(joint_child_segment_name).p;
      KDL::RigidBodyInertia inertia = KDL::RigidBodyInertia::Zero();
      for (const auto& seg : joint_segment_map.at(joint_name)) {
        if (seg.find("thrust") == std::string::npos) {
          KDL::Frame f = seg_frames.at(seg);
          inertia = inertia + f * inertia_map.at(seg);
        }
      }
      KDL::Vector c = inertia.getCOG();
      double m = inertia.getMass();

      KDL::Vector p_momentum_jacobian_col = a * (c - r) * m;
      KDL::Vector l_momentum_jacobian_col = (c - cog_all) * p_momentum_jacobian_col + inertia.RefPoint(c).getRotationalInertia() * a;

      cog_jacobian_.col(6 + col_index) = aerial_robot_model::kdlToEigen(p_momentum_jacobian_col / mass_all);
      l_momentum_jacobian_.col(6 + col_index) = aerial_robot_model::kdlToEigen(l_momentum_jacobian_col);
      col_index++;
    }

    // virtual 6dof root
    cog_jacobian_.leftCols(3) = Eigen::MatrixXd::Identity(3, 3);
    cog_jacobian_.middleCols(3, 3) = - aerial_robot_model::skew(aerial_robot_model::kdlToEigen(cog_all));
    cog_jacobian_ = root_rot * cog_jacobian_;

    l_momentum_jacobian_.leftCols(3) = Eigen::MatrixXd::Zero(3, 3);
    l_momentum_jacobian_.middleCols(3, 3) = getInertia<Eigen::Matrix3d>() * root_rot; // aready converted
    l_momentum_jacobian_.rightCols(joint_num) = root_rot * l_momentum_jacobian_.rightCols(joint_num);
  }

  KDL::Frame RobotModel::forwardKinematicsImpl(std::string link, const KDL::JntArray& joint_positions) const
  {
    if (joint_positions.rows() != tree_.getNrOfJoints())
      throw std::runtime_error("joint num is invalid");

    KDL::TreeFkSolverPos_recursive fk_solver(tree_);
    KDL::Frame f;
    int status = fk_solver.JntToCart(joint_positions, f, link);
    if(status < 0) ROS_ERROR("can not solve FK to link: %s", link.c_str());

    return f;
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


  TiXmlDocument RobotModel::getRobotModelXml(const std::string param, ros::NodeHandle nh)
  {
    std::string xml_string;
    TiXmlDocument xml_doc;

    if (!nh.hasParam(param))
      {
        ROS_ERROR("Could not find parameter %s on parameter server with namespace '%s'", param.c_str(), nh.getNamespace().c_str());
        return xml_doc;
      }

    nh.getParam(param, xml_string);
    xml_doc.Parse(xml_string.c_str());

    return xml_doc;
  }

  void RobotModel::kinematicsInit()
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
    const int full_body_dof = 6 + joint_num_;
    u_jacobians_.resize(rotor_num_);
    p_jacobians_.resize(rotor_num_);
    thrust_coord_jacobians_.resize(rotor_num_);
    cog_coord_jacobians_.resize(getInertiaMap().size());
    cog_jacobian_.resize(3, full_body_dof);
    l_momentum_jacobian_.resize(3, full_body_dof);
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

  void RobotModel::resolveLinkLength()
  {
    KDL::JntArray joint_positions(tree_.getNrOfJoints());
    //hard coding
    KDL::Frame f_link2 = forwardKinematics<KDL::Frame>("link2", joint_positions);
    KDL::Frame f_link3 = forwardKinematics<KDL::Frame>("link3", joint_positions);
    link_length_ = (f_link3.p - f_link2.p).Norm();
  }

} //namespace aerial_robot_model
