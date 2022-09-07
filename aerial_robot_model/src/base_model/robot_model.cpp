#include <aerial_robot_model/aerial_robot_model.h>

namespace aerial_robot_model {

  RobotModel::RobotModel(bool init_with_rosparam, bool verbose, double fc_f_min_thre, double fc_t_min_thre, double epsilon):
    verbose_(verbose),
    fc_f_min_thre_(fc_f_min_thre),
    fc_t_min_thre_(fc_t_min_thre),
    epsilon_(epsilon),
    baselink_("fc"),
    thrust_link_("thrust"),
    rotor_num_(0),
    joint_num_(0),
    thrust_max_(0),
    thrust_min_(0),
    mass_(0),
    initialized_(false)
  {
    if (init_with_rosparam)
      getParamFromRos();

    gravity_.resize(6);
    gravity_ <<  0, 0, 9.80665, 0, 0, 0;
    gravity_3d_.resize(3);
    gravity_3d_ << 0, 0, 9.80665;

    kinematicsInit();
    stabilityInit();
    staticsInit();

    // update robot model instantly for fixed model
    if (joint_num_ == 0) {
      updateRobotModel();
    }
  }

  void RobotModel::getParamFromRos()
  {
    ros::NodeHandle nh;
    nh.param("kinematic_verbose", verbose_, false);
    nh.param("fc_f_min_thre", fc_f_min_thre_, 0.0);
    nh.param("fc_t_min_thre", fc_t_min_thre_, 0.0);
    nh.param("epsilon", epsilon_, 10.0);
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

    rotors_origin_from_cog_.resize(rotor_num_);
    rotors_normal_from_cog_.resize(rotor_num_);
  }

  void RobotModel::stabilityInit()
  {
    fc_f_dists_.resize(rotor_num_ * (rotor_num_ - 1));
    fc_t_dists_.resize(rotor_num_ * (rotor_num_ - 1));
  }

  void RobotModel::staticsInit()
  {
    /* get baselink and thrust_link from robot model */
    auto robot_model_xml = getRobotModelXml("robot_description");

    /* set rotor property */
    TiXmlElement* m_f_rate_attr = robot_model_xml.FirstChildElement("robot")->FirstChildElement("m_f_rate");
    if(!m_f_rate_attr)
      ROS_ERROR("Can not get m_f_rate attribute from urdf model");
    else
      m_f_rate_attr->Attribute("value", &m_f_rate_);

    std::vector<urdf::LinkSharedPtr> urdf_links;
    model_.getLinks(urdf_links);
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

    q_mat_.resize(6, rotor_num_);
    static_thrust_.resize(rotor_num_);
    thrust_wrench_units_.resize(rotor_num_);
    thrust_wrench_allocations_.resize(rotor_num_);
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

            if(verbose_) ROS_WARN("Add new inertia base link: %s", current_seg.getName().c_str());
          }
      }
    /* special process for rotor */
    if(current_seg.getJoint().getName().find("rotor") != std::string::npos)
      {
        /* add the rotor direction */
        auto urdf_joint =  model_.getJoint(current_seg.getJoint().getName());
        if(urdf_joint->type == urdf::Joint::CONTINUOUS)
          {
            if(verbose_) ROS_WARN("joint name: %s, z axis: %f", current_seg.getJoint().getName().c_str(), urdf_joint->axis.z);
            rotor_direction_.insert(std::make_pair(std::atoi(current_seg.getJoint().getName().substr(5).c_str()), urdf_joint->axis.z));
          }
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

        if (joint_num_ == 0) {
          // update robot model instantly
          updateRobotModel();
        }

        return true;
      }
    else
      {
        ROS_WARN("[extra module]: fail to add new extra module %s, becuase it already exists", module_name.c_str());
        return false;
      }
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

        if (joint_num_ == 0) {
          // update robot model instantly
          updateRobotModel();
        }

        return true;
      }
  }

  void RobotModel::updateRobotModel()
  {
    KDL::JntArray dummy_joint_positions(tree_.getNrOfJoints());
    KDL::SetToZero(dummy_joint_positions);
    updateRobotModelImpl(dummy_joint_positions);
  }

  void RobotModel::updateRobotModel(const KDL::JntArray& joint_positions)
  {
    updateRobotModelImpl(joint_positions);
  }

  void RobotModel::updateRobotModel(const sensor_msgs::JointState& state)
  {
    updateRobotModel(jointMsgToKdl(state));
  }

  void RobotModel::updateRobotModelImpl(const KDL::JntArray& joint_positions)
  {
    joint_positions_ = joint_positions;

    KDL::RigidBodyInertia link_inertia = KDL::RigidBodyInertia::Zero();
    const auto seg_tf_map = fullForwardKinematics(joint_positions);
    setSegmentsTf(seg_tf_map);

    for(const auto& inertia : inertia_map_)
      {
        KDL::Frame f = seg_tf_map.at(inertia.first);
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
    KDL::Frame f_baselink = seg_tf_map.at(baselink_);
    KDL::Frame cog;
    cog.M = f_baselink.M * cog_desire_orientation_.Inverse();
    cog.p = link_inertia.getCOG();
    setCog(cog);
    mass_ = link_inertia.getMass();

    setInertia((cog.Inverse() * link_inertia).getRotationalInertia());
    setCog2Baselink(cog.Inverse() * f_baselink);

    /* thrust point based on COG */
    std::vector<KDL::Vector> rotors_origin_from_cog, rotors_normal_from_cog;
    for(int i = 0; i < rotor_num_; ++i)
      {
        std::string rotor = thrust_link_ + std::to_string(i + 1);
        KDL::Frame f = seg_tf_map.at(rotor);
        if(verbose_) ROS_WARN(" %s : [%f, %f, %f]", rotor.c_str(), f.p.x(), f.p.y(), f.p.z());
        rotors_origin_from_cog.push_back((cog.Inverse() * f).p);
        rotors_normal_from_cog.push_back((cog.Inverse() * f).M * KDL::Vector(0, 0, 1));
      }
    setRotorsNormalFromCog(rotors_normal_from_cog);
    setRotorsOriginFromCog(rotors_origin_from_cog);

    /* statics */
    calcStaticThrust();
    calcFeasibleControlFDists();
    calcFeasibleControlTDists();

    if (!initialized_) initialized_ = true;
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

  Eigen::VectorXd RobotModel::calcGravityWrenchOnRoot()
  {
    const auto seg_frames = getSegmentsTf();
    const auto& inertia_map = getInertiaMap();

    Eigen::MatrixXd root_rot = aerial_robot_model::kdlToEigen(getCogDesireOrientation<KDL::Rotation>() * seg_frames.at(baselink_).M.Inverse());
    Eigen::VectorXd wrench_g = Eigen::VectorXd::Zero(6);
    for(const auto& inertia : inertia_map)
      {
        Eigen::MatrixXd jacobi_root = Eigen::MatrixXd::Identity(3, 6);
        Eigen::Vector3d p = root_rot * aerial_robot_model::kdlToEigen(seg_frames.at(inertia.first).p + seg_frames.at(inertia.first).M * inertia.second.getCOG());
        jacobi_root.rightCols(3) = - aerial_robot_model::skew(p);
        wrench_g += jacobi_root.transpose() *  inertia.second.getMass() * (-gravity_3d_);
      }
    return wrench_g;
  }

  Eigen::MatrixXd RobotModel::calcWrenchMatrixOnCoG()
  {
    const std::vector<Eigen::Vector3d> p = getRotorsOriginFromCog<Eigen::Vector3d>();
    const std::vector<Eigen::Vector3d> u = getRotorsNormalFromCog<Eigen::Vector3d>();
    const auto& sigma = getRotorDirection();
    const int rotor_num = getRotorNum();
    const double m_f_rate = getMFRate();

    //Q : WrenchAllocationMatrix
    Eigen::MatrixXd Q(6, rotor_num);
    for (unsigned int i = 0; i < rotor_num; ++i) {
      Q.block(0, i, 3, 1) = u.at(i);
      Q.block(3, i, 3, 1) = p.at(i).cross(u.at(i)) + m_f_rate * sigma.at(i + 1) * u.at(i);
    }
    return Q;
  }

  void RobotModel::calcWrenchMatrixOnRoot()
  {
    const auto seg_frames = getSegmentsTf();
    const std::vector<Eigen::Vector3d>& u = getRotorsNormalFromCog<Eigen::Vector3d>();
    const auto& sigma = getRotorDirection();
    const int rotor_num = getRotorNum();
    const double m_f_rate = getMFRate();

    Eigen::MatrixXd root_rot = aerial_robot_model::kdlToEigen(getCogDesireOrientation<KDL::Rotation>() * seg_frames.at(baselink_).M.Inverse());

    q_mat_ = Eigen::MatrixXd::Zero(6, rotor_num);
    for (unsigned int i = 0; i < rotor_num; ++i) {
      std::string rotor = "thrust" + std::to_string(i + 1);
      Eigen::MatrixXd q_i = Eigen::MatrixXd::Identity(6, 6);
      Eigen::Vector3d p = root_rot * aerial_robot_model::kdlToEigen(seg_frames.at(rotor).p);
      q_i.bottomLeftCorner(3,3) = aerial_robot_model::skew(p);

      Eigen::VectorXd wrench_unit = Eigen::VectorXd::Zero(6);
      wrench_unit.head(3) = u.at(i);
      wrench_unit.tail(3) = m_f_rate * sigma.at(i + 1) * u.at(i);

      thrust_wrench_units_.at(i) = wrench_unit;
      thrust_wrench_allocations_.at(i) = q_i;
      q_mat_.col(i) = q_i * wrench_unit;
    }
  }

  void RobotModel::calcStaticThrust()
  {
    calcWrenchMatrixOnRoot(); // update Q matrix
    Eigen::VectorXd wrench_g = calcGravityWrenchOnRoot();
    static_thrust_ = aerial_robot_model::pseudoinverse(q_mat_) * (-wrench_g);
  }


  bool RobotModel::stabilityCheck(bool verbose)
  {
    if(fc_f_min_ < fc_f_min_thre_)
      {
        if(verbose)
          ROS_ERROR_STREAM("the min distance to the plane of feasible control force convex " << fc_f_min_ << " is lower than the threshold " <<  fc_f_min_thre_);
          return false;
      }

    if(fc_t_min_ < fc_t_min_thre_)
      {
        if(verbose)
          ROS_ERROR_STREAM("the min distance to the plane of feasible control torque convex " << fc_t_min_ << " is lower than the threshold " <<  fc_t_min_thre_);
        return false;
      }

    if(static_thrust_.maxCoeff() > thrust_max_ || static_thrust_.minCoeff() < thrust_min_)
      {
        if(verbose)
          ROS_ERROR("Invalid static thrust, max: %f, min: %f", static_thrust_.maxCoeff(), static_thrust_.minCoeff());
        return false;
      }

    return true;
  }

  double RobotModel::calcTripleProduct(const Eigen::Vector3d& ui, const Eigen::Vector3d& uj, const Eigen::Vector3d& uk)
  {
    Eigen::Vector3d uixuj = ui.cross(uj);
    if (uixuj.norm() < 0.00001) {
      return 0.0;
    }
    return uixuj.dot(uk) / uixuj.norm();
  }

  std::vector<Eigen::Vector3d> RobotModel::calcV()
  {
    const std::vector<Eigen::Vector3d> p = getRotorsOriginFromCog<Eigen::Vector3d>();
    const std::vector<Eigen::Vector3d> u = getRotorsNormalFromCog<Eigen::Vector3d>();
    const auto& sigma = getRotorDirection();
    const int rotor_num = getRotorNum();
    const double m_f_rate = getMFRate();
    std::vector<Eigen::Vector3d> v(rotor_num);

    for (int i = 0; i < rotor_num; ++i)
      v.at(i) = p.at(i).cross(u.at(i)) + m_f_rate * sigma.at(i + 1) * u.at(i);
    return v;
  }

  void RobotModel::calcFeasibleControlFDists()
  {
    const int rotor_num = getRotorNum();
    const double thrust_max = getThrustUpperLimit();

    const auto& u = getRotorsNormalFromCog<Eigen::Vector3d>();
    Eigen::Vector3d gravity_force = getMass() * gravity_3d_;

    int index = 0;
    for (int i = 0; i < rotor_num; ++i) {
      const Eigen::Vector3d& u_i = u.at(i);
      for (int j = 0; j < rotor_num; ++j) {
        if (i == j) continue;
        const Eigen::Vector3d& u_j = u.at(j);

        double dist_ij = 0.0;
        for (int k = 0; k < rotor_num; ++k) {
          if (i == k || j == k) continue;
          const Eigen::Vector3d& u_k = u.at(k);
          double u_triple_product = calcTripleProduct(u_i, u_j, u_k);
          dist_ij += std::max(0.0, u_triple_product * thrust_max);
        }

        Eigen::Vector3d uixuj = u_i.cross(u_j);
        fc_f_dists_(index) = fabs(dist_ij - (uixuj.dot(gravity_force) / uixuj.norm()));
        index++;
      }
    }
    fc_f_min_ = fc_f_dists_.minCoeff();
  }

  void RobotModel::calcFeasibleControlTDists()
  {
    const int rotor_num = getRotorNum();
    const double thrust_max = getThrustUpperLimit();

    const auto v = calcV();
    int index = 0;

    for (int i = 0; i < rotor_num; ++i) {
      const Eigen::Vector3d& v_i = v.at(i);
      for (int j = 0; j < rotor_num; ++j) {
        if (i == j) continue;
        const Eigen::Vector3d& v_j = v.at(j);
        double dist_ij = 0.0;
        for (int k = 0; k < rotor_num; ++k) {
          if (i == k || j == k) continue;
          const Eigen::Vector3d& v_k = v.at(k);
          double v_triple_product = calcTripleProduct(v_i, v_j, v_k);
          dist_ij += std::max(0.0, v_triple_product * thrust_max);
        }
        fc_t_dists_(index) = dist_ij;
        index++;
      }
    }

    fc_t_min_ = fc_t_dists_.minCoeff();
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

  KDL::JntArray RobotModel::convertEigenToKDL(const Eigen::VectorXd& joint_vector)
  {
    const auto& joint_indices = getJointIndices();
    KDL::JntArray joint_positions(getTree().getNrOfJoints());
    for (unsigned int i = 0; i < joint_indices.size(); ++i) {
      joint_positions(joint_indices.at(i)) = joint_vector(i);
    }
    return joint_positions;
  }

} //namespace aerial_robot_model

