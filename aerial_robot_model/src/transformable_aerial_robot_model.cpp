#include <aerial_robot_model/transformable_aerial_robot_model.h>

namespace aerial_robot_model {

  RobotModel::RobotModel(std::string baselink, std::string thrust_link, bool verbose):
    baselink_(baselink),
    thrust_link_(thrust_link),
    verbose_(verbose),
    rotor_num_(0)
  {
    /* robot model */
    if (!model_.initParam("robot_description"))
      ROS_ERROR("Failed to extract urdf model from rosparam");
    if (!kdl_parser::treeFromUrdfModel(model_, tree_))
      ROS_ERROR("Failed to extract kdl tree from xml robot description");

    inertialSetup(tree_.getRootSegment()->second);
    resolveLinkLength();

    ROS_ERROR("[kinematics] rotor num; %d", rotor_num_);
    rotors_origin_from_cog_.resize(rotor_num_);
  }

  void RobotModel::updateRobotModel(const KDL::JntArray& joint_positions)
  {
    KDL::TreeFkSolverPos_recursive fk_solver(tree_);
    KDL::RigidBodyInertia link_inertia = KDL::RigidBodyInertia::Zero();

    for(const auto& inertia : inertia_map_)
      {
        KDL::Frame f;
        int status = fk_solver.JntToCart(joint_positions, f, inertia.first);
        if(status < 0) ROS_ERROR("can not solve FK to inertia element: %s", inertia.first.c_str());
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
    KDL::Frame f_baselink;
    int status = fk_solver.JntToCart(joint_positions, f_baselink, baselink_);
    if(status < 0) ROS_ERROR("can not solve FK to the baselink: %s", baselink_.c_str());
    cog_.M = f_baselink.M * cog_desire_orientation_.Inverse();
    cog_.p = link_inertia.getCOG();
    mass_ = link_inertia.getMass();
    cog2baselink_transform_ = cog_.Inverse() * f_baselink;

    /* thrust point based on COG */
    for(int i = 0; i < rotor_num_; ++i)
      {
        std::string rotor = thrust_link_ + std::to_string(i + 1);

        KDL::Frame f;
        int status = fk_solver.JntToCart(joint_positions, f, rotor);
        if(status < 0) ROS_ERROR("can not solve FK to rotor: %s", rotor.c_str());
        if(verbose_) ROS_WARN(" %s status is : %d, [%f, %f, %f]", rotor.c_str(), status, f.p.x(), f.p.y(), f.p.z());
        rotors_origin_from_cog_.at(i) = (cog_.Inverse() * f).p;
      }

    link_inertia_cog_ = (cog_.Inverse() * link_inertia).getRotationalInertia();
  }

  void RobotModel::updateRobotModel(const sensor_msgs::JointState& state)
  {
    updateRobotModel(jointMsgToKdl(state));
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

  bool RobotModel::addExtraModuleCallback(const aerial_robot_model::AddExtraModule::Request &req, aerial_robot_model::AddExtraModule::Response &res)
  {
    switch(req.action)
      {
      case aerial_robot_model::AddExtraModule::Request::ADD:
        {
          geometry_msgs::TransformStamped ts;
          ts.transform = req.transform;
          KDL::Frame f = tf2::transformToKDL(ts);
          KDL::RigidBodyInertia rigid_body_inertia(req.inertia.m, KDL::Vector(req.inertia.com.x, req.inertia.com.y, req.inertia.com.z),
                                                   KDL::RotationalInertia(req.inertia.ixx, req.inertia.iyy,
                                                                          req.inertia.izz, req.inertia.ixy,
                                                                          req.inertia.ixz, req.inertia.iyz));
          res.status = addExtraModule(req.module_name, req.parent_link_name, f, rigid_body_inertia);
          return res.status;
          break;
        }
      case aerial_robot_model::AddExtraModule::Request::REMOVE:
        {
          res.status = removeExtraModule(req.module_name);
          return res.status;
          break;
        }
      default:
        {
          ROS_WARN("[extra module]: wrong action %d", req.action);
          return false;
          break;
        }
      }
    ROS_ERROR("[extra module]: should not reach here ");
    return false;
  }

  void RobotModel::setActuatorJointMap(const sensor_msgs::JointState& actuator_state)
  {
    /* CAUTION: be sure that the joints are in order !!!!!!! */
    for(auto itr = actuator_state.name.begin(); itr != actuator_state.name.end(); ++itr)
      {
        if(itr->find("joint") != std::string::npos)
          actuator_joint_map_.push_back(std::distance(actuator_state.name.begin(), itr));
      }
  }

  KDL::RigidBodyInertia RobotModel::inertialSetup(const KDL::TreeElement& tree_element)
  {
    const KDL::Segment current_seg = GetTreeElementSegment(tree_element);

    KDL::RigidBodyInertia current_seg_inertia = current_seg.getInertia();
    if(verbose_) ROS_WARN("segment %s, mass is: %f", current_seg.getName().c_str(), current_seg_inertia.getMass());

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
            actuator_map_.insert(std::make_pair(current_seg.getJoint().getName(), tree_element.q_nr));
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

  void RobotModel::resolveLinkLength() //TODO hard coding
  {
    KDL::JntArray joint_positions(tree_.getNrOfJoints());
    KDL::TreeFkSolverPos_recursive fk_solver(tree_);
    KDL::Frame f_link2, f_link3;
    fk_solver.JntToCart(joint_positions, f_link2, "link2"); //hard coding //TODO
    fk_solver.JntToCart(joint_positions, f_link3, "link3"); //hard coding //TODO
    link_length_ = (f_link3.p - f_link2.p).Norm();
  }

  KDL::Frame RobotModel::forwardKinematics(std::string link, const KDL::JntArray& joint_positions) const
  {
    KDL::TreeFkSolverPos_recursive fk_solver(tree_);
    KDL::Frame f;
    int status = fk_solver.JntToCart(joint_positions, f, link);
    if(status < 0) ROS_ERROR("can not solve FK to link: %s", link.c_str());

    return f;
  }

  KDL::Frame RobotModel::forwardKinematics(std::string link, const sensor_msgs::JointState& state) const
  {
    return forwardKinematics(link, jointMsgToKdl(state));
  }

  KDL::JntArray RobotModel::jointMsgToKdl(const sensor_msgs::JointState& state) const
  {
    KDL::JntArray joint_positions(tree_.getNrOfJoints());
    for(unsigned int i = 0; i < state.position.size(); ++i)
      {
        auto itr = actuator_map_.find(state.name[i]);
        if(itr != actuator_map_.end()) joint_positions(itr->second) = state.position[i];
      }
    return joint_positions;
  }

} //namespace aerial_robot_model
