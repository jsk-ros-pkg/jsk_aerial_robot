#include <hydrus/transform_control.h>

TransformController::TransformController(ros::NodeHandle nh, ros::NodeHandle nh_private, bool callback_flag):
  nh_(nh), nh_private_(nh_private),
  realtime_control_flag_(true),
  callback_flag_(callback_flag),
  kinematics_flag_(false),
  rotor_num_(0), link_length_(0),
  p_det_(0), stability_margin_(0),
  ik_result_(false), multilink_type_(MULTILINK_TYPE_SE2)
{
  /* robot model */
  if (!model_.initParam("robot_description"))
    ROS_ERROR("Failed to extract urdf model from rosparam");
  if (!kdl_parser::treeFromUrdfModel(model_, tree_))
    ROS_ERROR("Failed to extract kdl tree from xml robot description");

  nh_private_.param("verbose", verbose_, false);
  ROS_ERROR("ns is %s", nh_private_.getNamespace().c_str());

  /* base link */
  nh_private_.param("baselink", baselink_, std::string("link1"));
  if(verbose_) std::cout << "baselink: " << baselink_ << std::endl;
  nh_private_.param("thrust_link", thrust_link_, std::string("thrust"));
  if(verbose_) std::cout << "thrust_link: " << thrust_link_ << std::endl;

  addChildren(tree_.getRootSegment());
  getLinkLength();

  /* for inverse kinematics */
  for(auto itr = model_.joints_.begin(); itr != model_.joints_.end(); itr++)
    {
      if(itr->first.find("joint1") != std::string::npos)
        {
          joint_angle_min_ = itr->second->limits->lower;
          joint_angle_max_ = itr->second->limits->upper;
          ROS_WARN("the angle range: [%f, %f]", joint_angle_min_, joint_angle_max_);
          break;
        }

      if(itr->second->axis.z == 0) multilink_type_ = MULTILINK_TYPE_SE3;
    }

  ROS_ERROR("[kinematics] rotor num; %d, link model: %d", rotor_num_, multilink_type_);

  initParam();

  rotors_origin_from_cog_.resize(rotor_num_);

  /* Linear Quadratic Control */
  //U
  P_ = Eigen::MatrixXd::Zero(4,rotor_num_);

  //Q
  q_diagonal_ = Eigen::VectorXd::Zero(LQI_FOUR_AXIS_MODE * 3);
  q_diagonal_ << q_roll_,q_roll_d_,q_pitch_,q_pitch_d_,q_z_,q_z_d_,q_yaw_,q_yaw_d_, q_roll_i_,q_pitch_i_,q_z_i_,q_yaw_i_;
  //std::cout << "Q elements :"  << std::endl << q_diagonal_ << std::endl;

  lqi_mode_ = LQI_FOUR_AXIS_MODE;

  //those publisher is published from func param2controller
  std::string rpy_gain_pub_name;
  nh_private_.param("rpy_gain_pub_name", rpy_gain_pub_name, std::string("/rpy_gain"));
  rpy_gain_pub_ = nh_.advertise<spinal::RollPitchYawTerms>(rpy_gain_pub_name, 1);
  four_axis_gain_pub_ = nh_.advertise<aerial_robot_msgs::FourAxisGain>("/four_axis_gain", 1);
  transform_pub_ = nh_.advertise<geometry_msgs::TransformStamped>("/cog2baselink", 1);
  p_matrix_pseudo_inverse_inertia_pub_ = nh_.advertise<spinal::PMatrixPseudoInverseWithInertia>("p_matrix_pseudo_inverse_inertia", 1);

  //dynamic reconfigure server
  lqi_server_ = new dynamic_reconfigure::Server<hydrus::LQIConfig>(nh_private_);
  dynamic_reconf_func_lqi_ = boost::bind(&TransformController::cfgLQICallback, this, _1, _2);
  lqi_server_->setCallback(dynamic_reconf_func_lqi_);

  /* ros service for extra module */
  add_extra_module_service_ = nh_.advertiseService("add_extra_module", &TransformController::addExtraModuleCallback, this);
  end_effector_ik_service_ = nh_.advertiseService("end_effector_ik", &TransformController::endEffectorIkCallback, this);

  /* defualt callback */
  desire_coordinate_sub_ = nh_.subscribe("/desire_coordinate", 1, &TransformController::desireCoordinateCallback, this);

  if(callback_flag_)
    {
      realtime_control_sub_ = nh_.subscribe("realtime_control", 1, &TransformController::realtimeControlCallback, this, ros::TransportHints().tcpNoDelay());
      std::string joint_state_sub_name;
      nh_private_.param("joint_state_sub_name", joint_state_sub_name, std::string("joint_state"));
      joint_state_sub_ = nh_.subscribe(joint_state_sub_name, 1, &TransformController::jointStateCallback, this);

      control_thread_ = boost::thread(boost::bind(&TransformController::control, this));
    }
}

void TransformController::addChildren(const KDL::SegmentMap::const_iterator segment)
{
  const std::string& parent = GetTreeElementSegment(segment->second).getName();

  const std::vector<KDL::SegmentMap::const_iterator>& children = GetTreeElementChildren(segment->second);

  for (unsigned int i=0; i<children.size(); i++)
    {
      //ROS_WARN("child num; %d", children.size());
      const KDL::Segment& child = GetTreeElementSegment(children[i]->second);

      if (child.getJoint().getType() == KDL::Joint::None) {
        if (model_.getJoint(child.getJoint().getName()) && model_.getJoint(child.getJoint().getName())->type == urdf::Joint::FLOATING) {
          if(verbose_) ROS_INFO("Floating joint. Not adding segment from %s to %s. This TF can not be published based on joint_states info", parent.c_str(), child.getName().c_str());
        }
        else {
          if(verbose_) ROS_INFO("Adding fixed segment from %s to %s, joint: %s, [%f, %f, %f], q_nr: %d", parent.c_str(), child.getName().c_str(), child.getJoint().getName().c_str(), child.getFrameToTip().p.x(), child.getFrameToTip().p.y(), child.getFrameToTip().p.z(), children[i]->second.q_nr);
        }

        /* for the most parent link, e.g. link1 */
        if(inertia_map_.size() == 0 && parent.find("root") != std::string::npos)
          {
            inertia_map_.insert(std::make_pair(child.getName(), child.getInertia()));
            if(verbose_) ROS_WARN("Add new inertia base link: %s", child.getName().c_str());
            /*
            std::cout << "m: " << inertia_map_.find(child.getName())->second.getMass() << std::endl;
            std::cout << "p: \n" << Eigen::Map<const Eigen::Vector3d>(inertia_map_.find(child.getName())->second.getCOG().data) << std::endl;
            std::cout << "I: \n" << Eigen::Map<const Eigen::Matrix3d>(inertia_map_.find(child.getName())->second.getRotationalInertia().data) << std::endl;
            */
          }
        else
          {
            /* add the inertia to the parent link */
            std::map<std::string, KDL::RigidBodyInertia>::iterator it = inertia_map_.find(parent);
            KDL::RigidBodyInertia parent_prev_inertia = it->second;
            inertia_map_[parent] = child.getFrameToTip() * child.getInertia() + parent_prev_inertia;

            /*
            KDL:: RigidBodyInertia tmp = child.getFrameToTip() * child.getInertia();
            std::cout << "m: " << tmp.getMass() << std::endl;
            std::cout << "p: \n" << Eigen::Map<const Eigen::Vector3d>(tmp.getCOG().data) << std::endl;
            std::cout << "I: \n" << Eigen::Map<const Eigen::Matrix3d>(tmp.getRotationalInertia().data) << std::endl;
            */
          }
      }
      else {
        if(verbose_) ROS_INFO("Adding moving segment from %s to %s, joint: %s,  [%f, %f, %f], q_nr: %d", parent.c_str(), child.getName().c_str(), child.getJoint().getName().c_str(), child.getFrameToTip().p.x(), child.getFrameToTip().p.y(), child.getFrameToTip().p.z(), children[i]->second.q_nr);


        /* add the new inertia base (child) link if the joint is not a rotor */
        if(child.getJoint().getName().find("rotor") == std::string::npos)
          {
            /* create a new inertia base link */
            inertia_map_.insert(std::make_pair(child.getName(), child.getInertia()));
            actuator_map_.insert(std::make_pair(child.getJoint().getName(), children[i]->second.q_nr));
            if(verbose_) ROS_WARN("Add new inertia base link: %s", child.getName().c_str());
          }
        else
          {
            std::map<std::string, KDL::RigidBodyInertia>::iterator it = inertia_map_.find(parent);
            KDL::RigidBodyInertia parent_prev_inertia = it->second;
            inertia_map_[parent] = child.getFrameToTip() * child.getInertia() + parent_prev_inertia;

            /* add the rotor direction */
            if(verbose_) ROS_WARN("%s, rototation is %f", child.getJoint().getName().c_str(), child.getJoint().JointAxis().z());
            rotor_direction_.insert(std::make_pair(std::atoi(child.getJoint().getName().substr(5).c_str()), child.getJoint().JointAxis().z()));
          }
      }
      /* count the rotor */
      if(child.getName().find(thrust_link_.c_str()) != std::string::npos) rotor_num_++;
      addChildren(children[i]);
    }
}

void TransformController::getLinkLength()
{
  unsigned int nj = tree_.getNrOfJoints();
  KDL::JntArray joint_positions(nj);
  KDL::TreeFkSolverPos_recursive fk_solver(tree_);
  KDL::Frame f_link2, f_link3;
  fk_solver.JntToCart(joint_positions, f_link2, "link2"); //hard coding
  fk_solver.JntToCart(joint_positions, f_link3, "link3"); //hard coding
  link_length_ = (f_link3.p - f_link2.p).Norm();
  //ROS_ERROR("Update link length: %f", link_length_);
}

TransformController::~TransformController()
{
  if(callback_flag_)
    {
      control_thread_.interrupt();
      control_thread_.join();
    }
}

void TransformController::realtimeControlCallback(const std_msgs::UInt8ConstPtr & msg)
{
  if(msg->data == 1)
    {
      if(debug_verbose_) ROS_WARN("start realtime control");
      realtime_control_flag_ = true;
    }
  else if(msg->data == 0)
    {
      if(debug_verbose_) ROS_WARN("stop realtime control");
      realtime_control_flag_ = false;
    }
}

void TransformController::initParam()
{
  nh_private_.param("control_rate", control_rate_, 15.0);
  if(verbose_) std::cout << "control_rate: " << std::setprecision(3) << control_rate_ << std::endl;

  nh_private_.param("only_three_axis_mode", only_three_axis_mode_, false);
  nh_private_.param("gyro_moment_compensation", gyro_moment_compensation_, false);
  nh_private_.param("control_verbose", control_verbose_, false);
  nh_private_.param("kinematic_verbose", kinematic_verbose_, false);
  nh_private_.param("debug_verbose", debug_verbose_, false);
  nh_private_.param("a_dash_eigen_calc_flag", a_dash_eigen_calc_flag_, false);

  /* propeller direction and lqi R */
  r_.resize(rotor_num_);
  for(int i = 0; i < rotor_num_; i++)
    {
      std::stringstream ss;
      ss << i + 1;
      /* R */
      nh_private_.param(std::string("r") + ss.str(), r_[i], 1.0);
    }

    nh_private_.param ("stability_margin_thre", stability_margin_thre_, 0.01);
  if(verbose_) std::cout << "stability margin thre: " << std::setprecision(3) << stability_margin_thre_ << std::endl;
  nh_private_.param ("p_determinant_thre", p_det_thre_, 1e-6);
  if(verbose_) std::cout << "p determinant thre: " << std::setprecision(3) << p_det_thre_ << std::endl;
  nh_private_.param ("f_max", f_max_, 8.6);
  if(verbose_) std::cout << "f_max: " << std::setprecision(3) << f_max_ << std::endl;
  nh_private_.param ("f_min", f_min_, 2.0);
  if(verbose_) std::cout << "f_min: " << std::setprecision(3) << f_min_ << std::endl;

  nh_private_.param ("q_roll", q_roll_, 1.0);
  if(verbose_) std::cout << "Q: q_roll: " << std::setprecision(3) << q_roll_ << std::endl;
  nh_private_.param ("q_roll_d", q_roll_d_, 1.0);
  if(verbose_) std::cout << "Q: q_roll_d: " << std::setprecision(3) << q_roll_d_ << std::endl;
  nh_private_.param ("q_pitch", q_pitch_, 1.0);
  if(verbose_) std::cout << "Q: q_pitch: " << std::setprecision(3) << q_pitch_ << std::endl;
  nh_private_.param ("q_pitch_d", q_pitch_d_,  1.0);
  if(verbose_) std::cout << "Q: q_pitch_d: " << std::setprecision(3) << q_pitch_d_ << std::endl;
  nh_private_.param ("q_yaw", q_yaw_, 1.0);
  if(verbose_) std::cout << "Q: q_yaw: " << std::setprecision(3) << q_yaw_ << std::endl;
  nh_private_.param ("strong_q_yaw", strong_q_yaw_, 1.0);
  if(verbose_) std::cout << "Q: strong_q_yaw: " << std::setprecision(3) << strong_q_yaw_ << std::endl;
  nh_private_.param ("q_yaw_d", q_yaw_d_, 1.0);
  if(verbose_) std::cout << "Q: q_yaw_d: " << std::setprecision(3) << q_yaw_d_ << std::endl;
  nh_private_.param ("q_z", q_z_, 1.0);
  if(verbose_) std::cout << "Q: q_z: " << std::setprecision(3) << q_z_ << std::endl;
  nh_private_.param ("q_z_d", q_z_d_, 1.0);
  if(verbose_) std::cout << "Q: q_z_d: " << std::setprecision(3) << q_z_d_ << std::endl;

  nh_private_.param ("q_roll_i", q_roll_i_, 1.0);
  if(verbose_) std::cout << "Q: q_roll_i: " << std::setprecision(3) << q_roll_i_ << std::endl;
  nh_private_.param ("q_pitch_i", q_pitch_i_, 1.0);
  if(verbose_) std::cout << "Q: q_pitch_i: " << std::setprecision(3) << q_pitch_i_ << std::endl;
  nh_private_.param ("q_yaw_i", q_yaw_i_, 1.0);
  if(verbose_) std::cout << "Q: q_yaw_i: " << std::setprecision(3) << q_yaw_i_ << std::endl;
  nh_private_.param ("q_z_i", q_z_i_, 1.0);
  if(verbose_) std::cout << "Q: q_z_i: " << std::setprecision(3) << q_z_i_ << std::endl;

  /* inverse kinematics */
  nh_private_.param ("differential_motion_count", differential_motion_count_, 100);
  if(verbose_) std::cout << "differential_motion_count: " << differential_motion_count_ << std::endl;
  nh_private_.param ("ee_pos_err_thre", ee_pos_err_thre_, 0.001); //[m]
  if(verbose_) std::cout << "ee_pos_err_thre: " << std::setprecision(3) << ee_pos_err_thre_ << std::endl;
  nh_private_.param ("ee_rot_err_thre", ee_rot_err_thre_, 0.017); //[rad]
  if(verbose_) std::cout << "ee_rot_err_thre: " << std::setprecision(3) << ee_rot_err_thre_ << std::endl;
  nh_private_.param ("ee_pos_err_max", ee_pos_err_max_, 0.1); //[m]
  if(verbose_) std::cout << "ee_pos_err_max: " << std::setprecision(3) << ee_pos_err_max_ << std::endl;
  nh_private_.param ("ee_rot_err_max", ee_rot_err_max_, 0.17); //[rad]
  if(verbose_) std::cout << "ee_rot_err_max: " << std::setprecision(3) << ee_rot_err_max_ << std::endl;
  nh_private_.param ("w_ik_constraint", w_ik_constraint_, 1.0);
  if(verbose_) std::cout << "w_ik_constraint: " << std::setprecision(3) << w_ik_constraint_ << std::endl;
  nh_private_.param ("w_joint_vel", w_joint_vel_, 0.001);
  if(verbose_) std::cout << "w_joint_vel: " << std::setprecision(3) << w_joint_vel_ << std::endl;
  nh_private_.param ("w_root_vel", w_root_vel_, 0.001);
  if(verbose_) std::cout << "w_root_vel: " << std::setprecision(3) << w_root_vel_ << std::endl;
  nh_private_.param ("thre_joint_vel", thre_joint_vel_, 0.1);
  if(verbose_) std::cout << "thre_joint_vel: " << std::setprecision(3) << thre_joint_vel_ << std::endl;
  nh_private_.param ("thre_root_vel", thre_root_vel_, 0.1);
  if(verbose_) std::cout << "thre_root_vel: " << std::setprecision(3) << thre_root_vel_ << std::endl;

  nh_private_.param ("joint_vel_constraint_range", joint_vel_constraint_range_, 0.2);
  if(verbose_) std::cout << "joint_vel_constraint_range: " << std::setprecision(3) << joint_vel_constraint_range_ << std::endl;
  nh_private_.param ("joint_vel_forbidden_range", joint_vel_forbidden_range_, 0.1);
  if(verbose_) std::cout << "joint_vel_forbidden_range: " << std::setprecision(3) << joint_vel_forbidden_range_ << std::endl;

  /* dynamics: motor */
  ros::NodeHandle control_node("/motor_info");
  control_node.param("m_f_rate", m_f_rate_, 0.01);
  if(verbose_) std::cout << "m_f_rate: " << std::setprecision(3) << m_f_rate_ << std::endl;
}

void TransformController::control()
{
  ros::Rate loop_rate(control_rate_);
  static int i = 0;
  static int cnt = 0;

  if(!realtime_control_flag_) return;
  while(ros::ok())
    {
      if(ik_result_)
        {
          /* publish joint angle and tf */
          ros::Time now_time = ros::Time::now();
          br_.sendTransform(tf::StampedTransform(target_root_pose_, now_time, "ik_world", "root"));
          br_.sendTransform(tf::StampedTransform(target_ee_pose_, now_time, "ik_world", "target_ee"));

          tf::Transform end_link_ee_tf;
          end_link_ee_tf.setIdentity();
          end_link_ee_tf.setOrigin(tf::Vector3(link_length_, 0, 0));
          std::stringstream ss; ss << rotor_num_;
          br_.sendTransform(tf::StampedTransform(end_link_ee_tf, now_time, std::string("link") + ss.str(), "ee"));
          target_joint_vector_.header.stamp = now_time;
          joint_state_pub_.publish(target_joint_vector_);
        }

      if(debug_verbose_) ROS_ERROR("start lqi");
      lqi();
      if(debug_verbose_) ROS_ERROR("finish lqi");

      loop_rate.sleep();
    }
}

void TransformController::desireCoordinateCallback(const spinal::DesireCoordConstPtr & msg)
{
  setCogDesireOrientation(KDL::Rotation::RPY(msg->roll, msg->pitch, msg->yaw));
}

void TransformController::jointStateCallback(const sensor_msgs::JointStateConstPtr& state)
{
  joint_state_stamp_ = state->header.stamp;
  current_joint_state_ = *state;

  if(debug_verbose_) ROS_ERROR("start kinematics");
  forwardKinematics(current_joint_state_);
  if(debug_verbose_) ROS_ERROR("finish kinematics");

  br_.sendTransform(tf::StampedTransform(getCog(), state->header.stamp, GetTreeElementSegment(tree_.getRootSegment()->second).getName(), "cog"));

  if(!kinematics_flag_)
    {
      ROS_ERROR("the total mass is %f", getMass());
      kinematics_flag_ = true;
    }
}

void TransformController::forwardKinematics(sensor_msgs::JointState state)
{
  KDL::TreeFkSolverPos_recursive fk_solver(tree_);
  KDL::JntArray joint_positions(tree_.getNrOfJoints());   /* set joint array */

  unsigned int j = 0;
  for(unsigned int i = 0; i < state.position.size(); i++)
    {
      std::map<std::string, uint32_t>::iterator itr = actuator_map_.find(state.name[i]);

      if(itr != actuator_map_.end())  joint_positions(actuator_map_.find(state.name[i])->second) = state.position[i];
      //else ROS_FATAL("transform_control: no matching joint called %s", state.name[i].c_str());
    }

  KDL::RigidBodyInertia link_inertia = KDL::RigidBodyInertia::Zero();
  KDL::Frame cog_frame;
  for(auto it = inertia_map_.begin(); it != inertia_map_.end(); ++it)
    {
      KDL::Frame f;
      int status = fk_solver.JntToCart(joint_positions, f, it->first);
      //ROS_ERROR(" %s status is : %d, [%f, %f, %f]", it->first.c_str(), status, f.p.x(), f.p.y(), f.p.z());
      KDL::RigidBodyInertia link_inertia_tmp = link_inertia;
      link_inertia = link_inertia_tmp + f * it->second;

      /* process for the extra module */
      for(std::map<std::string, KDL::Segment>::iterator it_extra = extra_module_map_.begin(); it_extra != extra_module_map_.end(); it_extra++)
        {
          if(it_extra->second.getName() == it->first)
            {
              //ROS_INFO("[kinematics]: find the extra module %s", it_extra->second.getName().c_str());
              KDL::RigidBodyInertia link_inertia_tmp = link_inertia;
              link_inertia = link_inertia_tmp + f *  (it_extra->second.getFrameToTip() * it_extra->second.getInertia());
            }
        }

    }
  /* CoG */
  KDL::Frame f_baselink;
  int status = fk_solver.JntToCart(joint_positions, f_baselink, baselink_);
  if(status < 0) ROS_ERROR("can not get FK to the baselink: %s", baselink_.c_str());
  cog_frame.M = f_baselink.M * cog_desire_orientation_.Inverse();
  cog_frame.p = link_inertia.getCOG();
  tf::Transform cog_transform;
  tf::transformKDLToTF(cog_frame, cog_transform);
  setCog(cog_transform);
  setMass(link_inertia.getMass());

  /* thrust point based on COG */
  std::vector<Eigen::Vector3d> f_rotors;
  for(int i = 0; i < rotor_num_; i++)
    {
      std::stringstream ss;
      ss << i + 1;
      std::string rotor = thrust_link_ + ss.str();

      KDL::Frame f;
      int status = fk_solver.JntToCart(joint_positions, f, rotor);
      //if(verbose) ROS_WARN(" %s status is : %d, [%f, %f, %f]", rotor.c_str(), status, f.p.x(), f.p.y(), f.p.z());
      f_rotors.push_back(Eigen::Map<const Eigen::Vector3d>((cog_frame.Inverse() * f).p.data));

      //std::cout << "rotor" << i + 1 << ": \n"<< f_rotors[i] << std::endl;
    }

  setRotorsOriginFromCog(f_rotors);
  KDL::RigidBodyInertia link_inertia_from_cog = cog_frame.Inverse() * link_inertia;
  setInertia(Eigen::Map<const Eigen::Matrix3d>(link_inertia_from_cog.getRotationalInertia().data));

  tf::Transform cog2baselink_transform;
  tf::transformKDLToTF(cog_frame.Inverse() * f_baselink, cog2baselink_transform);
  setCog2Baselink(cog2baselink_transform);
}


bool TransformController::endEffectorIkCallback(hydrus::TargetPose::Request  &req,
                                                hydrus::TargetPose::Response &res)
{
  /* stop rosnode: joint_state_publisher, and publisher from this node instead */
  std::string joint_state_publisher_node_name = joint_state_sub_.getTopic().substr(0, joint_state_sub_.getTopic().find("/joint")) + std::string("/joint_state_publisher_");
  std::string command_string = std::string("rosnode kill ") + joint_state_publisher_node_name.c_str();
  system(command_string.c_str());
  joint_state_sub_.shutdown();
  joint_state_pub_ = nh_.advertise<sensor_msgs::JointState>(joint_state_sub_.getTopic().substr(0, joint_state_sub_.getTopic().find("/joint")) + std::string("/joint_states"), 1);


  /* start IK */
  target_root_pose_.setIdentity();
  tf::Quaternion q; q.setRPY(req.target_rot.x, req.target_rot.y, req.target_rot.z);
  target_ee_pose_ = tf::Transform (q, tf::Vector3(req.target_pos.x, req.target_pos.y, req.target_pos.z));
  if(!inverseKinematics(target_ee_pose_, target_joint_vector_, target_root_pose_,
                        req.ik_algorithm, req.orientation, req.full_body, req.debug))
    return false;

  /* ik visualization */
  ik_result_ = true;

  return true;
}

bool TransformController::inverseKinematics(tf::Transform target_ee_pose, sensor_msgs::JointState& target_joint_vector, tf::Transform& target_root_pose, int ik_algorithm, bool orientation, bool full_body, bool debug)
{
  /* get initial joint state from sensor_msgs::State and kinematics and modelling */
  target_joint_vector = current_joint_state_;

  /* CAUTION: be sure that the joints are in order !!!!!!! */
  std::vector<double> real_joints; // considering other type of joint (e.g. gimbal)
  std::vector<int> joints_map; // joint index map, temp
  for(auto itr = target_joint_vector.name.begin(); itr != target_joint_vector.name.end(); ++itr)
    {
      if(itr->find("joint") != std::string::npos)
        {
          real_joints.push_back(target_joint_vector.position.at(std::distance(target_joint_vector.name.begin(), itr)));
          joints_map.push_back(std::distance(target_joint_vector.name.begin(), itr));
        }
    }
  Eigen::VectorXd real_joint_vector = Eigen::Map<Eigen::VectorXd>(&real_joints[0], real_joints.size());

  /* get link list */
  KDL::Chain chain;
  std::stringstream ss;
  ss << rotor_num_;
  tree_.getChain(std::string("root"), std::string("link") + ss.str(), chain);
  /* definition of end coords */
  /* the end point of the end link */
  KDL::Segment ee_seg(std::string("end_effector"),
                      KDL::Joint(KDL::Joint::None),
                      KDL::Frame(KDL::Vector(link_length_, 0, 0)));
  chain.addSegment(ee_seg);

  /* set the link1(root) as the base link */
  baselink_ = std::string("link1");

  Eigen::MatrixXd jacobian;
  calcJointJacobian(chain, real_joint_vector, jacobian, orientation, full_body, false); // calc the first jacobian with init jont vector

  /* initialize the root pose if necessary */
  if(!full_body) target_root_pose.setIdentity();

  /* QP */
  /*
     1. QP cost function in qp Oases is : 1/2 * x^T H x + g^T x
        - so the hessian Matrix H shoube be multiplied by 2, or gradian g shoud be half.
     2. n_wrs is a total number in whole sequnece process, the value decrease every loop.
     3. the matrix in Eigen library is colunm-major, so use transpose to get row-major data.
        Plus, (A.transpose()).data is still column-mojar, please assign to a new matrix
  */
  boost::shared_ptr<SQProblem> qp_solver(new SQProblem(jacobian.cols(), rotor_num_ + 2));
  // constraint: 1: stability margin: 1; 2: p determinant thre;  3 flight statibility: rotor_num
  bool qp_init_flag = true;
  Eigen::MatrixXd qp_H = Eigen::MatrixXd::Identity(jacobian.cols(), jacobian.cols());
  Eigen::MatrixXd qp_g = Eigen::VectorXd::Zero(jacobian.cols());
  Eigen::VectorXd qp_lb = Eigen::VectorXd::Constant(jacobian.cols(), -1);
  Eigen::VectorXd qp_ub = Eigen::VectorXd::Constant(jacobian.cols(),  1);
  qp_lb.head(jacobian.cols() - chain.getNrOfJoints()) *= thre_root_vel_;
  qp_lb.tail(chain.getNrOfJoints()) *= thre_joint_vel_;
  qp_ub.head(jacobian.cols() - chain.getNrOfJoints()) *= thre_root_vel_;
  qp_ub.tail(chain.getNrOfJoints()) *= thre_joint_vel_;

  Eigen::MatrixXd qp_A = Eigen::MatrixXd::Zero(qp_solver->getNC(), jacobian.cols());
  Eigen::VectorXd qp_lA = Eigen::VectorXd::Constant(qp_solver->getNC(), -INFTY);
  Eigen::VectorXd qp_uA = Eigen::VectorXd::Constant(qp_solver->getNC(), INFTY);

  Eigen::MatrixXd W_ik_constraint = Eigen::MatrixXd::Identity(jacobian.rows(), jacobian.rows()) * w_ik_constraint_;
  Eigen::MatrixXd W_vel = Eigen::MatrixXd::Identity(jacobian.cols(), jacobian.cols()) * w_joint_vel_; /* for joint motion constraint */
  W_vel.block(0, 0, jacobian.cols() - chain.getNrOfJoints(), jacobian.cols() - chain.getNrOfJoints()) *= (w_root_vel_ / w_joint_vel_); /* for root motion constraint */

  if(debug) std::cout << "the Weight Matrix of vel: \n" << W_vel << std::endl;
  if(debug) std::cout << "the Weight Matrix of ik constraint: \n" << W_ik_constraint << std::endl;
  if(debug) std::cout << "the qp lb is: \n" << qp_lb << std::endl;
  if(debug) std::cout << "the qp ub is: \n" << qp_ub << std::endl;

  int n_wsr = 100;
  Options qp_options;
  qp_options.enableEqualities = BT_TRUE;
  qp_options.printLevel = PL_LOW;
  qp_solver->setOptions(qp_options);

  /* inverse kinematics loop */
  for(int l = 0; l < differential_motion_count_; l++)
    {
      /*
        1.
         update the current kinematics by forward kinematics, along with the modelling
      */

      KDL::Frame ee_frame;
      KDL::ChainFkSolverPos_recursive fk_solver(chain);
      KDL::JntArray joint_positions(chain.getNrOfJoints());
      joint_positions.data = real_joint_vector;
      fk_solver.JntToCart(joint_positions, ee_frame);

      /* update the modelling */
      KDL::Rotation root_att;
      tf::quaternionTFToKDL(target_root_pose.getRotation(), root_att);
      setCogDesireOrientation(root_att);
      forwardKinematics(target_joint_vector);
      if(!stabilityMarginCheck()) ROS_ERROR("[ik] update modelling, bad stability margin ");
      if(!modelling()) ROS_ERROR("[ik] update modelling, bad stability from force");

      /*
         2.
         calcualte the cartesian error and check the congergence
      */
      tf::Transform ee_tf;
      tf::transformKDLToTF(ee_frame, ee_tf);
      if(debug)
        ROS_WARN("ee x: %f,  y: %f, yaw: %f, target x: %f, y: %f, yaw: %f", ee_tf.getOrigin().x(), ee_tf.getOrigin().y(), getYaw(ee_tf.getRotation()), target_ee_pose.getOrigin().x(), target_ee_pose.getOrigin().y(), getYaw(target_ee_pose.getRotation()));
      tf::Transform ee_err_tf =  (target_root_pose * ee_tf).inverse() * target_ee_pose;
      double ee_pos_err = ee_err_tf.getOrigin().length();
      double ee_rot_err = fabs(ee_err_tf.getRotation().getAngleShortestPath());
      if(debug)
        ROS_INFO("ee err pos, rot(angle): %f[m], %f[rad], err pos vec from ee tf : [%f, %f, %f]", ee_pos_err, ee_rot_err, ee_err_tf.getOrigin().x(), ee_err_tf.getOrigin().y(), ee_err_tf.getOrigin().z());

      if(ee_pos_err < ee_pos_err_thre_ ) /* position convergence */
        {
          if(!orientation) return true;
          else
            if(ee_rot_err < ee_rot_err_thre_) return true;
        }

      /* transform the cartesian err to the root link frame */
      if(ee_pos_err > ee_pos_err_max_) ee_pos_err = ee_pos_err_max_;
      if(ee_rot_err > ee_rot_err_max_) ee_rot_err = ee_rot_err_max_;

      tf::Vector3 ee_target_pos_err_root_link = ee_tf.getBasis() * ee_err_tf.getOrigin().normalize() * ee_pos_err;
      tf::Vector3 ee_target_rot_err_root_link = ee_tf.getBasis() * ee_err_tf.getRotation().getAxis() * ee_rot_err;

      Eigen::VectorXd delta_cartesian = Eigen::VectorXd::Zero(6);
      Eigen::Vector3d temp_vec;
      tf::vectorTFToEigen(ee_target_pos_err_root_link, temp_vec);
      delta_cartesian.head(3) = temp_vec;
      tf::vectorTFToEigen(ee_target_rot_err_root_link, temp_vec);
      delta_cartesian.tail(3) = temp_vec;

      if(multilink_type_ == MULTILINK_TYPE_SE2)
        {
          if(!orientation) delta_cartesian.conservativeResize(2);
          else
            {
              delta_cartesian(2) = delta_cartesian(5);
              delta_cartesian.conservativeResize(3);
            }
        }
      else
        {
          if(!orientation) delta_cartesian.conservativeResize(3);
        }

      if(debug) std::cout << "delta cartesian: \n" << delta_cartesian << std::endl;

      /*
         3.
         calculate joint jacobian
      */
      if(!calcJointJacobian(chain, real_joint_vector, jacobian, orientation, full_body, debug))
        return false;

      /*
        4.
        calcualte the delta joint angle vector
       */
      Eigen::VectorXd delta_joint_vector = Eigen::VectorXd::Zero(jacobian.cols());
      if(ik_algorithm == IK_INVERSE_JACOBIAN)
        {
          //delta_joint_vector = pseudoinverse(jacobian) * delta_cartesian; // simple pseudo inverse
          delta_joint_vector = jacobian.transpose() *
            (jacobian * jacobian.transpose() +  0.001 * Eigen::MatrixXd::Identity(jacobian.rows(), jacobian.rows())).inverse()  * delta_cartesian;
        }
      else if(ik_algorithm == IK_QP)
        {
          /*
            QP:
            - cost function : (J dq - dx)^T W1 (J dq - dx) + dq^T W2 dq  :  The least square error term for equality conditions(e.g. IK)  + vel (i.e. joint, root) penality
            - dq^T (J^T W1 J + W2) dq - 2 dx^T W1 J dq + hoge
            - since:-2 dx^T W1 J = -2 (J^T W1^T dx)^T = -2 (J^T W1 dx)^T
          */
          qp_H = jacobian.transpose() * W_ik_constraint * jacobian + W_vel;
          qp_g = - delta_cartesian.transpose()  * W_ik_constraint * jacobian;
          /*
          if(debug)
            {
              std::cout << "qp H: \n" << qp_H << std::endl;
              std::cout << "qp g: \n" << qp_g << std::endl;
            }
          */

          /*************************************************************************************
           1. joint velocity constraint
          ****************************************************************************************/
          for(int i = 0; i < chain.getNrOfJoints(); i ++)
            {
              qp_lb(jacobian.cols() - chain.getNrOfJoints() + i)= -thre_joint_vel_;
              qp_ub(jacobian.cols() - chain.getNrOfJoints() + i)=  thre_joint_vel_;

              /* min */
              if(real_joint_vector(i) - joint_angle_min_ < joint_vel_constraint_range_)
                qp_lb(jacobian.cols() - chain.getNrOfJoints() + i) *=
                  (real_joint_vector(i) - joint_angle_min_ - joint_vel_forbidden_range_) / (joint_vel_constraint_range_ - joint_vel_forbidden_range_);
              /* max */
              if(joint_angle_max_ - real_joint_vector(i)  < joint_vel_constraint_range_)
                qp_ub(jacobian.cols() - chain.getNrOfJoints() + i) *=
                  (joint_angle_max_ - real_joint_vector(i) - joint_vel_forbidden_range_) / (joint_vel_constraint_range_ - joint_vel_forbidden_range_);
            }

          /*************************************************************************************
           2. stability margin
          ****************************************************************************************/
          Eigen::MatrixXd margin_att_jacobian = Eigen::MatrixXd::Zero(1, 2); /* roll pitch */
          Eigen::MatrixXd margin_joint_jacobian = Eigen::MatrixXd::Zero(1, chain.getNrOfJoints()); /* joint */
          double stability_margin = getStabilityMargin();
          /* fill the fixed size bounder lA */
          qp_lA(0) =  stability_margin_thre_ - stability_margin;
          if(debug) std::cout << "stability margin: " << stability_margin << std::endl;

          /*************************************************************************************
           2. singularity
          ****************************************************************************************/
          Eigen::MatrixXd det_att_jacobian = Eigen::MatrixXd::Zero(1, 2); /* roll pitch */
          Eigen::MatrixXd det_joint_jacobian = Eigen::MatrixXd::Zero(1, chain.getNrOfJoints()); /* joint */
          double p_det = getPdeterminant();
          /* fill the fixed size bounder lA */
          qp_lA(1) =  p_det_thre_ - p_det;
          if(debug) std::cout << "singularity: " << p_det << std::endl;

          /*************************************************************************************
           4. optimal hovering thrust constraint (including singularity check)
           f_min < F + delta_f < f_max =>   delta_f =  (f(q + d_q) - f(q)) / d_q * delta_q
           TODO: use virutal state (beta) to maximize the flight stability
          ****************************************************************************************/
          Eigen::MatrixXd f_att_jacobian = Eigen::MatrixXd::Zero(rotor_num_, 2); /* roll pitch */
          Eigen::MatrixXd f_joint_jacobian = Eigen::MatrixXd::Zero(rotor_num_, chain.getNrOfJoints()); /* joint */
          double delta_angle = 0.001; // [rad]
          Eigen::VectorXd hovering_f =  getOptimalHoveringThrust();
          if(debug) std::cout << "hovering f: " << hovering_f.transpose() << std::endl;
          /* fill the fixed size bounder lA/uA */
          qp_lA.segment(2, rotor_num_) = Eigen::VectorXd::Constant(rotor_num_, f_min_) - hovering_f;
          qp_uA.segment(2, rotor_num_) = Eigen::VectorXd::Constant(rotor_num_, f_max_) - hovering_f;

          /* joint */
          for(int index = 0; index < chain.getNrOfJoints(); index++)
            {
              sensor_msgs::JointState joint_state = target_joint_vector;
              joint_state.position.at(joints_map.at(index)) += delta_angle;
              forwardKinematics(joint_state);
              if(!stabilityMarginCheck())
                ROS_ERROR("[optimal hovering thrust constraint] delta joint, bad margin ");
              if(!modelling())
                ROS_ERROR("[optimal hovering thrust constraint] delta joint, bad stability ");

              /* stability margin */
              margin_joint_jacobian(0, index) = (getStabilityMargin() - stability_margin) /delta_angle;
              /* singularity */
              det_joint_jacobian(0, index) = (getPdeterminant() - p_det) /delta_angle;
              /* hovering thrust */
              f_joint_jacobian.block(0, index, rotor_num_, 1) = (getOptimalHoveringThrust() - hovering_f) / delta_angle;

            }
          /* fill the constraint matrix A */
          qp_A.block(0, jacobian.cols() - chain.getNrOfJoints(), 1, chain.getNrOfJoints()) = margin_joint_jacobian;
          qp_A.block(1, jacobian.cols() - chain.getNrOfJoints(), 1, chain.getNrOfJoints()) = det_joint_jacobian;
          qp_A.block(2, jacobian.cols() - chain.getNrOfJoints(), rotor_num_, chain.getNrOfJoints()) = f_joint_jacobian;

#if 0
          /* debug */
          std::cout << "delta angle 0.01, f_d_joint jacobian \n" << f_joint_jacobian << std::endl;
          std::cout << "delta angle 0.01, margin_d_joint jacobian \n" << margin_joint_jacobian << std::endl;
          std::cout << "delta angle 0.01, det_d_joint jacobian \n" << det_joint_jacobian << std::endl;

          delta_angle = -0.001; // [rad]
          for(int index = 0; index < chain.getNrOfJoints(); index++)
            {
              sensor_msgs::JointState joint_state = target_joint_vector;
              joint_state.position.at(joints_map.at(index)) += delta_angle;
              forwardKinematics(joint_state);

              if(!stabilityMarginCheck())
                ROS_ERROR("[optimal hovering thrust constraint] delta joint, bad margin ");
              if(!modelling())
                ROS_ERROR("[optimal hovering thrust constraint] delta joint, bad stability ");

              /* stability margin */
              margin_joint_jacobian(0, index) = (getStabilityMargin() - stability_margin) / delta_angle;
              /* singularity */
              det_joint_jacobian(0, index) = (getPdeterminant() - p_det) /delta_angle;
              /* hovering thrust */
              f_joint_jacobian.block(0, index, rotor_num_, 1) = (getOptimalHoveringThrust() - hovering_f) / delta_angle;
            }
          //std::cout << "delta angle -0.01, f_d_joint jacobian \n" << f_joint_jacobian << std::endl;
          std::cout << "delta angle -0.01, margin_d_joint jacobian \n" << margin_joint_jacobian << std::endl;
          std::cout << "delta angle -0.01, det_d_joint jacobian \n" << det_joint_jacobian << std::endl;
#endif

          /* att */
          if(multilink_type_ == MULTILINK_TYPE_SE3 && full_body)
            {
              /* reset joint vector */
              /* 2.2.1 roll */
              setCogDesireOrientation(root_att * KDL::Rotation::RPY(delta_angle, 0, 0));
              forwardKinematics(target_joint_vector);
              if(!stabilityMarginCheck())
                ROS_ERROR("[optimal hovering thrust constraint] delta joint, bad margin ");
              if(!modelling())
                ROS_ERROR("[optimal hovering thrust constraint] delta roll, bad stability ");

              /* stability margin */
              margin_att_jacobian(0, 0) = (getStabilityMargin() - stability_margin) / delta_angle;
              /* singularity */
              det_att_jacobian(0, 0) = (getPdeterminant() - p_det) /delta_angle;
              /* hovering thrust */
              f_att_jacobian.block(0, 0, rotor_num_, 1) = (getOptimalHoveringThrust() - hovering_f) / delta_angle;

              /* 2.2.2 pitch */
              setCogDesireOrientation(root_att * KDL::Rotation::RPY(0, delta_angle, 0));
              forwardKinematics(target_joint_vector);
              if(!stabilityMarginCheck())
                ROS_ERROR("[optimal hovering thrust constraint] delta roll, bad stability ");
              if(!modelling())
                ROS_ERROR("[optimal hovering thrust constraint] delta roll, bad stability ");

              /* stability margin */
              margin_att_jacobian(0, 1) = (getStabilityMargin() - stability_margin) / delta_angle;
              /* singularity */
              det_att_jacobian(0, 1) = (getPdeterminant() - p_det) /delta_angle;
              /* hovering thrust */
              f_att_jacobian.block(0, 1, rotor_num_, 1) = (getOptimalHoveringThrust() - hovering_f) / delta_angle;

              /* fill the constraint matrix A */
              qp_A.block(0, 3, 0, 2) = margin_att_jacobian;
              qp_A.block(1, 3, 0, 2) = det_att_jacobian;
              qp_A.block(2, 3, rotor_num_, 2) = f_att_jacobian;

#if 0
              /* debug */
              std::cout << "delta angle 0.01, f_d_att jacobian \n" << f_att_jacobian << std::endl;
              std::cout << "delta angle 0.01, margin_d_att jacobian \n" << margin_att_jacobian << std::endl;
              std::cout << "delta angle 0.01, det_d_att jacobian \n" << det_att_jacobian << std::endl;
              delta_angle = -0.01; // [rad]
              setCogDesireOrientation(root_att * KDL::Rotation::RPY(delta_angle, 0, 0));
              forwardKinematics(target_joint_vector);
              if(!modelling())
                ROS_ERROR("[optimal hovering thrust constraint] delta roll, bad stability ");
              if(!modelling())
                ROS_ERROR("[optimal hovering thrust constraint] delta roll, bad stability ");

              margin_att_jacobian(0, 0) = (getStabilityMargin() - stability_margin) / delta_angle;
              det_att_jacobian(0, 0) = (getPdeterminant() - p_det) /delta_angle;
              f_att_jacobian.block(0, 0, rotor_num_, 1) = (getOptimalHoveringThrust() - hovering_f) / delta_angle;

              setCogDesireOrientation(root_att * KDL::Rotation::RPY(0, delta_angle, 0));
              forwardKinematics(target_joint_vector);
              if(!modelling())
                ROS_ERROR("[optimal hovering thrust constraint] delta roll, bad stability ");
              if(!modelling())
                ROS_ERROR("[optimal hovering thrust constraint] delta roll, bad stability ");
              margin_att_jacobian(0, 1) = (getStabilityMargin() - stability_margin) / delta_angle;
              det_att_jacobian(0, 1) = (getPdeterminant() - p_det) /delta_angle;
              f_att_jacobian.block(0, 1, rotor_num_, 1) = (getOptimalHoveringThrust() - hovering_f) / delta_angle;
              //std::cout << "delta angle -0.01, f_d_att jacobian \n" << f_att_jacobian << std::endl;
              //std::cout << "delta angle 0.01, margin_d_att jacobian \n" << margin_att_jacobian << std::endl;
              //std::cout << "delta angle 0.01, det_d_att jacobian \n" << det_att_jacobian << std::endl;
#endif
            }

          //std::cout << "qp A \n" << qp_A << std::endl; //debug
          std::cout << "qp lA \n" << qp_lA.transpose() << std::endl; //debug
          std::cout << "qp uA \n" << qp_uA.transpose() << std::endl; //debug

          int solver_result;
          n_wsr = 100; /* this value have to be updated every time, otherwise it will decrease every loop */
          Eigen::MatrixXd qp_At = qp_A.transpose();
          if(qp_init_flag)
            { /* first time */
              qp_init_flag = false;

              solver_result = qp_solver->init(qp_H.data(), qp_g.data(),
                                              qp_At.data(), /* CAUTION: eigen is col-major order array in default! */
                                              qp_lb.data(), qp_ub.data(),
                                              qp_lA.data(), qp_uA.data(), n_wsr);
            }
          else
            {
              solver_result = qp_solver->hotstart(qp_H.data(), qp_g.data(),
                                                  qp_At.data(),
                                                  qp_lb.data(), qp_ub.data(),
                                                  qp_lA.data(), qp_uA.data(), n_wsr);
            }
          if(solver_result != 0)
            {
              ROS_ERROR("can not solve QP the solver_result is %d", solver_result);
              return false;
            }

          qp_solver->getPrimalSolution(delta_joint_vector.data());
        }
      else
        {
          ROS_ERROR("invalid ik algorithm %d", ik_algorithm);
          return false;
        }

      if(debug) std::cout << "delta joint vector: \n" << delta_joint_vector << std::endl;

      std::cout << "delta f \n" << (qp_A *  delta_joint_vector).transpose() << std::endl; //debug

      /*
         5.
         update kinematics, root link transformation
      */
      if(full_body)
        {
          if(multilink_type_ == MULTILINK_TYPE_SE2)
            {
              assert(delta_joint_vector.size() == chain.getNrOfJoints() + 3);

              /* root link incremental transformation: */
              target_root_pose *= tf::Transform(tf::createQuaternionFromYaw(delta_joint_vector(2)),
                                                tf::Vector3(delta_joint_vector(0), delta_joint_vector(1), 0));
              /* joint */
              real_joint_vector += delta_joint_vector.tail(delta_joint_vector.size() - 3);
            }
          else //  multilink_type_ == MULTILINK_TYPE_SE3
            {
              assert(delta_joint_vector.size() == chain.getNrOfJoints() + 6);

              /* root link incremental transformation: */
              tf::Vector3 delta_pos_from_root_link;
              tf::vectorEigenToTF(delta_joint_vector.head(3), delta_pos_from_root_link);
              tf::Vector3 delta_rot_from_root_link;
              tf::vectorEigenToTF(delta_joint_vector.segment(3, 3), delta_rot_from_root_link);

              target_root_pose *= tf::Transform(tf::Quaternion(delta_rot_from_root_link, delta_rot_from_root_link.length()), delta_pos_from_root_link);

              /* joint */
              real_joint_vector += delta_joint_vector.tail(delta_joint_vector.size() - 6);
            }

        }
      else
        real_joint_vector += delta_joint_vector;

      /* clip */
      for(int index = 0; index < real_joint_vector.size(); index++)
        {
          if(real_joint_vector[index] < joint_angle_min_)
            {
              ROS_ERROR("joint%d should be not exceed: %f", index, real_joint_vector[index]);
              real_joint_vector[index] = joint_angle_min_;
            }
          if(real_joint_vector[index] > joint_angle_max_)
            {
              ROS_ERROR("joint%d should be not exceed: %f", index, real_joint_vector[index]);
              real_joint_vector[index] = joint_angle_max_;
            }
        }

      if(debug)  std::cout << "real joint vector: \n" << real_joint_vector << std::endl;

      /* udpate the joint angles */
      for(auto itr = joints_map.begin(); itr != joints_map.end(); ++itr)
        target_joint_vector.position.at(*itr) = real_joint_vector(std::distance(joints_map.begin(), itr));
    }

  ROS_WARN("can not resolve inverse kinematics with the max loop count %d", differential_motion_count_);
  return false;
}

bool TransformController::calcJointJacobian(const KDL::Chain& chain, const Eigen::VectorXd& angle_vector, Eigen::MatrixXd& jacobian,  bool orientation, bool full_body, bool debug)
{
  /* fill the joint state */
  KDL::JntArray joint_positions(chain.getNrOfJoints());
  joint_positions.data = angle_vector;

  /* calculate the jacobian */
  KDL::ChainJntToJacSolver jac_solver_(chain);
  KDL::Jacobian jac(chain.getNrOfJoints());
  if(jac_solver_.JntToJac(joint_positions, jac) == KDL::SolverI::E_NOERROR)
    {
      /* consider root is attached with a 6Dof free joint */
      if(full_body)
        {
          /* get ee position from root */
          KDL::Frame ee_frame;
          KDL::ChainFkSolverPos_recursive fk_solver(chain);
          fk_solver.JntToCart(joint_positions, ee_frame);
          /* debug */
          ROS_INFO("ee pose: [%f %f %f]", ee_frame.p.x(), ee_frame.p.y(), ee_frame.p.z());

          /* fill the 6x6 matrix */
          jacobian = Eigen::MatrixXd::Zero(6, 6 + jac.data.cols());
          /* root link */
          jacobian.block(0, 0, 6, 6) = Eigen::MatrixXd::Identity(6, 6);
          jacobian.block(0, 3, 3, 1) = (Eigen::Vector3d(1, 0, 0)).cross(Eigen::Vector3d(ee_frame.p.data));
          jacobian.block(0, 4, 3, 1) = (Eigen::Vector3d(0, 1, 0)).cross(Eigen::Vector3d(ee_frame.p.data));
          jacobian.block(0, 5, 3, 1) = (Eigen::Vector3d(0, 0, 1)).cross(Eigen::Vector3d(ee_frame.p.data));

          /* joint part */
          jacobian.block(0, 6, 6, jac.data.cols()) = jac.data;
        }
      else
        jacobian = jac.data;

      /* debug */
      //std::cout << "raw jacobian: \n" << jacobian << std::endl;

      /* resize */
      if(multilink_type_ == MULTILINK_TYPE_SE2)
        {
          /* row process */
          if(!orientation)
            {
              jacobian.conservativeResize(2, jacobian.cols());
            }
          else
            {
              jacobian.row(2) = jacobian.row(5);
              jacobian.conservativeResize(3, jacobian.cols());
            }

          /* col process */
          if(full_body)
            {
              jacobian.block(0, 2, jacobian.rows(),  chain.getNrOfJoints() + 1)
                = jacobian.block(0, 5, jacobian.rows(),  chain.getNrOfJoints() + 1);
              jacobian.conservativeResize(jacobian.rows(), chain.getNrOfJoints() + 3);
            }
        }
      else
        {
          /* row process */
          if(!orientation) jacobian.conservativeResize(3, jacobian.cols()); //only position
        }

      if(debug) std::cout << "jacobian: \n" << jacobian << std::endl;
      return true;
    }
  return false; //else
}

bool TransformController::addExtraModuleCallback(hydrus::AddExtraModule::Request  &req,
                                                 hydrus::AddExtraModule::Response &res)
{
  return addExtraModule(req.action, req.module_name, req.parent_link_name, req.transform, req.inertia);
}

bool TransformController::addExtraModule(int action, std::string module_name, std::string parent_link_name, geometry_msgs::Transform transform, geometry_msgs::Inertia inertia)
{
  switch(action)
    {
    case hydrus::AddExtraModule::Request::ADD:
      {
        std::map<std::string, KDL::Segment>::iterator it = extra_module_map_.find(module_name);
        if(it == extra_module_map_.end())
          {
            if(inertia_map_.find(parent_link_name) == inertia_map_.end())
              {
                ROS_WARN("[extra module]: fail to add new extra module %s, becuase it's parent link (%s) is invalid", module_name.c_str(), parent_link_name.c_str());
                return false;
              }


            if(fabs(1 - tf::Quaternion(transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w).length2()) > 1e-6)
              {
                ROS_WARN("[extra module]: fail to add new extra module %s, becuase the orientation is invalid", module_name.c_str());
                return false;
              }

            if(inertia.m <= 0)
              {
                ROS_WARN("[extra module]: fail to add new extra module %s, becuase the mass %f is invalid", module_name.c_str(), inertia.m);
                return false;
              }

            KDL::Frame f;
            tf::transformMsgToKDL(transform, f);
            KDL::RigidBodyInertia rigid_body_inertia(inertia.m, KDL::Vector(inertia.com.x, inertia.com.y, inertia.com.z),
                                                     KDL::RotationalInertia(inertia.ixx, inertia.iyy,
                                                                            inertia.izz, inertia.ixy,
                                                                            inertia.ixz, inertia.iyz));
            KDL::Segment extra_module(parent_link_name, KDL::Joint(KDL::Joint::None), f, rigid_body_inertia);
            extra_module_map_.insert(std::make_pair(module_name, extra_module));
            ROS_INFO("[extra module]: succeed to add new extra module %s", module_name.c_str());
            return true;
          }
        else
          {
            ROS_WARN("[extra module]: fail to add new extra module %s, becuase it already exists", module_name.c_str());
            return false;
          }
        break;
      }
    case hydrus::AddExtraModule::Request::REMOVE:
      {
        std::map<std::string, KDL::Segment>::iterator it = extra_module_map_.find(module_name);
        if(it == extra_module_map_.end())
          {
            ROS_WARN("[extra module]: fail to remove the extra module %s, becuase it does not exists", module_name.c_str());
            return false;
          }
        else
          {
            extra_module_map_.erase(module_name);
            ROS_INFO("[extra module]: suscced to remove the extra module %s", module_name.c_str());
            return true;
          }
        break;
      }
    default:
      {
        ROS_WARN("[extra module]: wrong action %d", action);
        return false;
        break;
      }
    }
  ROS_ERROR("[extra module]: should not reach here ");
  return false;
}

void TransformController::lqi()
{
  if(!kinematics_flag_) return;

  /* check the thre check */
  if(debug_verbose_) ROS_WARN(" start dist thre check");
  if(!stabilityMarginCheck()) //[m]
    {
      ROS_ERROR("LQI: invalid pose, cannot pass the distance thresh check");
      return;
    }
  if(debug_verbose_) ROS_WARN(" finish dist thre check");

  /* check the propeller overlap */
  if(debug_verbose_) ROS_WARN(" start overlap check");
  if(!overlapCheck()) //[m]
    {
      ROS_ERROR("LQI: invalid pose, some propellers overlap");
      return;
    }
  if(debug_verbose_) ROS_WARN(" finish dist thre check");

  /* modelling the multilink based on the inertia assumption */
  if(debug_verbose_) ROS_WARN(" start modelling");
  if(!modelling())
      ROS_ERROR("LQI: invalid pose, can not be four axis stable, switch to three axis stable mode");

  if(debug_verbose_) ROS_WARN(" finish modelling");

  if(debug_verbose_) ROS_WARN(" start ARE calc");
  if(!hamiltonMatrixSolver(lqi_mode_))
    {
      ROS_ERROR("LQI: can not solve hamilton matrix");
      return;
    }
  if(debug_verbose_) ROS_WARN(" finish ARE calc");

  param2controller();
  if(debug_verbose_) ROS_WARN(" finish param2controller");
}

bool TransformController::stabilityMarginCheck(bool verbose)
{
  double average_x = 0, average_y = 0;

  std::vector<Eigen::Vector3d> rotors_origin_from_cog = getRotorsOriginFromCog();

  /* calcuate the average */
  for(int i = 0; i < rotor_num_; i++)
    {
      average_x += rotors_origin_from_cog[i](0);
      average_y += rotors_origin_from_cog[i](1);
      if(verbose)
        ROS_INFO("rotor%d x: %f, y: %f", i + 1, rotors_origin_from_cog[i](0), rotors_origin_from_cog[i](1));
    }
  average_x /= rotor_num_;
  average_y /= rotor_num_;

  double s_xy =0, s_xx = 0, s_yy =0;
  for(int i = 0; i < rotor_num_; i++)
    {
      s_xy += ((rotors_origin_from_cog[i](0) - average_x) * (rotors_origin_from_cog[i](1) - average_y));
      s_xx += ((rotors_origin_from_cog[i](0) - average_x) * (rotors_origin_from_cog[i](0) - average_x));
      s_yy += ((rotors_origin_from_cog[i](1) - average_y) * (rotors_origin_from_cog[i](1) - average_y));
    }

  Eigen::Matrix2d S;
  S << s_xx / rotor_num_, s_xy / rotor_num_, s_xy / rotor_num_, s_yy / rotor_num_;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(S);
  double link_len = (rotors_origin_from_cog[1] - rotors_origin_from_cog[0]).norm();

  assert(link_length_ > 0);
  stability_margin_ = sqrt(es.eigenvalues()[0]) / link_length_;
  if(verbose) ROS_INFO("stability_margin: %f", stability_margin_);
  if( stability_margin_ < stability_margin_thre_ ) return false;
  return true;

#if 0 // correlation coefficient
  double correlation_coefficient = fabs(s_xy / sqrt(s_xx * s_yy));
  //ROS_INFO("correlation_coefficient: %f", correlation_coefficient);

  if(correlation_coefficient > correlation_thre_ ) return false;
  return true;
#endif
}

bool  TransformController::modelling(bool verbose)
{
  std::vector<Eigen::Vector3d> rotors_origin_from_cog = getRotorsOriginFromCog();
  Eigen::Matrix3d links_inertia = getInertia();

  Eigen::VectorXd g(4);
  g << 0, 0, 9.8, 0;
  Eigen::VectorXd p_x(rotor_num_), p_y(rotor_num_), p_c(rotor_num_), p_m(rotor_num_);

  for(int i = 0; i < rotor_num_; i++)
    {
      p_y(i) =  rotors_origin_from_cog[i](1);
      p_x(i) = -rotors_origin_from_cog[i](0);
      p_c(i) = rotor_direction_.at(i + 1) * m_f_rate_ ;
      p_m(i) = 1 / getMass();
      if(kinematic_verbose_ || verbose)
        std::cout << "link" << i + 1 <<"origin :\n" << rotors_origin_from_cog[i] << std::endl;
    }

  Eigen::MatrixXd P_att = Eigen::MatrixXd::Zero(3,rotor_num_);
  P_att.row(0) = p_y;
  P_att.row(1) = p_x;
  P_att.row(2) = p_c;

  Eigen::MatrixXd P_att_tmp = P_att;
  P_att = links_inertia.inverse() * P_att_tmp;
  if(control_verbose_)
    std::cout << "links_inertia inverse:"  << std::endl << links_inertia.inverse() << std::endl;

  // P_.row(0) = p_y / links_principal_inertia(0,0);
  // P_.row(1) = p_x / links_principal_inertia(1,1);
  // P_.row(3) = p_c / links_principal_inertia(2,2);

  /* roll, pitch, alt, yaw */
  P_.row(0) = P_att.row(0);
  P_.row(1) = P_att.row(1);
  P_.row(2) = p_m;
  P_.row(3) = P_att.row(2);

  if(control_verbose_ || verbose)
    std::cout << "P_:"  << std::endl << P_ << std::endl;

  ros::Time start_time = ros::Time::now();
  /* lagrange mothod */
  // issue: min x_t * x; constraint: g = P_ * x  (stable point)
  //lamda: [4:0]
  // x = P_t * lamba
  // (P_  * P_t) * lamda = g
  // x = P_t * (P_ * P_t).inv * g
  Eigen::FullPivLU<Eigen::MatrixXd> solver((P_ * P_.transpose()));
  Eigen::VectorXd lamda;
  lamda = solver.solve(g);
  optimal_hovering_f_ = P_.transpose() * lamda;

  p_det_ = (P_ * P_.transpose()).determinant();
  if(control_verbose_)
    std::cout << "P det:"  << std::endl << p_det_ << std::endl;

  if(control_verbose_)
    ROS_INFO("P solver is: %f\n", ros::Time::now().toSec() - start_time.toSec());

  if(optimal_hovering_f_.maxCoeff() > f_max_ || optimal_hovering_f_.minCoeff() < f_min_ || p_det_ < p_det_thre_ || only_three_axis_mode_)
    {
      lqi_mode_ = LQI_THREE_AXIS_MODE;

      //no yaw constraint
      Eigen::MatrixXd P_dash = Eigen::MatrixXd::Zero(3, rotor_num_);
      P_dash.row(0) = P_.row(0);
      P_dash.row(1) = P_.row(1);
      P_dash.row(2) = P_.row(2);
      Eigen::VectorXd g3(3);
      g3 << 0, 0, 9.8;
      Eigen::FullPivLU<Eigen::MatrixXd> solver((P_dash * P_dash.transpose()));
      Eigen::VectorXd lamda;
      lamda = solver.solve(g3);
      optimal_hovering_f_ = P_dash.transpose() * lamda;
      if(control_verbose_)
        std::cout << "three axis mode: optimal_hovering_f_:"  << std::endl << optimal_hovering_f_ << std::endl;

      /* calculate the P_orig(without inverse inertia) peusdo inverse */
      P_dash.row(0) = p_y;
      P_dash.row(1) = p_x;
      P_dash.row(2) = p_m;
      Eigen::MatrixXd P_dash_pseudo_inverse = P_dash.transpose() * (P_dash * P_dash.transpose()).inverse();
      P_orig_pseudo_inverse_ = Eigen::MatrixXd::Zero(rotor_num_, 4);
      P_orig_pseudo_inverse_.block(0, 0, rotor_num_, 3) = P_dash_pseudo_inverse;
      if(control_verbose_)
        std::cout << "P orig_pseudo inverse for three axis mode:"  << std::endl << P_orig_pseudo_inverse_ << std::endl;

      /* if we do the 4dof underactuated control */
      if(!only_three_axis_mode_) return false;

      p_det_ = (P_dash * P_dash.transpose()).determinant();

      /* if we only do the 3dof control, we still need to check the steady state validation */
      if(optimal_hovering_f_.maxCoeff() > f_max_ || optimal_hovering_f_.minCoeff() < f_min_
         || p_det_ < p_det_thre_)
        return false;

      return true;
    }

  /* calculate the P_orig(without inverse inertia) peusdo inverse */
  Eigen::MatrixXd P_dash = Eigen::MatrixXd::Zero(4,rotor_num_);
  P_dash.row(0) = p_y;
  P_dash.row(1) = p_x;
  P_dash.row(2) = p_m;
  P_dash.row(3) = p_c;

  P_orig_pseudo_inverse_ = P_dash.transpose() * (P_dash * P_dash.transpose()).inverse();
  if(control_verbose_)
    std::cout << "P orig_pseudo inverse for four axis mode:" << std::endl << P_orig_pseudo_inverse_ << std::endl;

  if(control_verbose_ || verbose)
    std::cout << "four axis mode optimal_hovering_f_:"  << std::endl << optimal_hovering_f_ << std::endl;

  lqi_mode_ = LQI_FOUR_AXIS_MODE;

  return true;
}

bool TransformController::hamiltonMatrixSolver(uint8_t lqi_mode)
{
  /* for the R which is  diagonal matrix. should be changed to rotor_num */

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(lqi_mode_ * 3, lqi_mode_ * 3);
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(lqi_mode_ * 3, rotor_num_);
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(lqi_mode_, lqi_mode_ * 3);

  for(int i = 0; i < lqi_mode_; i++)
    {
      A(2 * i, 2 * i + 1) = 1;
      B.row(2 * i + 1) = P_.row(i);
      C(i, 2 * i) = 1;
    }
  A.block(lqi_mode_ * 2, 0, lqi_mode_, lqi_mode_ * 3) = -C;

  if(control_verbose_) std::cout << "B:"  << std::endl << B << std::endl;

  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(lqi_mode_ * 3, lqi_mode_ * 3);
  if(lqi_mode_ == LQI_THREE_AXIS_MODE)
    {
      Eigen::MatrixXd Q_tmp = q_diagonal_.asDiagonal();
      Q.block(0, 0, 6, 6) = Q_tmp.block(0, 0, 6, 6);
      Q.block(6, 6, 3, 3) = Q_tmp.block(8, 8, 3, 3);
    }
  if(lqi_mode_ == LQI_FOUR_AXIS_MODE) Q = q_diagonal_.asDiagonal();

  Eigen::MatrixXd R_inv  = Eigen::MatrixXd::Zero(rotor_num_, rotor_num_);
  for(int i = 0; i < rotor_num_; i ++)
    R_inv(i,i) = 1/r_[i];

  // B12_aug_ = Eigen::MatrixXd::Zero(12, rotor_num_);
  // for(int j = 0; j < 8; j++) B12_aug_.row(j) = B_.row(j);


  Eigen::MatrixXcd H = Eigen::MatrixXcd::Zero(lqi_mode_ * 6, lqi_mode_ * 6);
  H.block(0,0, lqi_mode_ * 3, lqi_mode_ * 3) = A.cast<std::complex<double> >();
  H.block(lqi_mode_ * 3, 0, lqi_mode_ * 3, lqi_mode_ * 3) = -(Q.cast<std::complex<double> >());
  H.block(0, lqi_mode_ * 3, lqi_mode_ * 3, lqi_mode_ * 3) = - (B * R_inv * B.transpose()).cast<std::complex<double> >();
  H.block(lqi_mode_ * 3, lqi_mode_ * 3, lqi_mode_ * 3, lqi_mode_ * 3) = - (A.transpose()).cast<std::complex<double> >();

  //std::cout << " H  is:" << std::endl << H << std::endl;
  if(debug_verbose_) ROS_INFO("  start H eigen compute");
  //eigen solving
  ros::Time start_time = ros::Time::now();
  Eigen::ComplexEigenSolver<Eigen::MatrixXcd> ces;
  ces.compute(H);
  if(debug_verbose_) ROS_INFO("  finish H eigen compute");

  if(control_verbose_)
    ROS_INFO("h eigen time is: %f\n", ros::Time::now().toSec() - start_time.toSec());

  Eigen::MatrixXcd phy = Eigen::MatrixXcd::Zero(lqi_mode_ * 6, lqi_mode_ * 3);
  int j = 0;

  for(int i = 0; i < lqi_mode_ * 6; i++)
    {
      if(ces.eigenvalues()[i].real() < 0)
        {
          if(j >= lqi_mode_ * 3)
            {
              ROS_ERROR("nagativa sigular amount is larger");
              return false;
            }

          phy.col(j) = ces.eigenvectors().col(i);
          j++;
        }

    }

  if(j != lqi_mode_ * 3)
    {
      ROS_ERROR("nagativa sigular value amount is not enough");
      return false;
    }

  Eigen::MatrixXcd f = phy.block(0, 0, lqi_mode_ * 3, lqi_mode_ * 3);
  Eigen::MatrixXcd g = phy.block(lqi_mode_ * 3, 0, lqi_mode_ * 3, lqi_mode_ * 3);


  if(debug_verbose_) ROS_INFO("  start calculate f inv");
  start_time = ros::Time::now();
  Eigen::MatrixXcd f_inv  = f.inverse();
  if(control_verbose_)
    ROS_INFO("f inverse: %f\n", ros::Time::now().toSec() - start_time.toSec());

  if(debug_verbose_) ROS_INFO("  finish calculate f inv");

  Eigen::MatrixXcd P = g * f_inv;

  //K
  K_ = -R_inv * B.transpose() * P.real();

  if(control_verbose_)
    std::cout << "K is:" << std::endl << K_ << std::endl;

  if(a_dash_eigen_calc_flag_)
    {
      if(debug_verbose_) ROS_INFO("  start A eigen compute");
      //check the eigen of new A
      Eigen::MatrixXd A_dash = Eigen::MatrixXd::Zero(lqi_mode_ * 3, lqi_mode_ * 3);
      A_dash = A + B * K_;
      // start_time = ros::Time::now();
      Eigen::EigenSolver<Eigen::MatrixXd> esa(A_dash);
      if(debug_verbose_) ROS_INFO("  finish A eigen compute");
      if(control_verbose_)
        std::cout << "The eigenvalues of A_hash are:" << std::endl << esa.eigenvalues() << std::endl;
    }
  return true;
}

void TransformController::param2controller()
{
  aerial_robot_msgs::FourAxisGain four_axis_gain_msg;
  spinal::RollPitchYawTerms rpy_gain_msg; //for rosserial
  geometry_msgs::TransformStamped transform_msg; //for rosserial
  spinal::PMatrixPseudoInverseWithInertia p_pseudo_inverse_with_inertia_msg;

  four_axis_gain_msg.motor_num = rotor_num_;
  rpy_gain_msg.motors.resize(rotor_num_);
  p_pseudo_inverse_with_inertia_msg.pseudo_inverse.resize(rotor_num_);

  /* the transform from cog to baselink */
  transform_msg.header.stamp = joint_state_stamp_;
  transform_msg.header.frame_id = std::string("cog");
  transform_msg.child_frame_id = baselink_;
  tf::transformTFToMsg(getCog2Baselink(), transform_msg.transform);
  transform_pub_.publish(transform_msg);

  for(int i = 0; i < rotor_num_; i ++)
    {
      /* to flight controller via rosserial */
      rpy_gain_msg.motors[i].roll_p = K_(i,0) * 1000; //scale: x 1000
      rpy_gain_msg.motors[i].roll_d = K_(i,1) * 1000;  //scale: x 1000
      rpy_gain_msg.motors[i].roll_i = K_(i, lqi_mode_ * 2) * 1000; //scale: x 1000

      rpy_gain_msg.motors[i].pitch_p = K_(i,2) * 1000; //scale: x 1000
      rpy_gain_msg.motors[i].pitch_d = K_(i,3) * 1000; //scale: x 1000
      rpy_gain_msg.motors[i].pitch_i = K_(i,lqi_mode_ * 2 + 1) * 1000; //scale: x 1000

      /* to aerial_robot_base, feedback */
      four_axis_gain_msg.pos_p_gain_roll.push_back(K_(i,0));
      four_axis_gain_msg.pos_d_gain_roll.push_back(K_(i,1));
      four_axis_gain_msg.pos_i_gain_roll.push_back(K_(i,lqi_mode_ * 2));

      four_axis_gain_msg.pos_p_gain_pitch.push_back(K_(i,2));
      four_axis_gain_msg.pos_d_gain_pitch.push_back(K_(i,3));
      four_axis_gain_msg.pos_i_gain_pitch.push_back(K_(i,lqi_mode_ * 2 + 1));

      four_axis_gain_msg.pos_p_gain_alt.push_back(K_(i,4));
      four_axis_gain_msg.pos_d_gain_alt.push_back(K_(i,5));
      four_axis_gain_msg.pos_i_gain_alt.push_back(K_(i, lqi_mode_ * 2 + 2));

      if(lqi_mode_ == LQI_FOUR_AXIS_MODE)
        {
          /* to flight controller via rosserial */
          rpy_gain_msg.motors[i].yaw_d = K_(i,7) * 1000; //scale: x 1000

          /* to aerial_robot_base, feedback */
          four_axis_gain_msg.pos_p_gain_yaw.push_back(K_(i,6));
          four_axis_gain_msg.pos_d_gain_yaw.push_back(K_(i,7));
          four_axis_gain_msg.pos_i_gain_yaw.push_back(K_(i,11));

        }
      else if(lqi_mode_ == LQI_THREE_AXIS_MODE)
        {
          rpy_gain_msg.motors[i].yaw_d = 0;

          /* to aerial_robot_base, feedback */
          four_axis_gain_msg.pos_p_gain_yaw.push_back(0.0);
          four_axis_gain_msg.pos_d_gain_yaw.push_back(0.0);
          four_axis_gain_msg.pos_i_gain_yaw.push_back(0.0);
        }

      /* the p matrix pseudo inverse and inertia */
      p_pseudo_inverse_with_inertia_msg.pseudo_inverse[i].r = P_orig_pseudo_inverse_(i, 0) * 1000;
      p_pseudo_inverse_with_inertia_msg.pseudo_inverse[i].p = P_orig_pseudo_inverse_(i, 1) * 1000;
      p_pseudo_inverse_with_inertia_msg.pseudo_inverse[i].y = P_orig_pseudo_inverse_(i, 3) * 1000;
    }
  rpy_gain_pub_.publish(rpy_gain_msg);
  four_axis_gain_pub_.publish(four_axis_gain_msg);


  /* the multilink inertia */
  Eigen::Matrix3d inertia = getInertia();
  p_pseudo_inverse_with_inertia_msg.inertia[0] = inertia(0, 0) * 1000;
  p_pseudo_inverse_with_inertia_msg.inertia[1] = inertia(1, 1) * 1000;
  p_pseudo_inverse_with_inertia_msg.inertia[2] = inertia(2, 2) * 1000;
  p_pseudo_inverse_with_inertia_msg.inertia[3] = inertia(0, 1) * 1000;
  p_pseudo_inverse_with_inertia_msg.inertia[4] = inertia(1, 2) * 1000;
  p_pseudo_inverse_with_inertia_msg.inertia[5] = inertia(0, 2) * 1000;

  if(gyro_moment_compensation_)
    p_matrix_pseudo_inverse_inertia_pub_.publish(p_pseudo_inverse_with_inertia_msg);
}

void TransformController::cfgLQICallback(hydrus::LQIConfig &config, uint32_t level)
{
  if(config.lqi_gain_flag)
    {
      printf("LQI Param:");
      switch(level)
        {
        case LQI_RP_P_GAIN:
          q_roll_ = config.q_roll;
          q_pitch_ = config.q_roll;
          printf("change the gain of lqi roll and pitch p gain: %f\n", q_roll_);
          break;
        case LQI_RP_I_GAIN:
          q_roll_i_ = config.q_roll_i;
          q_pitch_i_ = config.q_roll_i;
          printf("change the gain of lqi roll and pitch i gain: %f\n", q_roll_i_);
          break;
        case LQI_RP_D_GAIN:
          q_roll_d_ = config.q_roll_d;
          q_pitch_d_ = config.q_roll_d;
          printf("change the gain of lqi roll and pitch d gain:%f\n", q_roll_d_);
          break;
        case LQI_Y_P_GAIN:
          q_yaw_ = config.q_yaw;
          printf("change the gain of lqi yaw p gain:%f\n", q_yaw_);
          break;
        case LQI_Y_I_GAIN:
          q_yaw_i_ = config.q_yaw_i;
          printf("change the gain of lqi yaw i gain:%f\n", q_yaw_i_);
          break;
        case LQI_Y_D_GAIN:
          q_yaw_d_ = config.q_yaw_d;
          printf("change the gain of lqi yaw d gain:%f\n", q_yaw_d_);
          break;
        case LQI_Z_P_GAIN:
          q_z_ = config.q_z;
          printf("change the gain of lqi z p gain:%f\n", q_z_);
          break;
        case LQI_Z_I_GAIN:
          q_z_i_ = config.q_z_i;
          printf("change the gain of lqi z i gain:%f\n", q_z_i_);
          break;
        case LQI_Z_D_GAIN:
          q_z_d_ = config.q_z_d;
          printf("change the gain of lqi z d gain:%f\n", q_z_d_);
          break;
        default :
          printf("\n");
          break;
        }
      q_diagonal_ << q_roll_,q_roll_d_,q_pitch_,q_pitch_d_,q_z_,q_z_d_,q_yaw_,q_yaw_d_, q_roll_i_,q_pitch_i_,q_z_i_,q_yaw_i_;
    }
}

tf::Transform TransformController::getRoot2Link(std::string link, sensor_msgs::JointState state)
{
  KDL::TreeFkSolverPos_recursive fk_solver(tree_);
  unsigned int nj = tree_.getNrOfJoints();
  KDL::JntArray joint_positions(nj);

  unsigned int j = 0;
  for(unsigned int i = 0; i < state.position.size(); i++)
    {
      std::map<std::string, uint32_t>::iterator itr = actuator_map_.find(state.name[i]);
      if(itr != actuator_map_.end())  joint_positions(actuator_map_.find(state.name[i])->second) = state.position[i];
    }

  KDL::Frame f;
  tf::Transform  link_f;
  int status = fk_solver.JntToCart(joint_positions, f, link);
  tf::transformKDLToTF(f, link_f);

  return link_f;
}
