#include <hydrus/transform_control.h>

TransformController::TransformController(ros::NodeHandle nh, ros::NodeHandle nh_private, bool callback_flag):
  nh_(nh), nh_private_(nh_private),
  realtime_control_flag_(true),
  callback_flag_(callback_flag),
  kinematics_flag_(false),
  rotor_num_(0)
{
  /* robot model */
  if (!model_.initParam("robot_description"))
    ROS_ERROR("Failed to extract urdf model from rosparam");
  if (!kdl_parser::treeFromUrdfModel(model_, tree_))
    ROS_ERROR("Failed to extract kdl tree from xml robot description");

  /* debug */
  //ROS_ERROR("root link: %s", GetTreeElementSegment(tree_.getRootSegment()->second).getName().c_str());
  //ROS_ERROR("joint num: %d", tree_.getNrOfJoints());
  addChildren(tree_.getRootSegment());
  ROS_ERROR("rotor num; %d", rotor_num_);

  initParam();

  rotors_origin_from_cog_.resize(rotor_num_);

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
  rpy_gain_pub_ = nh_.advertise<aerial_robot_msgs::RollPitchYawTerms>(rpy_gain_pub_name, 1);
  four_axis_gain_pub_ = nh_.advertise<aerial_robot_msgs::FourAxisGain>("/four_axis_gain", 1);
  transform_pub_ = nh_.advertise<geometry_msgs::TransformStamped>("/cog2baselink", 1);

  //dynamic reconfigure server
  lqi_server_ = new dynamic_reconfigure::Server<hydrus::LQIConfig>(nh_private_);
  dynamic_reconf_func_lqi_ = boost::bind(&TransformController::cfgLQICallback, this, _1, _2);
  lqi_server_->setCallback(dynamic_reconf_func_lqi_);

  /* ros service for extra module */
  add_extra_module_service_ = nh_.advertiseService("add_extra_module", &TransformController::addExtraModuleCallback, this);

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
          ROS_INFO("Floating joint. Not adding segment from %s to %s. This TF can not be published based on joint_states info", parent.c_str(), child.getName().c_str());
        }
        else {
          ROS_INFO("Adding fixed segment from %s to %s, joint: %s, [%f, %f, %f], q_nr: %d", parent.c_str(), child.getName().c_str(), child.getJoint().getName().c_str(), child.getFrameToTip().p.x(), child.getFrameToTip().p.y(), child.getFrameToTip().p.z(), children[i]->second.q_nr);
        }

        /* for the most parent link, e.g. link1 */
        if(inertia_map_.size() == 0 && parent.find("root") != std::string::npos)
          {
            inertia_map_.insert(std::make_pair(child.getName(), child.getInertia()));
            ROS_WARN("Add new inertia base link: %s", child.getName().c_str());
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
        ROS_INFO("Adding moving segment from %s to %s, joint: %s,  [%f, %f, %f], q_nr: %d", parent.c_str(), child.getName().c_str(), child.getJoint().getName().c_str(), child.getFrameToTip().p.x(), child.getFrameToTip().p.y(), child.getFrameToTip().p.z(), children[i]->second.q_nr);


        /* add the new inertia base (child) link if the joint is not a rotor */
        if(child.getJoint().getName().find("rotor") == std::string::npos)
          {
            /* create a new inertia base link */
            inertia_map_.insert(std::make_pair(child.getName(), child.getInertia()));
            ROS_WARN("Add new inertia base link: %s", child.getName().c_str());
          }
        else
          {
            std::map<std::string, KDL::RigidBodyInertia>::iterator it = inertia_map_.find(parent);
            KDL::RigidBodyInertia parent_prev_inertia = it->second;
            inertia_map_[parent] = child.getFrameToTip() * child.getInertia() + parent_prev_inertia;

            /* add the rotor direction */
            ROS_WARN("%s, rototation is %f", child.getJoint().getName().c_str(), child.getJoint().JointAxis().z());
            rotor_direction_.insert(std::make_pair(std::atoi(child.getJoint().getName().substr(5).c_str()), child.getJoint().JointAxis().z()));
          }
      }

      /* count the rotor */
      if(child.getName().find("propeller") != std::string::npos) rotor_num_++;
      addChildren(children[i]);
    }
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
  std::cout << "control_rate: " << std::setprecision(3) << control_rate_ << std::endl;

  nh_private_.param("only_three_axis_mode", only_three_axis_mode_, false);
  nh_private_.param("control_verbose", control_verbose_, false);
  nh_private_.param("kinematic_verbose", kinematic_verbose_, false);
  nh_private_.param("debug_verbose", debug_verbose_, false);
  nh_private_.param("a_dash_eigen_calc_flag", a_dash_eigen_calc_flag_, false);

  /* base link */
  nh_private_.param("baselink", baselink_, std::string("link1"));
  std::cout << "baselink: " << baselink_ << std::endl;
  /* propeller direction and lqi R */
  r_.resize(rotor_num_);
  for(int i = 0; i < rotor_num_; i++)
    {
      std::stringstream ss;
      ss << i + 1;
      /* R */
      nh_private_.param(std::string("r") + ss.str(), r_[i], 1.0);
    }

  nh_private_.param ("dist_thre", dist_thre_, 0.05);
  std::cout << "dist_thre: " << std::setprecision(3) << dist_thre_ << std::endl;
  nh_private_.param ("f_max", f_max_, 8.6);
  std::cout << "f_max: " << std::setprecision(3) << f_max_ << std::endl;
  nh_private_.param ("f_min", f_min_, 2.0);
  std::cout << "f_min: " << std::setprecision(3) << f_min_ << std::endl;

  nh_private_.param ("q_roll", q_roll_, 1.0);
  std::cout << "Q: q_roll: " << std::setprecision(3) << q_roll_ << std::endl;
  nh_private_.param ("q_roll_d", q_roll_d_, 1.0);
  std::cout << "Q: q_roll_d: " << std::setprecision(3) << q_roll_d_ << std::endl;
  nh_private_.param ("q_pitch", q_pitch_, 1.0);
  std::cout << "Q: q_pitch: " << std::setprecision(3) << q_pitch_ << std::endl;
  nh_private_.param ("q_pitch_d", q_pitch_d_,  1.0);
  std::cout << "Q: q_pitch_d: " << std::setprecision(3) << q_pitch_d_ << std::endl;
  nh_private_.param ("q_yaw", q_yaw_, 1.0);
  std::cout << "Q: q_yaw: " << std::setprecision(3) << q_yaw_ << std::endl;
  nh_private_.param ("strong_q_yaw", strong_q_yaw_, 1.0);
  std::cout << "Q: strong_q_yaw: " << std::setprecision(3) << strong_q_yaw_ << std::endl;
  nh_private_.param ("q_yaw_d", q_yaw_d_, 1.0);
  std::cout << "Q: q_yaw_d: " << std::setprecision(3) << q_yaw_d_ << std::endl;
  nh_private_.param ("q_z", q_z_, 1.0);
  std::cout << "Q: q_z: " << std::setprecision(3) << q_z_ << std::endl;
  nh_private_.param ("q_z_d", q_z_d_, 1.0);
  std::cout << "Q: q_z_d: " << std::setprecision(3) << q_z_d_ << std::endl;

  nh_private_.param ("q_roll_i", q_roll_i_, 1.0);
  std::cout << "Q: q_roll_i: " << std::setprecision(3) << q_roll_i_ << std::endl;
  nh_private_.param ("q_pitch_i", q_pitch_i_, 1.0);
  std::cout << "Q: q_pitch_i: " << std::setprecision(3) << q_pitch_i_ << std::endl;
  nh_private_.param ("q_yaw_i", q_yaw_i_, 1.0);
  std::cout << "Q: q_yaw_i: " << std::setprecision(3) << q_yaw_i_ << std::endl;
  nh_private_.param ("q_z_i", q_z_i_, 1.0);
  std::cout << "Q: q_z_i: " << std::setprecision(3) << q_z_i_ << std::endl;

  /* dynamics: motor */
  ros::NodeHandle control_node("/motor_info");
  control_node.param("pwm_rate", pwm_rate_, 1.0);
  std::cout << "pwm_rate: " << std::setprecision(3) << pwm_rate_ << std::endl;
  control_node.param("m_f_rate", m_f_rate_, 0.01); //-0.016837; //the sgn is right?, should be nagative
  std::cout << "m_f_rate: " << std::setprecision(3) << m_f_rate_ << std::endl;
  control_node.param("f_pwm_rate", f_pwm_rate_, 1.0); //0.3029; // with the pwm percentage: x / 1800 * 100
  std::cout << "f_pwm_rate: " << std::setprecision(3) << f_pwm_rate_ << std::endl;
  control_node.param("f_pwm_offset", f_pwm_offset_, 0.0); // -21.196;  // with the pwm percentage: x / 1800 * 100
  std::cout << "f_pwm_offset: " << std::setprecision(3) <<f_pwm_offset_ << std::endl;

}

void TransformController::control()
{
  ros::Rate loop_rate(control_rate_);
  static int i = 0;
  static int cnt = 0;

  if(!realtime_control_flag_) return;
  while(ros::ok())
    {
      if(debug_verbose_) ROS_ERROR("start lqi");
      lqi();
      if(debug_verbose_) ROS_ERROR("finish lqi");

      loop_rate.sleep();
    }
}

void TransformController::desireCoordinateCallback(const aerial_robot_base::DesireCoordConstPtr & msg)
{
  cog_desire_orientation_ = KDL::Rotation::RPY(msg->roll, msg->pitch, msg->yaw);
}

void TransformController::jointStatecallback(const sensor_msgs::JointStateConstPtr& state)
{
  joint_state_stamp_ = state->header.stamp;

  if(debug_verbose_) ROS_ERROR("start kinematics");
  kinematics(*state);
  if(debug_verbose_) ROS_ERROR("finish kinematics");

  br_.sendTransform(tf::StampedTransform(getCog(), state->header.stamp, GetTreeElementSegment(tree_.getRootSegment()->second).getName(), "cog"));

  if(!kinematics_flag_)
    {
      ROS_ERROR("the total mass is %f", getMass());
      kinematics_flag_ = true;
    }
}

void TransformController::kinematics(sensor_msgs::JointState state)
{
  KDL::TreeFkSolverPos_recursive fk_solver(tree_);
  unsigned int nj = tree_.getNrOfJoints();
  KDL::JntArray jointpositions(nj);

  unsigned int j = 0;
  for(unsigned int i = 0; i < state.position.size(); i++)
    {
      if(state.name[i].find("joint") != std::string::npos)
        {
          jointpositions(j) = state.position[i];
          j++;
        }
    }

  KDL::RigidBodyInertia link_inertia = KDL::RigidBodyInertia::Zero();
  KDL::Frame cog_frame;
  for(auto it = inertia_map_.begin(); it != inertia_map_.end(); ++it)
    {
      KDL::Frame f;
      int status = fk_solver.JntToCart(jointpositions, f, it->first);
      //ROS_ERROR(" %s status is : %d, [%f, %f, %f]", it->first.c_str(), status, f.p.x(), f.p.y(), f.p.z());
      KDL::RigidBodyInertia link_inertia_tmp = link_inertia;
      link_inertia = link_inertia_tmp + f * it->second;

      /* CoG */
      if(it->first == baselink_) cog_frame.M = f.M * cog_desire_orientation_.Inverse();
    }
  cog_frame.p = link_inertia.getCOG();
  tf::Transform cog_transform;
  tf::transformKDLToTF(cog_frame, cog_transform);
  setCog(cog_transform);
  setMass(link_inertia.getMass());

  /* rotor(propeller) based on COG */
  std::vector<Eigen::Vector3d> f_rotors;
  for(int i = 0; i < rotor_num_; i++)
    {
      std::stringstream ss;
      ss << i + 1;
      std::string rotor = std::string("propeller") + ss.str();

      KDL::Frame f;
      int status = fk_solver.JntToCart(jointpositions, f, rotor);
      //ROS_ERROR(" %s status is : %d, [%f, %f, %f]", rotor.c_str(), status, f.p.x(), f.p.y(), f.p.z());
      f_rotors.push_back(Eigen::Map<const Eigen::Vector3d>((cog_frame.Inverse() * f).p.data));

      //std::cout << "rotor" << i + 1 << ": \n"<< f_rotors[i] << std::endl;
    }

  setRotorsFromCog(f_rotors);
  KDL::RigidBodyInertia link_inertia_from_cog = cog_frame.Inverse() * link_inertia;
  setInertia(Eigen::Map<const Eigen::Matrix3d>(link_inertia_from_cog.getRotationalInertia().data));


  /* find the transform from cog to baselink */
  KDL::Frame f;
  int status = fk_solver.JntToCart(jointpositions, f, baselink_);

  //setCog2Baselink(Eigen::Map<const Eigen::Vector3d>((cog_frame.Inverse() * f).p.data));
  tf::Transform cog2baselink_transform;
  tf::transformKDLToTF(cog_frame.Inverse() * f, cog2baselink_transform);
  setCog2Baselink(cog2baselink_transform);
}

bool TransformController::addExtraModuleCallback(hydrus::AddExtraModule::Request  &req,
                                                 hydrus::AddExtraModule::Response &res)
{
  res.status = addExtraModule(req.extra_module_link, req.extra_module_mass, req.extra_module_offset);
  return true;
}


bool TransformController::addExtraModule(int extra_module_link, float extra_module_mass, float extra_module_offset)
{
  std::stringstream ss;
  ss << extra_module_link;
  std::string parent = std::string("link") + ss.str();

  KDL::RigidBodyInertia extra_inertia(extra_module_mass, KDL::Vector(extra_module_offset, 0, 0), KDL::RotationalInertia::Zero());

  std::map<std::string, KDL::RigidBodyInertia>::iterator it = inertia_map_.find(parent);
  if(it == inertia_map_.end())
    {
      ROS_ERROR("add extra module: can not find the base link: %s", parent.c_str());
      return false;
    }

  KDL::RigidBodyInertia parent_prev_inertia = it->second;
  inertia_map_[parent] = extra_inertia + parent_prev_inertia;

  ROS_INFO("Add extra module, %s, mass: %f, offset: %f",
           parent.c_str(), extra_module_mass, extra_module_offset);

  return true;
}

void TransformController::lqi()
{
  if(!kinematics_flag_) return;

  /* check the thre check */
  if(debug_verbose_) ROS_WARN(" start dist thre check");
  if(!distThreCheck()) //[m]
    {
      ROS_ERROR("LQI: invalid pose, can pass the distance thresh check");
      return;
    }
  if(debug_verbose_) ROS_WARN(" finish dist thre check");

  /* modelling the multilink based on the inertia assumption */
  if(debug_verbose_) ROS_WARN(" start modelling");
  if(!modelling())
    {
      if(!only_three_axis_mode_)  ROS_ERROR("LQI: invalid pose, can not be four axis stable, switch to three axis stable mode");
    }
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

bool TransformController::distThreCheck()
{
  float x_minus_max_dist = 0, x_plus_max_dist = 0;
  float y_minus_max_dist = 0, y_plus_max_dist = 0;

  for(int i = 0; i < rotor_num_; i++)
    {

      std::vector<Eigen::Vector3d> rotors_origin_from_cog(rotor_num_);
      getRotorsFromCog(rotors_origin_from_cog);
      //x
      if(rotors_origin_from_cog[i](0) > 0 && rotors_origin_from_cog[i](0) > x_plus_max_dist)
        x_plus_max_dist = rotors_origin_from_cog[i](0);
      if(rotors_origin_from_cog[i](0) < 0 && rotors_origin_from_cog[i](0) < x_minus_max_dist)
        x_minus_max_dist = rotors_origin_from_cog[i](0);
      //y
      if(rotors_origin_from_cog[i](1) > 0 && rotors_origin_from_cog[i](1) > y_plus_max_dist)
        y_plus_max_dist = rotors_origin_from_cog[i](1);
      if(rotors_origin_from_cog[i](1) < 0 && rotors_origin_from_cog[i](1) < y_minus_max_dist)
        y_minus_max_dist = rotors_origin_from_cog[i](1);
    }

  if(x_plus_max_dist < dist_thre_ || -x_minus_max_dist < dist_thre_ ||
     y_plus_max_dist < dist_thre_ || -y_minus_max_dist < dist_thre_ ) //[m]
    {
      return false;
    }
  return true;
}

bool  TransformController::modelling(bool verbose)
{
  std::vector<Eigen::Vector3d> rotors_origin_from_cog(rotor_num_);
  getRotorsFromCog(rotors_origin_from_cog);
  Eigen::Matrix3d links_inertia = getInertia();

  Eigen::VectorXd x;
  x.resize(rotor_num_);
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
  if(kinematic_verbose_)
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
  x = P_.transpose() * lamda;

  double P_det = (P_ * P_.transpose()).determinant();
  if(control_verbose_)
    std::cout << "P det:"  << std::endl << P_det << std::endl;

  if(control_verbose_)
    ROS_INFO("P solver is: %f\n", ros::Time::now().toSec() - start_time.toSec());

  if(control_verbose_ || verbose)
    std::cout << "x:"  << std::endl << x << std::endl;

  if(x.maxCoeff() > f_max_ || x.minCoeff() < f_min_ || P_det < 1e-6 || only_three_axis_mode_)
    {
      lqi_mode_ = LQI_THREE_AXIS_MODE;

      //no yaw constraint
      Eigen::MatrixXd P_dash = Eigen::MatrixXd::Zero(3, rotor_num_);
      P_dash.row(0) = P_.row(0);
      P_dash.row(1) = P_.row(1);
      P_dash.row(2) = P_.row(3);
      Eigen::VectorXd g3(3);
      g3 << 0, 0, 9.8;
      Eigen::FullPivLU<Eigen::MatrixXd> solver((P_dash * P_dash.transpose()));
      Eigen::VectorXd lamda;
      lamda = solver.solve(g3);
      x = P_dash.transpose() * lamda;
      if(control_verbose_)
        std::cout << "x:"  << std::endl << x << std::endl;

      return false; //can not be stable
    }

  else lqi_mode_ = LQI_FOUR_AXIS_MODE;

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
  aerial_robot_msgs::RollPitchYawTerms rpy_gain_msg; //for rosserial
  geometry_msgs::TransformStamped transform_msg; //for rosserial

  four_axis_gain_msg.motor_num = rotor_num_;
  rpy_gain_msg.motors.resize(rotor_num_);

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
    }
  rpy_gain_pub_.publish(rpy_gain_msg);
  four_axis_gain_pub_.publish(four_axis_gain_msg);
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
      q_diagonal_ << q_roll_,q_roll_d_,q_pitch_,q_pitch_d_,q_yaw_,q_yaw_d_,q_z_,q_z_d_, q_roll_i_,q_pitch_i_,q_yaw_i_,q_z_i_;
    }
}
