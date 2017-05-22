#include <hydrus_transform_control/transform_control.h>

TransformController::TransformController(ros::NodeHandle nh, ros::NodeHandle nh_private, bool callback_flag): nh_(nh),nh_private_(nh_private)
{
  initParam();

  //thread
  realtime_control_flag_ = true;
  callback_flag_ = callback_flag;
  /* the mutex lock should be used in multi thread process!! if there is only one thream, the value of mutex will be wrong */
  /* the lock guard is lock the process until the scope(bock) end */
  //multi_thread_flag_ = callback_flag_; //not good

  cog_(0) = 0;
  cog_(1) = 0;
  cog_(2) = 0;

  links_origin_from_cog_.resize(link_num_);
  rotate_matrix_ = Eigen::Matrix3d::Identity();
  cog_matrix_ = Eigen::Matrix3d::Identity();

  //U
  U_ = Eigen::MatrixXd::Zero(4,link_num_); //4 is the link number, should change!!

  //A
  A8_ = Eigen::MatrixXd::Zero(8,8);
  A8_(0,1) = 1;
  A8_(2,3) = 1;
  A8_(4,5) = 1;
  A8_(6,7) = 1;

  A6_ = Eigen::MatrixXd::Zero(6,6);
  A6_(0,1) = 1;
  A6_(2,3) = 1;
  A6_(4,5) = 1;


  //B
  B8_ = Eigen::MatrixXd::Zero(8,link_num_); //4 is the link number, should change!!
  B6_ = Eigen::MatrixXd::Zero(6,link_num_); //4 is the link number, should change!!

  //C
  C8_ = Eigen::MatrixXd::Zero(4,8);
  C8_(0,0) = 1;
  C8_(1,2) = 1;
  C8_(2,4) = 1;
  C8_(3,6) = 1;
  C6_ = Eigen::MatrixXd::Zero(3,6);
  C6_(0,0) = 1;
  C6_(1,2) = 1;
  C6_(2,4) = 1;


  //A_aug
  A12_aug_ = Eigen::MatrixXd::Zero(12,12);
  A12_aug_.block<8,8>(0,0) = A8_;
  A12_aug_.block<4,8>(8,0) = -C8_;

  A9_aug_ = Eigen::MatrixXd::Zero(9,9);
  A9_aug_.block<6,6>(0,0) = A6_;
  A9_aug_.block<3,6>(6,0) = -C6_;


  //A_aug
  C12_aug_ = Eigen::MatrixXd::Zero(4,12);
  C12_aug_.block<4,8>(0,0) = C8_;

  C9_aug_ = Eigen::MatrixXd::Zero(3,9);
  C9_aug_.block<3,6>(0,0) = C6_;


  //Q
  Eigen::VectorXd  q12_diagonal(12);
  q12_diagonal << q_roll_,q_roll_d_,q_pitch_,q_pitch_d_,q_yaw_,q_yaw_d_,q_z_,q_z_d_, q_roll_i_,q_pitch_i_,q_yaw_i_,q_z_i_;
  Q12_ = q12_diagonal.asDiagonal();

  Eigen::VectorXd  q9_diagonal(9);
  q9_diagonal << q_roll_,q_roll_d_,q_pitch_,q_pitch_d_,q_z_,q_z_d_, q_roll_i_,q_pitch_i_, q_z_i_;
  Q9_ = q9_diagonal.asDiagonal();

  //R
  R_  = Eigen::MatrixXd::Zero(link_num_, link_num_);
  for(int i = 0; i < link_num_; i ++)
    R_(i,i) = r_[i];

  std::cout << "A8:"  << std::endl << A8_ << std::endl;
  std::cout << "C8:"  << std::endl << C8_ << std::endl;
  std::cout << "A12_aug:"  << std::endl << A12_aug_ << std::endl;
  std::cout << "C12_aug:"  << std::endl << C12_aug_ << std::endl;
  std::cout << "Q12:"  << std::endl << Q12_ << std::endl;

  std::cout << "A6:"  << std::endl << A6_ << std::endl;
  std::cout << "C6:"  << std::endl << C6_ << std::endl;
  std::cout << "A9_aug:"  << std::endl << A9_aug_ << std::endl;
  std::cout << "C9_aug:"  << std::endl << C9_aug_ << std::endl;
  std::cout << "Q9:"  << std::endl << Q9_ << std::endl;

  std::cout << "R:"  << std::endl << R_ << std::endl;

  lqi_mode_ = LQI_FOUR_AXIS_MODE;

  //those publisher is published from func param2controller
  rpy_gain_pub_ = nh_.advertise<hydrus_transform_control::RollPitchYawGain>(rpy_gain_pub_name_, 1);
  four_axis_gain_pub_ = nh_.advertise<aerial_robot_msgs::FourAxisGain>("four_axis_gain", 1);

  //dynamic reconfigure server
  lqi_server_ = new dynamic_reconfigure::Server<hydrus_transform_control::LQIConfig>(nh_private_);
  dynamic_reconf_func_lqi_ = boost::bind(&TransformController::cfgLQICallback, this, _1, _2);
  lqi_server_->setCallback(dynamic_reconf_func_lqi_);

  if(callback_flag_)
    {// the name callback flag is not correct
      realtime_control_sub_ = nh_.subscribe<std_msgs::UInt8>("realtime_control", 1, &TransformController::realtimeControlCallback, this, ros::TransportHints().tcpNoDelay());

      principal_axis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("orientation_data", 1);

      yaw_gain_sub_ = nh_.subscribe<std_msgs::UInt8>(yaw_pos_gain_sub_name_, 1, &TransformController::yawGainCallback, this, ros::TransportHints().tcpNoDelay());

      add_extra_module_service_ = nh_.advertiseService("add_extra_module", &TransformController::addExtraModuleCallback, this);

      cog_rotate_pub_ = nh_.advertise<aerial_robot_base::DesireCoord>("/desire_coordinate", 1); //absolute

      control_thread_ = boost::thread(boost::bind(&TransformController::control, this));
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

  nh_private_.param("rpy_gain_pub_name", rpy_gain_pub_name_, std::string("/rpy_gain"));
  nh_private_.param("yaw_pos_gain_sub_name", yaw_pos_gain_sub_name_, std::string("/yaw_pos_gain"));

  nh_private_.param("control_verbose", control_verbose_, false);
  nh_private_.param("kinematic_verbose", kinematic_verbose_, false);
  nh_private_.param("debug_verbose", debug_verbose_, false);
  nh_private_.param("a_dash_eigen_calc_flag", a_dash_eigen_calc_flag_, false);

  /* number of links */
  nh_private_.param("link_num", link_num_, 4);
  std::cout << " link num: " << link_num_ << std::endl;

  /* lqi */
  r_.resize(link_num_);
  for(int i = 0; i < link_num_; i++)
    {
      std::stringstream ss;
      ss << i + 1;
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

  /* kinematics */
  links_name_.resize(link_num_);
  propeller_direction_.resize(link_num_);
  propeller_order_.resize(link_num_);
  link_base_model_.resize(link_num_);
  link_base_rod_mass_.resize(link_num_);
  link_joint_model_.resize(link_num_ -1);
  all_mass_ = 0;

  /* root link */
  nh_private_.param("root_link", root_link_, 3);
  std::stringstream ss;
  ss << root_link_;
  root_link_name_ = std::string("/link") + ss.str();
  std::cout << " root_link_name_: " << root_link_name_ << std::endl;

  /* base components position [m] */
  nh_private_.param ("link_length", link_length_, 0.44);
  std::cout << "link_length: " << std::setprecision(3) << link_length_ << std::endl;
  nh_private_.param ("link_base_rod_length", link_base_rod_length_, 0.4);
  std::cout << "link_base_rod_length: " << std::setprecision(3) << link_base_rod_length_ << std::endl;
  nh_private_.param ("ring_radius", ring_radius_, 0.1425);
  std::cout << "ring_radius: " << std::setprecision(3) << ring_radius_ << std::endl;
  nh_private_.param ("link_joint_offset", link_joint_offset_, 0.22);
  std::cout << "link_joint_offset: " << std::setprecision(3) << link_joint_offset_ << std::endl;

  /* base components mass [Kg] */
  nh_private_.param ("link_base_rod_mid_mass", link_base_rod_mid_mass_, 0.1);
  std::cout << "link_base_rod_mid_mass: " << std::setprecision(3) << link_base_rod_mid_mass_ << std::endl;
  nh_private_.param ("link_base_ring_mass", link_base_ring_mass_, 0.048);
  std::cout << "link_base_ring_mass: " << std::setprecision(3) << link_base_ring_mass_ << std::endl;
  nh_private_.param ("link_base_two_ring_holder_mass", link_base_two_ring_holder_mass_, 0.053);
  std::cout << "link_base_two_ring_holder_mass: " << std::setprecision(3) << link_base_two_ring_holder_mass_ << std::endl;
  nh_private_.param ("link_base_center_mass", link_base_center_mass_, 0.238);
  std::cout << "link_base_center_mass: " << std::setprecision(3) << link_base_center_mass_ << std::endl;
  nh_private_.param ("link_joint_mass", link_joint_mass_,0.0785);
  std::cout << "link_joint_mass: " << std::setprecision(3) << link_joint_mass_ << std::endl;

  Eigen::Matrix3d zero_inertia = Eigen::Matrix3d::Zero(3,3);
  for(int i = 0; i < link_num_; i++)
    {
      std::stringstream ss;
      ss << i + 1;
      links_name_[i] = std::string("/link") + ss.str(); //link name

      nh_private_.param(std::string("link") + ss.str() + std::string("_base_rod_mass"), link_base_rod_mass_[i], link_base_rod_mid_mass_);
      std::cout << "link " << i + 1 << "_base_rod_mass:" << std::setprecision(3) << link_base_rod_mass_[i] << std::endl;

      /* model */
      /* 1. link base model */
      float link_base_model_weight = link_base_rod_mass_[i] + link_base_ring_mass_ + link_base_two_ring_holder_mass_ + link_base_center_mass_;

      Eigen::Matrix3d link_base_model_inertia;
      /* ring model + two_end model + rod model */
      link_base_model_inertia <<
        link_base_ring_mass_ / 2 * ring_radius_ * ring_radius_, 0, 0,
         0, (link_base_ring_mass_/2 + link_base_two_ring_holder_mass_) * ring_radius_ * ring_radius_ + link_base_rod_mass_[i] * link_base_rod_length_ * link_base_rod_length_ / 12, 0,
         0, 0, (link_base_ring_mass_ + link_base_two_ring_holder_mass_) * ring_radius_ * ring_radius_ + link_base_rod_mass_[i] * link_base_rod_length_ * link_base_rod_length_ / 12;

      ElementModel link_base_model(i, link_base_model_weight, link_base_model_inertia);
      link_base_model_[i] = link_base_model;
      all_mass_ += link_base_model.getWeight();

      /* 2. link joint model */
      if(i+1 < link_num_)
        {
          Eigen::Vector3d offset_;
          offset_ << link_joint_offset_, 0, 0;
          ElementModel link_joint_model(i + 1, link_joint_mass_, zero_inertia, offset_);
          link_joint_model_[i] = link_joint_model;
          all_mass_ += link_joint_model.getWeight();
        }

      /* propeller */
      nh_private_.param(std::string("propeller") + ss.str() + std::string("_direction"), propeller_direction_[i], 1);
      nh_private_.param(std::string("propeller") + ss.str() + std::string("_order"), propeller_order_[i], i);
    }

  /* 3. extra module */
  nh_private_.param ("extra_module_num", extra_module_num_, 2); //default: MCU + PC
  std::cout << "extra_module_num: " << extra_module_num_ << std::endl;
  extra_module_model_.resize(extra_module_num_);

  for(int i = 0; i < extra_module_num_; i++)
    {
      std::stringstream ss;
      ss << i + 1;

      double extra_module_offset;
      int extra_module_link;
      double extra_module_mass;
      double extra_module_ixx, extra_module_iyy, extra_module_izz;
      nh_private_.param (std::string("extra_module") + ss.str() + std::string("_link"), extra_module_link, link_num_ / 2 - 1);
      std::cout << std::string("extra_module") + ss.str() + std::string("_link: ") << extra_module_link << std::endl;
      nh_private_.param (std::string("extra_module") + ss.str() + std::string("_mass"), extra_module_mass, 0.0);
      std::cout << std::string("extra_module") + ss.str() + std::string("_mass: ") << std::setprecision(3) << extra_module_mass << std::endl;
      nh_private_.param (std::string("extra_module") + ss.str() + std::string("_offset"), extra_module_offset, 0.0);
      std::cout << std::string("extra_module") + ss.str() + std::string("_offset: ") << std::setprecision(3) << extra_module_offset << std::endl;
      nh_private_.param (std::string("extra_module") + ss.str() + std::string("_ixx"), extra_module_ixx, 0.0);
      std::cout << std::string("extra_module") + ss.str() + std::string("_ixx: ") << std::setprecision(3) << extra_module_ixx << std::endl;
      nh_private_.param (std::string("extra_module") + ss.str() + std::string("_iyy"), extra_module_iyy, 0.0);
      std::cout << std::string("extra_module") + ss.str() + std::string("_iyy: ") << std::setprecision(3) << extra_module_iyy << std::endl;
      nh_private_.param (std::string("extra_module") + ss.str() + std::string("_izz"), extra_module_izz, 0.0);
      std::cout << std::string("extra_module") + ss.str() + std::string("_izz: ") << std::setprecision(3) << extra_module_izz << std::endl;
      Eigen::Vector3d extra_module_i_diagnoal;
      extra_module_i_diagnoal << extra_module_ixx, extra_module_iyy, extra_module_izz;

      Eigen::Vector3d offset;
      offset << extra_module_offset, 0, 0;
      ElementModel extra_module_model(extra_module_link, extra_module_mass, extra_module_i_diagnoal.asDiagonal(), offset, true);
      all_mass_ += extra_module_model.getWeight();
      extra_module_model_[i] = extra_module_model;
    }
  ROS_INFO("Mass :%f", all_mass_);
}

void TransformController::control()
{
  ros::Rate loop_rate(control_rate_);
  static int i = 0;
  static int cnt = 0;

  while(ros::ok())
    {
      if(realtime_control_flag_)
        {
          /* Kinematics calculation */
          if(debug_verbose_) ROS_ERROR("start kinematics");
          bool get_kinematics = kinematics();
          if(debug_verbose_) ROS_ERROR("finish kinematics");
          /* LQI parameter calculation */
          if(debug_verbose_) ROS_ERROR("start lqi");
          if(get_kinematics) lqi();
          if(debug_verbose_) ROS_ERROR("finish lqi");
        }
      loop_rate.sleep();
    }
}

bool TransformController::kinematics()
{
  //get transform;
  std::vector<tf::StampedTransform>  transforms;
  transforms.resize(link_num_);

  ros::Duration dur (0.02);
  if (tf_.waitForTransform(root_link_name_, links_name_[link_num_ - 1], ros::Time(0),dur))
    {
      if(debug_verbose_) ROS_WARN(" start tf listening");
      for(int i = 0; i < link_num_; i++)
        {
          try
            {
              tf_.lookupTransform(root_link_name_, links_name_[i], ros::Time(0), transforms[i]);
            }
          catch (tf::TransformException ex)
            {
              ROS_ERROR("%s",ex.what());
              return false;
            }
        }
      if(debug_verbose_) ROS_WARN(" finish tf listening");

      if(debug_verbose_) ROS_WARN(" start cog compute");
      cogComputation(transforms);
      if(debug_verbose_) ROS_WARN(" finish cog compute");
      if(debug_verbose_) ROS_WARN(" start inertial compute");
      principalInertiaComputation(transforms);
      if(debug_verbose_) ROS_WARN(" finish inertial compute");
      if(debug_verbose_) ROS_WARN(" start cog coord pub");
      cogCoordPublish();
      if(debug_verbose_) ROS_WARN(" finsih cog coord pub");
      if(debug_verbose_) ROS_WARN(" start visualize");
      visualization();
      if(debug_verbose_) ROS_WARN(" finish visualize");

      return true;
    }
  return false;
}

bool TransformController::addExtraModuleCallback(hydrus_transform_control::AddExtraModule::Request  &req,
                                                 hydrus_transform_control::AddExtraModule::Response &res)
{
  addExtraModule(req.extra_module_link, req.extra_module_mass, req.extra_module_offset); 
  res.status = true;
  return true;
}


void TransformController::addExtraModule(int extra_module_link, float extra_module_mass, float extra_module_offset)
{
  Eigen::Vector3d offset;
  offset << extra_module_offset, 0, 0;
  Eigen::Matrix3d zero_inertia = Eigen::Matrix3d::Zero(3,3);
  ElementModel extra_module_model(extra_module_link, extra_module_mass, zero_inertia, offset);
  all_mass_ += extra_module_model.getWeight();
  extra_module_num_++;
  extra_module_model_.push_back(extra_module_model);

  ROS_INFO("Add extra module, link: %d, mass: %f, offset: %f",
           extra_module_link, extra_module_mass, extra_module_offset);
}

void TransformController::cogComputation(const std::vector<tf::StampedTransform>& transforms)
{
  Eigen::Vector3d cog_n  = Eigen::Vector3d::Zero();

  for(int i = 0; i < link_num_; i++)
    {
      Eigen::Vector3d link_origin;
      tf::vectorTFToEigen(transforms[i].getOrigin(), link_origin);

      Eigen::Quaterniond q;
      tf::quaternionTFToEigen(transforms[i].getRotation(), q);
      Eigen::Matrix3d link_rotate(q);

      /* 1. link base model */
      cog_n += link_origin * link_base_model_[i].getWeight();
      if(i > 0)
        {
          /* 2. link joint model */
          /* transformation */
          Eigen::Vector3d joint_origin = link_rotate * link_joint_model_[i -1].getOffset()  + link_origin;
          cog_n += joint_origin * link_joint_model_[i - 1].getWeight();
        }
    }

  /* 3. extra module */
  for(int i = 0; i < extra_module_num_; i++)
    {
      int link_no = extra_module_model_[i].getLink();
      Eigen::Vector3d link_origin;
      tf::vectorTFToEigen(transforms[link_no].getOrigin(), link_origin);

      Eigen::Quaterniond q;
      tf::quaternionTFToEigen(transforms[link_no].getRotation(), q);
      Eigen::Matrix3d link_rotate(q);

      Eigen::Vector3d extra_module_origin = link_rotate * extra_module_model_[i].getOffset()  + link_origin;
      cog_n += (extra_module_origin * extra_module_model_[i].getWeight());
    }

  Eigen::Vector3d cog = cog_n / all_mass_;
  setCog(cog);
}

void TransformController::principalInertiaComputation(const std::vector<tf::StampedTransform>& transforms, bool continuous_flag)
{
  static bool init_flag = false;

  Eigen::Matrix3d links_inertia = Eigen::Matrix3d::Zero(3,3);
  std::vector<Eigen::Vector3d > links_origin_from_cog;
  links_origin_from_cog.resize(link_num_);
  for(int i = 0; i < link_num_; i ++)
    {
      /* offset */
      Eigen::Vector3d origin_from_root_link;
      tf::vectorTFToEigen(transforms[i].getOrigin(), origin_from_root_link);

      /* rotate */
      Eigen::Quaterniond q;
      tf::quaternionTFToEigen(transforms[i].getRotation(), q);
      Eigen::Matrix3d rotate_m(q);

      /* 1. link base model */
      /* rotate(R_t * I * R) */
      Eigen::Vector3d origin_from_cog = origin_from_root_link - getCog();
      links_origin_from_cog[i] = origin_from_cog;

      Eigen::Matrix3d link_rotated_inertia = rotate_m.transpose() *  link_base_model_[i].getInertia() * rotate_m;

      //offset http://eman-physics.net/dynamics/mom_tensor.html
      Eigen::Matrix3d link_offset_inertia;
      float link_mass = link_base_model_[i].getWeight();

      link_offset_inertia <<
        link_mass * origin_from_cog(1) * origin_from_cog(1),
        link_mass * (-origin_from_cog(0) * origin_from_cog(1)),
        0,
        link_mass * (-origin_from_cog(0) * origin_from_cog(1)),
        link_mass * origin_from_cog(0) * origin_from_cog(0),
        0,
        0,
        0,
        link_mass * (origin_from_cog(0) * origin_from_cog(0) + origin_from_cog(1) * origin_from_cog(1));

      Eigen::Matrix3d links_inertia_tmp = links_inertia;
      links_inertia = links_inertia_tmp + link_rotated_inertia + link_offset_inertia;

      /* 2. link joint model */
      if(i > 0)
        {
          origin_from_cog = rotate_m * link_joint_model_[i - 1].getOffset()  + origin_from_root_link - getCog();

          float joint_mass = link_joint_model_[i - 1].getWeight();
          Eigen::Matrix3d joint_offset_inertia;
          joint_offset_inertia <<
            joint_mass * origin_from_cog(1) * origin_from_cog(1),
            joint_mass * (-origin_from_cog(0) * origin_from_cog(1)),
            0,
            joint_mass * (-origin_from_cog(0) * origin_from_cog(1)),
            joint_mass * origin_from_cog(0) * origin_from_cog(0),
            0,
            0,
            0,
            joint_mass * (origin_from_cog(0) * origin_from_cog(0) + origin_from_cog(1) * origin_from_cog(1));

          Eigen::Matrix3d links_inertia_tmp = links_inertia;
          links_inertia = links_inertia_tmp + joint_offset_inertia;
        }
    }

  /* 3. extra module model */
  for(int i = 0; i < extra_module_num_; i++)
    {
      int link_no = extra_module_model_[i].getLink();

      /* rotate */
      Eigen::Quaterniond q;
      tf::quaternionTFToEigen(transforms[link_no].getRotation(), q);
      Eigen::Matrix3d rotate_m(q);

      /* 1. link base model */
      /* rotate(R_t * I * R) */
      Eigen::Vector3d origin_from_cog = rotate_m * extra_module_model_[i].getOffset() + links_origin_from_cog[link_no];

      float controller_mass = extra_module_model_[i].getWeight();
      Eigen::Matrix3d extra_module_offset_inertia;
      extra_module_offset_inertia <<
        controller_mass * origin_from_cog(1) * origin_from_cog(1),
        controller_mass * (-origin_from_cog(0) * origin_from_cog(1)),
        0,
        controller_mass * (-origin_from_cog(0) * origin_from_cog(1)),
        controller_mass * origin_from_cog(0) * origin_from_cog(0),
        0,
        0,
        0,
        controller_mass * (origin_from_cog(0) * origin_from_cog(0) + origin_from_cog(1) * origin_from_cog(1));

      Eigen::Matrix3d links_inertia_tmp = links_inertia;
      links_inertia = links_inertia_tmp + extra_module_offset_inertia;
    }

  links_inertia_ = links_inertia;


  if(kinematic_verbose_)
     std::cout << "links inertia :\n" << links_inertia_ << std::endl;

  /* pricipal inertia */
  /* eigen solver */
  ros::Time start_time = ros::Time::now();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(links_inertia_);
  Eigen::Matrix3d links_principal_inertia = eig.eigenvalues().asDiagonal();
  Eigen::Matrix3d rotate_matrix = eig.eigenvectors();

  /* the reorder of the inertia and rotate matrix (just for 2D)!! */
  if(sgn(rotate_matrix(0,0)) != sgn(rotate_matrix(1,1)))
    {
      rotate_matrix.col(0).swap(rotate_matrix.col(1));
      links_principal_inertia.col(0).swap(links_principal_inertia.col(1));
      links_principal_inertia.row(0).swap(links_principal_inertia.row(1));
    }

  if(continuous_flag)
    {
      int no = 0;

      if(!init_flag)
        {
          ROS_ERROR("transform control: start");
          setRotateMatrix(rotate_matrix);
          setPrincipalInertia(links_principal_inertia);

          init_flag = true;
        }
      else
        {
          std::vector<Eigen::Matrix3d> rotate_matrix_candidates;
          rotate_matrix_candidates.resize(4);
          std::vector<Eigen::Matrix3d> links_principal_inertia_candidates;
          links_principal_inertia_candidates.resize(4);
          std::vector<float> rotate_matrix_candidates_delta;
          rotate_matrix_candidates_delta.resize(4);
          float  min_delta = 1000000;

          for(int i = 0; i < 4; i++)
            {
              rotate_matrix_candidates[i] = rotate_matrix;
              links_principal_inertia_candidates[i] = links_principal_inertia;

              if(i == 1)
                {
                  Eigen::Vector3d tmp;
                  tmp = -rotate_matrix_candidates[i].col(0);
                  rotate_matrix_candidates[i].col(0) =  tmp;
                  tmp = -rotate_matrix_candidates[i].col(1);
                  rotate_matrix_candidates[i].col(1) =  tmp;
                }
              else if(i == 2)
                {
                  rotate_matrix_candidates[i].col(0).swap(rotate_matrix_candidates[i].col(1));
                  Eigen::Vector3d tmp;
                  tmp = -rotate_matrix_candidates[i].col(0);
                  rotate_matrix_candidates[i].col(0) =  tmp;

                  links_principal_inertia_candidates[i].col(0).swap(links_principal_inertia_candidates[i].col(1));
                  links_principal_inertia_candidates[i].row(0).swap(links_principal_inertia_candidates[i].row(1));

                }
              else if(i == 3)
                {
                  rotate_matrix_candidates[i].col(0).swap(rotate_matrix_candidates[i].col(1));
                  Eigen::Vector3d tmp;
                  tmp = -rotate_matrix_candidates[i].col(1);
                  rotate_matrix_candidates[i].col(1) =  tmp;

                  links_principal_inertia_candidates[i].col(0).swap(links_principal_inertia_candidates[i].col(1));
                  links_principal_inertia_candidates[i].row(0).swap(links_principal_inertia_candidates[i].row(1));
                }


              Eigen::Matrix3d rotate_matrix_candidates_delta_m = rotate_matrix_candidates[i] - getRotateMatrix();
              rotate_matrix_candidates_delta[i] = rotate_matrix_candidates_delta_m.squaredNorm();

              if(rotate_matrix_candidates_delta[i] < min_delta)
                {
                  min_delta = rotate_matrix_candidates_delta[i];
                  no = i;
                }
            }

          setRotateMatrix(rotate_matrix_candidates[no]);
          setPrincipalInertia(links_principal_inertia_candidates[no]);
        }
    }
  else
    {
      setPrincipalInertia(links_principal_inertia);
      setRotateMatrix(rotate_matrix);
    }

  if(kinematic_verbose_)
     std::cout << "pricipal inertia :\n" << getPrincipalInertia() << std::endl;

  /* rotate the link origins from cog */
  cog_matrix_ = getRotateMatrix().transpose();

  std::vector<Eigen::Vector3d > links_origin_from_principal_cog;
  links_origin_from_principal_cog.resize(link_num_);
  for(int i = 0; i < link_num_; i ++)
    {
      links_origin_from_principal_cog[i] = cog_matrix_ * links_origin_from_cog[i];
       if(kinematic_verbose_)
         std::cout << "link" << i + 1 <<"origin :\n" << links_origin_from_principal_cog[i] << std::endl;

    }
  setLinksOriginFromCog(links_origin_from_principal_cog);

  Eigen::Matrix3d rotate_matrix_tmp = getRotateMatrix();
  rotate_angle_ = atan2(rotate_matrix_tmp(1,0), rotate_matrix_tmp(0,0));
}

void TransformController::cogCoordPublish()
{

  tf::TransformBroadcaster br;
  tf::Transform transform;

  Eigen::Vector3d cog = getCog();
  Eigen::Matrix3d rotate_matrix = getRotateMatrix();
  transform.setOrigin( tf::Vector3(cog(0), cog(1), cog(2)));
  Eigen::Quaterniond q_eigen(rotate_matrix);

  tf::Quaternion q_tf;
  tf::quaternionEigenToTF(q_eigen, q_tf);
  transform.setRotation(q_tf);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), root_link_name_, "cog"));

}

void TransformController::visualization()
{
  Eigen::Vector3d cog = getCog();
  Eigen::Matrix3d rotate_matrix = getRotateMatrix();

  visualization_msgs::MarkerArray cog_inertia;
  visualization_msgs::Marker cog_point;
  cog_point.header.frame_id = root_link_name_;
  cog_point.header.stamp = ros::Time::now();
  cog_point.ns = "cog";
  cog_point.id = 0;
  cog_point.type = visualization_msgs::Marker::SPHERE;
  cog_point.action = visualization_msgs::Marker::ADD;
  cog_point.lifetime = ros::Duration();
  cog_point.pose.position.x = cog(0);
  cog_point.pose.position.y = cog(1);
  cog_point.pose.position.z = cog(2);
  cog_point.pose.orientation.x = 0;
  cog_point.pose.orientation.y = 0;
  cog_point.pose.orientation.z = 0;
  cog_point.pose.orientation.w = 1;
  cog_point.scale.x = 0.1;
  cog_point.scale.y = 0.1;
  cog_point.scale.z = 0.1;
  cog_point.color.a = 1.0;
  cog_point.color.r = 1;
  cog_point.color.g = 0.0;
  cog_point.color.b = 0;
  cog_inertia.markers.push_back(cog_point);

  visualization_msgs::Marker inertia_axis_x;
  inertia_axis_x.header.frame_id = root_link_name_;
  inertia_axis_x.header.stamp = ros::Time::now();
  inertia_axis_x.ns = "inertia_x";
  inertia_axis_x.id = 1;
  inertia_axis_x.type = visualization_msgs::Marker::ARROW;
  inertia_axis_x.action = visualization_msgs::Marker::ADD;
  inertia_axis_x.lifetime = ros::Duration();


  inertia_axis_x.pose.position.x = cog(0);
  inertia_axis_x.pose.position.y = cog(1);
  inertia_axis_x.pose.position.z = cog(2);  
  Eigen::Quaterniond q(rotate_matrix);

  inertia_axis_x.pose.orientation.x = q.x();
  inertia_axis_x.pose.orientation.y = q.y();
  inertia_axis_x.pose.orientation.z = q.z();
  inertia_axis_x.pose.orientation.w = q.w();
  inertia_axis_x.scale.x = 1;
  inertia_axis_x.scale.y = 0.05;
  inertia_axis_x.scale.z = 0.05;
  inertia_axis_x.color.a = 1.0;
  inertia_axis_x.color.r = 0.0;
  inertia_axis_x.color.g = 0.0;
  inertia_axis_x.color.b = 1.0;
  cog_inertia.markers.push_back(inertia_axis_x);

  visualization_msgs::Marker inertia_axis_y;
  inertia_axis_y.header.frame_id = root_link_name_;
  inertia_axis_y.header.stamp = ros::Time::now();
  inertia_axis_y.ns = "inertia_y";
  inertia_axis_y.id = 1;
  inertia_axis_y.type = visualization_msgs::Marker::ARROW;
  inertia_axis_y.action = visualization_msgs::Marker::ADD;
  inertia_axis_y.lifetime = ros::Duration();
  inertia_axis_y.pose.position.x = cog(0);
  inertia_axis_y.pose.position.y = cog(1);
  inertia_axis_y.pose.position.z = cog(2);  
  Eigen::Matrix3d rot = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d(0, 0, 1)) * rotate_matrix;
  Eigen::Quaterniond q2(rot);
  inertia_axis_y.pose.orientation.x = q2.x();
  inertia_axis_y.pose.orientation.y = q2.y();
  inertia_axis_y.pose.orientation.z = q2.z();
  inertia_axis_y.pose.orientation.w = q2.w();
  inertia_axis_y.scale.x = 1;
  inertia_axis_y.scale.y = 0.05;
  inertia_axis_y.scale.z = 0.05;
  inertia_axis_y.color.a = 1.0;
  inertia_axis_y.color.r = 0.0;
  inertia_axis_y.color.g = 1.0;
  inertia_axis_y.color.b = 0.0;
  cog_inertia.markers.push_back(inertia_axis_y);

  principal_axis_pub_.publish(cog_inertia);
}

void TransformController::param2contoller()
{
  aerial_robot_base::DesireCoord desire_coord_msg;
  aerial_robot_msgs::FourAxisGain four_axis_gain_msg;
  hydrus_transform_control::RollPitchYawGain rpy_gain_msg;

  desire_coord_msg.roll = 0;
  desire_coord_msg.pitch = 0;
  desire_coord_msg.yaw = -rotate_angle_;  // should be reverse (cog coord is parent)

  four_axis_gain_msg.motor_num = link_num_;

  /* TODO:
     The size of "aerial_robot_msgs::FourAxisGain" and "hydrus_transform_control::RollPitchYawGain" 
     is fixed to be 6.
     So the model with more than 6 links(i.e. hex) is not suitable for this message.
     If we have to fix this problem, we have to change the message structure of those message.
     However, the increase of the elements of those message affects the bandwidth of serial communication
     between pc and fcu.
     Right now, only a hardcoding for the octo model is implemented.
  */
  if(link_num_ > 6) return;

#if 0
  /* temp */
  if (std::signbit(K12_(0,0)) != std::signbit(K12_(5,0)))
    {
      K12_(0,0) = 0;
      K12_(5,0) = 0;
      K12_(0,1) = 0;
      K12_(5,1) = 0;
      K12_(0,8) = 0;
      K12_(5,8) = 0;
    }
  else
    {
      K12_(0,2) = 0;
      K12_(5,2) = 0;
      K12_(0,3) = 0;
      K12_(5,3) = 0;
      K12_(0,9) = 0;
      K12_(5,9) = 0;
    }
#endif

  for(int i = 0; i < link_num_; i ++)
    {
      if(lqi_mode_ == LQI_FOUR_AXIS_MODE)
        {
          /* to flight controller */
          rpy_gain_msg.roll_p_gain[i] = K12_(i,0) * 1000; //scale: x 1000
          rpy_gain_msg.roll_d_gain[i] = K12_(i,1) * 1000;  //scale: x 1000
          rpy_gain_msg.roll_i_gain[i] = K12_(i,8) * 1000; //scale: x 1000

          rpy_gain_msg.pitch_p_gain[i] = K12_(i,2) * 1000; //scale: x 1000
          rpy_gain_msg.pitch_d_gain[i] = K12_(i,3) * 1000; //scale: x 1000
          rpy_gain_msg.pitch_i_gain[i] = K12_(i,9) * 1000; //scale: x 1000

          rpy_gain_msg.yaw_d_gain[i] = K12_(i,5) * 1000; //scale: x 1000

          /* to aerial_robot_base, feedback */
          four_axis_gain_msg.pos_p_gain_roll.push_back(K12_(i,0));
          four_axis_gain_msg.pos_d_gain_roll.push_back(K12_(i,1));
          four_axis_gain_msg.pos_i_gain_roll.push_back(K12_(i,8));

          four_axis_gain_msg.pos_p_gain_pitch.push_back(K12_(i,2));
          four_axis_gain_msg.pos_d_gain_pitch.push_back(K12_(i,3));
          four_axis_gain_msg.pos_i_gain_pitch.push_back(K12_(i,9));

          four_axis_gain_msg.pos_p_gain_throttle.push_back(K12_(i,6));
          four_axis_gain_msg.pos_d_gain_throttle.push_back(K12_(i,7));
          four_axis_gain_msg.pos_i_gain_throttle.push_back(K12_(i,11));

          four_axis_gain_msg.pos_p_gain_yaw.push_back(K12_(i,4));
          four_axis_gain_msg.pos_d_gain_yaw.push_back(K12_(i,5));
          four_axis_gain_msg.pos_i_gain_yaw.push_back(K12_(i,10));

          /* to aerial_robot_base, feedforward */
          four_axis_gain_msg.ff_roll_vec.push_back(-K12_(i,0));
          four_axis_gain_msg.ff_pitch_vec.push_back(-K12_(i,2));
          four_axis_gain_msg.ff_yaw_vec.push_back(-K12_(i,4));
          //four_axis_gain_msg.ff_yaw_vec.push_back(0);

        }
      else if(lqi_mode_ == LQI_THREE_AXIS_MODE)
        {
          rpy_gain_msg.roll_p_gain[i] = K9_(i,0) * 1000; //scale: x 1000
          rpy_gain_msg.roll_d_gain[i] = K9_(i,1) * 1000; //scale: x 1000
          rpy_gain_msg.roll_i_gain[i] = K9_(i,6) * 1000; //scale: x 1000

          rpy_gain_msg.pitch_p_gain[i] = K9_(i,2) * 1000; //scale: x 1000
          rpy_gain_msg.pitch_d_gain[i] = K9_(i,3) * 1000; //scale: x 1000
          rpy_gain_msg.pitch_i_gain[i] = K9_(i,7) * 1000; //scale: x 1000

          rpy_gain_msg.yaw_d_gain[i] = 0;

          /* to aerial_robot_base, feedback */
          four_axis_gain_msg.pos_p_gain_roll.push_back(K9_(i,0));
          four_axis_gain_msg.pos_d_gain_roll.push_back(K9_(i,1));
          four_axis_gain_msg.pos_i_gain_roll.push_back(K9_(i,6));

          four_axis_gain_msg.pos_p_gain_pitch.push_back(K9_(i,2));
          four_axis_gain_msg.pos_d_gain_pitch.push_back(K9_(i,3));
          four_axis_gain_msg.pos_i_gain_pitch.push_back(K9_(i,7));

          four_axis_gain_msg.pos_p_gain_throttle.push_back(K9_(i,4));
          four_axis_gain_msg.pos_d_gain_throttle.push_back(K9_(i,5));
          four_axis_gain_msg.pos_i_gain_throttle.push_back(K9_(i,8));

          four_axis_gain_msg.pos_p_gain_yaw.push_back(0.0);
          four_axis_gain_msg.pos_d_gain_yaw.push_back(0.0);
          four_axis_gain_msg.pos_i_gain_yaw.push_back(0.0);

          //to aerial_robot_base, feedforward
          four_axis_gain_msg.ff_roll_vec.push_back(-K9_(i,0));
          four_axis_gain_msg.ff_pitch_vec.push_back(-K9_(i,2));
          four_axis_gain_msg.ff_yaw_vec.push_back(0);
        }
    }

  cog_rotate_pub_.publish(desire_coord_msg);
  rpy_gain_pub_.publish(rpy_gain_msg);
  four_axis_gain_pub_.publish(four_axis_gain_msg);

}

std::vector<tf::StampedTransform> TransformController::transformsFromJointValues(const std::vector<double>& joint_values, int joint_offset)
{
  //temporary for quad-type
  std::vector<tf::Vector3> origins;
  std::vector<tf::StampedTransform>  transforms;
  transforms.resize(link_num_);

  float theta1 = 0, theta2 = 0;
  tf::Quaternion q;

  int root_link = root_link_ - 1; //minus -1
  transforms[root_link].setOrigin( tf::Vector3(0, 0, 0) );
  q.setRPY(0, 0, 0);
  transforms[root_link].setRotation(q);

  for(int i = root_link - 1; i >= 0; i--)
    {
      theta2 -= joint_values[i+joint_offset];
      transforms[i].setOrigin( tf::Vector3(transforms[i+1].getOrigin().getX() + link_length_ / 2 * cos(theta1) + link_length_ / 2 * cos(theta2),
                                           transforms[i+1].getOrigin().getY() + link_length_ / 2 * sin(theta1) + link_length_ / 2 * sin(theta2),
                                           0) );
      q.setRPY(0, 0, theta2);
      transforms[i].setRotation(q);
      theta1 -= joint_values[i+joint_offset];
    }

  theta1 = 0, theta2 = 0;
  for(int i = root_link + 1; i < link_num_; i ++)
    {
      theta2 += joint_values[i+joint_offset -1];
      transforms[i].setOrigin( tf::Vector3(transforms[i-1].getOrigin().getX() - link_length_ / 2 * cos(theta1) - link_length_ / 2 * cos(theta2),
                                           transforms[i-1].getOrigin().getY() - link_length_ / 2 * sin(theta1) - link_length_ / 2 * sin(theta2),
                                           0) );
      q.setRPY(0, 0, theta2);
      transforms[i].setRotation(q);
      theta1 += joint_values[i+joint_offset -1];
    }


  //for transform from link1

#if 0
  for(int i = 0; i < link_num_; i ++)
    {
      if(i == 0)
        {
          transforms[0].setOrigin( tf::Vector3(0, 0, 0) );

          q.setRPY(0, 0, 0);
          transforms[0].setRotation(q);
        }
      else
        {
          theta2 += joint_values[i+joint_offset -1];
          transforms[i].setOrigin( tf::Vector3(transforms[i-1].getOrigin().getX() - link_length_ / 2 * cos(theta1) - link_length_ / 2 * cos(theta2),
                                               transforms[i-1].getOrigin().getY() - link_length_ / 2 * sin(theta1) - link_length_ / 2 * sin(theta2),
                                               0) );
          q.setRPY(0, 0, theta2);
          transforms[i].setRotation(q);
          theta1 += joint_values[i+joint_offset -1];
        }
    }
#endif

  return transforms;
}

bool TransformController::distThreCheck()
{
  static int i = 0;
  float x_minus_max_dist = 0, x_plus_max_dist = 0;
  float y_minus_max_dist = 0, y_plus_max_dist = 0;

  for(; i < link_num_; i++)
    {

      std::vector<Eigen::Vector3d> links_origin_from_cog(link_num_);
      getLinksOriginFromCog(links_origin_from_cog);
      //x
      if(links_origin_from_cog[i](0) > 0 && links_origin_from_cog[i](0) > x_plus_max_dist)
        x_plus_max_dist = links_origin_from_cog[i](0);
      if(links_origin_from_cog[i](0) < 0 && links_origin_from_cog[i](0) < x_minus_max_dist)
        x_minus_max_dist = links_origin_from_cog[i](0);
      //y
      if(links_origin_from_cog[i](1) > 0 && links_origin_from_cog[i](1) > y_plus_max_dist)
        y_plus_max_dist = links_origin_from_cog[i](1);
      if(links_origin_from_cog[i](1) < 0 && links_origin_from_cog[i](1) < y_minus_max_dist)
        y_minus_max_dist = links_origin_from_cog[i](1);
    }
  i = 0;

  if(x_plus_max_dist < dist_thre_ || -x_minus_max_dist < dist_thre_ ||
     y_plus_max_dist < dist_thre_ || -y_minus_max_dist < dist_thre_ ) //[m]
    {
      return false;
    }
  return true;
}

bool TransformController::distThreCheckFromJointValues(const std::vector<double>& joint_values, int joint_offset, bool continous_flag)
{
  std::vector<tf::StampedTransform> transforms = transformsFromJointValues(joint_values, joint_offset);
  cogComputation(transforms);
  principalInertiaComputation(transforms, continous_flag);
  return distThreCheck();
}

bool  TransformController::stabilityCheck(bool verbose)
{
  std::vector<Eigen::Vector3d> links_origin_from_cog(link_num_);
  getLinksOriginFromCog(links_origin_from_cog);
  Eigen::Matrix3d links_principal_inertia = getPrincipalInertia();

  Eigen::VectorXd x;
  x.resize(link_num_);
  Eigen::VectorXd g(4);
  g << 0, 0, 0, 9.8;

  Eigen::VectorXd p_x(link_num_), p_y(link_num_), p_c(link_num_), p_m(link_num_); 
  static int i = 0;

  for(; i < link_num_; i++)
    {
      int order = propeller_order_[i];
      p_y(order) =  links_origin_from_cog[i](1);
      p_x(order) = -links_origin_from_cog[i](0);
      p_c(order) = propeller_direction_[i] * m_f_rate_ ;
      p_m(order) = 1 / all_mass_;

      if(control_verbose_ || verbose)
        std::cout << "link" << i + 1 <<"origin :\n" << links_origin_from_cog[i] << std::endl;
    }
  i = 0;

  U_.row(0) = p_y / links_principal_inertia(0,0);
  U_.row(1) = p_x / links_principal_inertia(1,1);
  U_.row(2) = p_c / links_principal_inertia(2,2);
  U_.row(3) = p_m;
  if(control_verbose_ || verbose)
    std::cout << "U_:"  << std::endl << U_ << std::endl;

      ros::Time start_time = ros::Time::now();
  if(link_num_ == 4) 
    {//square mothod
      Eigen::FullPivLU<Eigen::MatrixXd> solver(U_);

      x = solver.solve(g);
      if(control_verbose_)
        std::cout << "U det:"  << std::endl << U_.determinant() << std::endl;
    }
  else
    {//lagrange mothod
      // issue: min x_t * x; constraint: g = U_ * x  (stable point)
      //lamda: [4:0]
      // x = U_t * lamba
      // (U_  * U_t) * lamda = g
      // x = U_t * (U_ * U_t).inv * g
      Eigen::FullPivLU<Eigen::MatrixXd> solver((U_ * U_.transpose())); 
      Eigen::VectorXd lamda;
      lamda = solver.solve(g);
      x = U_.transpose() * lamda;

      if(control_verbose_)
        std::cout << "U det:"  << std::endl << (U_ * U_.transpose()).determinant() << std::endl;
    }

  if(control_verbose_)
    ROS_INFO("U solver is: %f\n", ros::Time::now().toSec() - start_time.toSec());

  if(control_verbose_ || verbose)
    std::cout << "x:"  << std::endl << x << std::endl;

  if(x.maxCoeff() > f_max_ || x.minCoeff() < f_min_)
    {
      lqi_mode_ = LQI_THREE_AXIS_MODE;

#if 1
      //no yaw constraint
      Eigen::MatrixXd U_dash = Eigen::MatrixXd::Zero(3, link_num_);
      U_dash.row(0) = U_.row(0);
      U_dash.row(1) = U_.row(1);
      U_dash.row(2) = U_.row(3);
      Eigen::VectorXd g3(3);
      g3 << 0, 0, 9.8;
      Eigen::FullPivLU<Eigen::MatrixXd> solver((U_dash * U_dash.transpose())); 
      Eigen::VectorXd lamda;
      lamda = solver.solve(g3);
      x = U_dash.transpose() * lamda;
      if(control_verbose_)
        std::cout << "x:"  << std::endl << x << std::endl;

#endif

      return false; //can not be stable
    }

  lqi_mode_ = LQI_FOUR_AXIS_MODE;
  return true;
}

void TransformController::lqi()
{
  //check the thre check
  if(debug_verbose_) ROS_WARN(" start dist thre check");
  if(!distThreCheck()) //[m]
    {
      ROS_ERROR("LQI: singular pose, can not resolve the lqi control problem");
      return;
    }
  if(debug_verbose_) ROS_WARN(" finish dist thre check");

  //check the stability within the range of the motor force
  if(debug_verbose_) ROS_WARN(" start stability check");
  if(!stabilityCheck())
    ROS_ERROR("can not be four axis stable, switch to three axis stable mode");
  if(debug_verbose_) ROS_WARN(" finish stability check");

  if(debug_verbose_) ROS_WARN(" start hamilton calc");
  if(!hamiltonMatrixSolver(lqi_mode_))
    {
      ROS_ERROR("LQI: can not solve hamilton matrix");
      return;
    }
  if(debug_verbose_) ROS_WARN(" finish hamilton calc");

  param2contoller();
  if(debug_verbose_) ROS_WARN(" finish param2controller");
}

bool TransformController::hamiltonMatrixSolver(uint8_t lqi_mode)
{
  //for the R which is  diagonal matrix. should be changed to link_num
  Eigen::MatrixXd R_inv  = Eigen::MatrixXd::Zero(link_num_, link_num_);
  for(int i = 0; i < link_num_; i ++)
    R_inv(i,i) = 1/r_[i];

  // hamilton matrix
  if(lqi_mode_ == LQI_FOUR_AXIS_MODE)
    {
      B8_.row(1) = U_.row(0);
      B8_.row(3) = U_.row(1);
      B8_.row(5) = U_.row(2);
      B8_.row(7) = U_.row(3);
      if(control_verbose_)
        std::cout << "B8:"  << std::endl << B8_ << std::endl;

      B12_aug_ = Eigen::MatrixXd::Zero(12, link_num_);
      //B12_aug_.block<8,link_num_>(0,0) = B8_;
      for(int j = 0; j < 8; j++)
        B12_aug_.row(j) = B8_.row(j);


      Eigen::MatrixXcd H = Eigen::MatrixXcd::Zero(24,24); //all right?
      H.block<12,12>(0,0) = A12_aug_.cast<std::complex<double> >();

      H.block<12,12>(12,0) = -(Q12_.cast<std::complex<double> >());
      H.block<12,12>(0,12) = - (B12_aug_ * R_inv * B12_aug_.transpose()).cast<std::complex<double> >();
      H.block<12,12>(12,12) = - (A12_aug_.transpose()).cast<std::complex<double> >();

      //std::cout << " H  is:" << std::endl << H << std::endl;
      if(debug_verbose_) ROS_INFO("  start H eigen compute");
      //eigen solving
      ros::Time start_time = ros::Time::now();
      Eigen::ComplexEigenSolver<Eigen::MatrixXcd> ces;
      ces.compute(H);
      if(debug_verbose_) ROS_INFO("  finish H eigen compute");

      if(control_verbose_)
        ROS_INFO("h eigen time is: %f\n", ros::Time::now().toSec() - start_time.toSec());

      Eigen::MatrixXcd phy = Eigen::MatrixXcd::Zero(24,12);
      int j = 0;


      for(int i = 0; i < 24; i++)
        {
          if(ces.eigenvalues()[i].real() < 0)
            {
              if(j > 11)
                {
                  ROS_ERROR("nagativa sigular amount is larger");
                  return false;
                }

              phy.col(j) = ces.eigenvectors().col(i);
              j++;
            }

        }

      if(j != 12)
        {
          ROS_ERROR("nagativa sigular value amount is not enough");
          return false;
        }

      Eigen::MatrixXcd f = phy.block<12,12>(0,0);
      Eigen::MatrixXcd g = phy.block<12,12>(12,0);


      if(debug_verbose_) ROS_INFO("  start calculate f inv");
      start_time = ros::Time::now();
      Eigen::MatrixXcd f_inv  = f.inverse();
      if(control_verbose_)
        ROS_INFO("f inverse: %f\n", ros::Time::now().toSec() - start_time.toSec());

      if(debug_verbose_) ROS_INFO("  finish calculate f inv");

      Eigen::MatrixXcd P = g * f_inv;

      //K
      K12_ = -R_inv * B12_aug_.transpose() * P.real();

      if(control_verbose_)
        std::cout << "K is:" << std::endl << K12_ << std::endl;

      if(a_dash_eigen_calc_flag_)
        {
          if(debug_verbose_) ROS_INFO("  start A eigen compute");
          //check the eigen of new A
          Eigen::MatrixXd A12_dash = Eigen::MatrixXd::Zero(12, 12);
          A12_dash = A12_aug_ + B12_aug_ * K12_;
          // start_time = ros::Time::now();
          Eigen::EigenSolver<Eigen::MatrixXd> esa(A12_dash);
          if(debug_verbose_) ROS_INFO("  finish A eigen compute");
          if(control_verbose_)
            std::cout << "The eigenvalues of A_hash are:" << std::endl << esa.eigenvalues() << std::endl;
        }
    }
  
  if(lqi_mode_ == LQI_THREE_AXIS_MODE)
    {
      B6_.row(1) = U_.row(0);
      B6_.row(3) = U_.row(1);
      B6_.row(5) = U_.row(3);
      if(control_verbose_)
        std::cout << "B6:"  << std::endl << B6_ << std::endl;

      B9_aug_ = Eigen::MatrixXd::Zero(9, link_num_);
      //B9_aug_.block<6,4>(0,0) = B6_;
      for(int j = 0; j < 6; j++)
        B9_aug_.row(j) = B6_.row(j);


      Eigen::MatrixXcd H = Eigen::MatrixXcd::Zero(18,18); //all right?
      H.block<9,9>(0,0) = A9_aug_.cast<std::complex<double> >();

      H.block<9,9>(9,0) = -(Q9_.cast<std::complex<double> >());
      H.block<9,9>(0,9) = - (B9_aug_ * R_inv * B9_aug_.transpose()).cast<std::complex<double> >();
      H.block<9,9>(9,9) = - (A9_aug_.transpose()).cast<std::complex<double> >();

      //eigen solving
      ros::Time start_time = ros::Time::now();
      Eigen::ComplexEigenSolver<Eigen::MatrixXcd> ces;
      ces.compute(H);

      if(control_verbose_)
        ROS_INFO("h eigen time is: %f\n", ros::Time::now().toSec() - start_time.toSec());

      Eigen::MatrixXcd phy = Eigen::MatrixXcd::Zero(18,9);
      int j = 0;


      for(int i = 0; i < 18; i++)
        {
          if(ces.eigenvalues()[i].real() < 0)
            {
              if(j > 8) 
                {
                  ROS_ERROR("nagativa sigular amount is larger");
                  return false;
                }

              phy.col(j) = ces.eigenvectors().col(i);
              j++;
            }

        }

      if(j != 9)
        {
          ROS_ERROR("nagativa sigular value amount is not enough");
          return false;
        }

      Eigen::MatrixXcd f = phy.block<9,9>(0,0);
      Eigen::MatrixXcd g = phy.block<9,9>(9,0);

      start_time = ros::Time::now();
      Eigen::MatrixXcd f_inv  = f.inverse();
      if(control_verbose_)
        ROS_INFO("f inverse: %f\n", ros::Time::now().toSec() - start_time.toSec());

      Eigen::MatrixXcd P = g * f_inv;

      //K
      K9_ = -R_inv * B9_aug_.transpose() * P.real();

      if(control_verbose_)
        std::cout << "K is:" << std::endl << K9_ << std::endl;

      if(a_dash_eigen_calc_flag_)
        {
          //check the eigen of new A
          Eigen::MatrixXd A9_dash = Eigen::MatrixXd::Zero(9, 9);
          A9_dash = A9_aug_ + B9_aug_ * K9_;
          // start_time = ros::Time::now();
          Eigen::EigenSolver<Eigen::MatrixXd> esa(A9_dash);
          if(control_verbose_)
            std::cout << "The eigenvalues of A_hash are:" << std::endl << esa.eigenvalues() << std::endl;
        }
    }
  return true;
}

void TransformController::yawGainCallback(const std_msgs::UInt8ConstPtr & msg)
{
  if(msg->data == 1)
    {/* change to strong one */
      Q12_(4,4) = strong_q_yaw_;
    }
  else
    {/* back to week one */
      Q12_(4,4) = q_yaw_;
    }
  std::cout << "Q12:"  << std::endl << Q12_ << std::endl;
}


void TransformController::cfgLQICallback(hydrus_transform_control::LQIConfig &config, uint32_t level)
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
      Eigen::VectorXd  q12_diagonal(12);
      q12_diagonal << q_roll_,q_roll_d_,q_pitch_,q_pitch_d_,q_yaw_,q_yaw_d_,q_z_,q_z_d_, q_roll_i_,q_pitch_i_,q_yaw_i_,q_z_i_;
      Q12_ = q12_diagonal.asDiagonal();
    }
}

