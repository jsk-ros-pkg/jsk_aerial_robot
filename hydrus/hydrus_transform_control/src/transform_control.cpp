#include <hydrus_transform_control/transform_control.h>

TransformController::TransformController(ros::NodeHandle nh, ros::NodeHandle nh_private, bool callback_flag):
  nh_(nh),nh_private_(nh_private)
{
  initParam();

  realtime_control_flag_ = true;
  callback_flag_ = callback_flag;

  cog_(0) = 0;
  cog_(1) = 0;
  cog_(2) = 0;

  links_origin_from_cog_.resize(link_num_);

  //U
  P_ = Eigen::MatrixXd::Zero(4,link_num_);

  //Q
  q_diagonal_ = Eigen::VectorXd::Zero(LQI_FOUR_AXIS_MODE * 3);
  q_diagonal_ << q_roll_,q_roll_d_,q_pitch_,q_pitch_d_,q_z_,q_z_d_,q_yaw_,q_yaw_d_, q_roll_i_,q_pitch_i_,q_z_i_,q_yaw_i_;
  //q9_diagonal << q_roll_,q_roll_d_,q_pitch_,q_pitch_d_,q_z_,q_z_d_, q_roll_i_,q_pitch_i_, q_z_i_;

  std::cout << "Q elements :"  << std::endl << q_diagonal_ << std::endl;

  lqi_mode_ = LQI_FOUR_AXIS_MODE;

  //those publisher is published from func param2controller
  rpy_gain_pub_ = nh_.advertise<aerial_robot_msgs::RollPitchYawTerms>(rpy_gain_pub_name_, 1);
  four_axis_gain_pub_ = nh_.advertise<aerial_robot_msgs::FourAxisGain>("four_axis_gain", 1);

  //dynamic reconfigure server
  lqi_server_ = new dynamic_reconfigure::Server<hydrus_transform_control::LQIConfig>(nh_private_);
  dynamic_reconf_func_lqi_ = boost::bind(&TransformController::cfgLQICallback, this, _1, _2);
  lqi_server_->setCallback(dynamic_reconf_func_lqi_);

  if(callback_flag_)
    {// the name callback flag is not correct
      realtime_control_sub_ = nh_.subscribe<std_msgs::UInt8>("realtime_control", 1, &TransformController::realtimeControlCallback, this, ros::TransportHints().tcpNoDelay());

      principal_axis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("orientation_data", 1);

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

  nh_private_.param("only_three_axis_mode", only_three_axis_mode_, false);
  nh_private_.param("control_verbose", control_verbose_, false);
  nh_private_.param("kinematic_verbose", kinematic_verbose_, false);
  nh_private_.param("debug_verbose", debug_verbose_, false);
  nh_private_.param("a_dash_eigen_calc_flag", a_dash_eigen_calc_flag_, false);


  /* number of links */
  nh_private_.param("link_num", link_num_, 4);
  nh_private_.param("joint_num", joint_num_, link_num_ - 1);
  std::cout << " link num: " << link_num_ << std::endl;
  std::cout << " joint num: " << joint_num_ << std::endl;

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
  rotor_direction_.resize(link_num_);
  link_base_model_.resize(link_num_);
  link_base_rod_mass_.resize(link_num_);
  link_joint_model_.resize(link_num_ -1);
  all_mass_ = 0;

  /* root link */
  nh_private_.param("root_link", root_link_, 3);
  std::stringstream ss;
  ss << root_link_;
  root_link_name_ = std::string("/link") + ss.str() + std::string("_center");
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
  nh_private_.param ("link_base_rod_common_mass", link_base_rod_common_mass_, 0.1);
  std::cout << "link_base_rod_common_mass: " << std::setprecision(3) << link_base_rod_common_mass_ << std::endl;
  nh_private_.param ("link_base_ring_mass", link_base_ring_mass_, 0.048);
  std::cout << "link_base_ring_mass: " << std::setprecision(3) << link_base_ring_mass_ << std::endl;
  nh_private_.param ("link_base_two_ring_holder_mass", link_base_two_ring_holder_mass_, 0.053);
  std::cout << "link_base_two_ring_holder_mass: " << std::setprecision(3) << link_base_two_ring_holder_mass_ << std::endl;
  nh_private_.param ("link_base_center_mass", link_base_center_mass_, 0.238);
  std::cout << "link_base_center_mass: " << std::setprecision(3) << link_base_center_mass_ << std::endl;
  nh_private_.param ("link_joint_mass", link_joint_mass_,0.0785);
  std::cout << "link_joint_mass: " << std::setprecision(3) << link_joint_mass_ << std::endl;

  Eigen::Matrix3d zero_inertia = Eigen::Matrix3d::Zero(3,3);

  /* 1. link base model */
  for(int i = 0; i < link_num_; i++)
    {
      std::stringstream ss;
      ss << i + 1;
      links_name_[i] = std::string("/link") + ss.str() + std::string("_center"); //link name

      nh_private_.param(std::string("link") + ss.str() + std::string("_base_rod_mass"), link_base_rod_mass_[i], link_base_rod_common_mass_);
      std::cout << "link " << i + 1 << "_base_rod_mass:" << std::setprecision(3) << link_base_rod_mass_[i] << std::endl;

      /* model */

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

      /* propeller */
      nh_private_.param(std::string("rotor") + ss.str() + std::string("_direction"), rotor_direction_[i], 1);
    }

  /* 2. link joint model */
  for(int j = 0; j < joint_num_; j++)
    {
      int joint_base_link;
      std::stringstream ss;
      ss << j + 1;
      nh_private_.param (std::string("joint") + ss.str() + std::string("_base_link"), joint_base_link, j);
      std::cout << std::string("joint") + ss.str() + std::string("_base_link: ") << joint_base_link << std::endl;

      Eigen::Vector3d offset_;
      offset_ << link_joint_offset_, 0, 0;
      ElementModel link_joint_model(joint_base_link, link_joint_mass_, zero_inertia, offset_);
      link_joint_model_[j] = link_joint_model;
      all_mass_ += link_joint_model.getWeight();
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
      ElementModel extra_module_model(extra_module_link, extra_module_mass, extra_module_i_diagnoal.asDiagonal(), offset);
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

  if(!realtime_control_flag_) return;

  while(ros::ok())
    {
      /* Kinematics calculation */
      if(debug_verbose_) ROS_ERROR("start kinematics");
      bool get_kinematics = kinematics();
      if(debug_verbose_) ROS_ERROR("finish kinematics");
      /* LQI parameter calculation */
      if(debug_verbose_) ROS_ERROR("start lqi");
      if(get_kinematics) lqi();
      if(debug_verbose_) ROS_ERROR("finish lqi");

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

      inertiaParams(transforms);
      return true;
    }
  return false;
}

void TransformController::inertiaParams(std::vector<tf::StampedTransform>  transforms)
{
  /* calculate the cog */
  Eigen::Vector3d cog_n  = Eigen::Vector3d::Zero();

  /* 1. link base model */
  for(int i = 0; i < link_num_; i++)
    {
      Eigen::Vector3d link_origin;
      tf::vectorTFToEigen(transforms[i].getOrigin(), link_origin);

      Eigen::Quaterniond q;
      tf::quaternionTFToEigen(transforms[i].getRotation(), q);
      Eigen::Matrix3d link_rotate(q);


      cog_n += link_origin * link_base_model_[i].getWeight();
    }

  /* 2. link joint model */
  for(int j = 0; j < joint_num_; j++)
    {
      int link_no = link_joint_model_[j].getLink();
      Eigen::Vector3d link_origin;
      tf::vectorTFToEigen(transforms[link_no].getOrigin(), link_origin);

      Eigen::Quaterniond q;
      tf::quaternionTFToEigen(transforms[link_no].getRotation(), q);
      Eigen::Matrix3d link_rotate(q);

      Eigen::Vector3d joint_origin = link_rotate * link_joint_model_[j].getOffset()  + link_origin;
      cog_n += joint_origin * link_joint_model_[j].getWeight();

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

  /* cog calculate */
  Eigen::Vector3d cog = cog_n / all_mass_;
  setCog(cog);

  Eigen::Matrix3d links_inertia = Eigen::Matrix3d::Zero(3,3);
  for(int i = 0; i < link_num_; i ++)
    {
      /* offset */
      Eigen::Vector3d origin_from_root_link;
      tf::vectorTFToEigen(transforms[i].getOrigin(), origin_from_root_link);

      /* rotate */
      Eigen::Quaterniond q;
      tf::quaternionTFToEigen(transforms[i].getRotation(), q);
      Eigen::Matrix3d rotate_m(q);

      Eigen::Vector3d origin_from_cog = origin_from_root_link - getCog();
      links_origin_from_cog_[i] = origin_from_cog;

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
    }


  /* 2. link joint model */
  for(int j = 0; j < joint_num_; j ++)
    {
      int link_no = link_joint_model_[j].getLink();

      /* rotate */
      Eigen::Quaterniond q;
      tf::quaternionTFToEigen(transforms[link_no].getRotation(), q);
      Eigen::Matrix3d rotate_m(q);

      Eigen::Vector3d origin_from_cog = rotate_m * link_joint_model_[j].getOffset() + links_origin_from_cog_[link_no];

      float joint_mass = link_joint_model_[j].getWeight();
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

  /* 3. extra module model */
  for(int i = 0; i < extra_module_num_; i++)
    {
      int link_no = extra_module_model_[i].getLink();

      /* rotate */
      Eigen::Quaterniond q;
      tf::quaternionTFToEigen(transforms[link_no].getRotation(), q);
      Eigen::Matrix3d rotate_m(q);

      Eigen::Vector3d origin_from_cog = rotate_m * extra_module_model_[i].getOffset() + links_origin_from_cog_[link_no];

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

  tf::Transform cog_transform;
  cog_transform.setOrigin(tf::Vector3(cog(0), cog(1), cog(2)));
  cog_transform.setRotation(tf::Quaternion(0, 0, 0, 1));
  br_.sendTransform(tf::StampedTransform(cog_transform, transforms[0].stamp_, root_link_name_, "cog"));

  if(debug_verbose_) ROS_WARN(" finish kinematic compute");
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

void TransformController::param2contoller()
{
  aerial_robot_msgs::FourAxisGain four_axis_gain_msg;
  aerial_robot_msgs::RollPitchYawTerms rpy_gain_msg; //for rosserial

  four_axis_gain_msg.motor_num = link_num_;
  rpy_gain_msg.motors.resize(link_num_);

  for(int i = 0; i < link_num_; i ++)
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

      four_axis_gain_msg.pos_p_gain_throttle.push_back(K_(i,4));
      four_axis_gain_msg.pos_d_gain_throttle.push_back(K_(i,5));
      four_axis_gain_msg.pos_i_gain_throttle.push_back(K_(i, lqi_mode_ * 2 + 2));

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
  inertiaParams(transforms);
  return distThreCheck();
}

bool  TransformController::stabilityCheck(bool verbose)
{
  std::vector<Eigen::Vector3d> links_origin_from_cog(link_num_);
  getLinksOriginFromCog(links_origin_from_cog);

  Eigen::VectorXd x;
  x.resize(link_num_);
  Eigen::VectorXd g(4);
  g << 0, 0, 9.8, 0;

  Eigen::VectorXd p_x(link_num_), p_y(link_num_), p_c(link_num_), p_m(link_num_);

  for(int i = 0; i < link_num_; i++)
    {
      p_y(i) =  links_origin_from_cog[i](1);
      p_x(i) = -links_origin_from_cog[i](0);
      p_c(i) = rotor_direction_[i] * m_f_rate_ ;
      p_m(i) = 1 / all_mass_;

      if(control_verbose_ || verbose)
        std::cout << "link" << i + 1 <<"origin :\n" << links_origin_from_cog[i] << std::endl;
    }

  Eigen::MatrixXd P_att = Eigen::MatrixXd::Zero(3,link_num_);
  P_att.row(0) = p_y;
  P_att.row(1) = p_x;
  P_att.row(2) = p_c;

  Eigen::MatrixXd P_att_tmp = P_att;
  P_att = links_inertia_.inverse() * P_att_tmp;
  if(control_verbose_)
    std::cout << "links_inertia_.inverse():"  << std::endl << links_inertia_.inverse() << std::endl;

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

  if(control_verbose_)
    std::cout << "P det:"  << std::endl << (P_ * P_.transpose()).determinant() << std::endl;

  if(control_verbose_)
    ROS_INFO("P solver is: %f\n", ros::Time::now().toSec() - start_time.toSec());

  if(control_verbose_ || verbose)
    std::cout << "x:"  << std::endl << x << std::endl;

  if(x.maxCoeff() > f_max_ || x.minCoeff() < f_min_)
    {
      lqi_mode_ = LQI_THREE_AXIS_MODE;

      //no yaw constraint
      Eigen::MatrixXd P_dash = Eigen::MatrixXd::Zero(3, link_num_);
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

  if(only_three_axis_mode_)lqi_mode_ = LQI_THREE_AXIS_MODE;
  else lqi_mode_ = LQI_FOUR_AXIS_MODE;

  return true;
}

void TransformController::lqi()
{
  //check the thre check
  if(debug_verbose_) ROS_WARN(" start dist thre check");
  if(!distThreCheck()) //[m]
    {
      ROS_ERROR("LQI: invalid pose, can pass the distance thresh check");
      return;
    }
  if(debug_verbose_) ROS_WARN(" finish dist thre check");

  //check the stability within the range of the motor force
  if(debug_verbose_) ROS_WARN(" start stability check");
  if(!stabilityCheck())
    {
      if(!only_three_axis_mode_)  ROS_ERROR("LQI: invalid pose, can not be four axis stable, switch to three axis stable mode");
    }
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
  /* for the R which is  diagonal matrix. should be changed to link_num */

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(lqi_mode_ * 3, lqi_mode_ * 3);
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(lqi_mode_ * 3, link_num_);
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

  Eigen::MatrixXd R_inv  = Eigen::MatrixXd::Zero(link_num_, link_num_);
  for(int i = 0; i < link_num_; i ++)
    R_inv(i,i) = 1/r_[i];

  // B12_aug_ = Eigen::MatrixXd::Zero(12, link_num_);
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
      q_diagonal_ << q_roll_,q_roll_d_,q_pitch_,q_pitch_d_,q_yaw_,q_yaw_d_,q_z_,q_z_d_, q_roll_i_,q_pitch_i_,q_yaw_i_,q_z_i_;
    }
}
