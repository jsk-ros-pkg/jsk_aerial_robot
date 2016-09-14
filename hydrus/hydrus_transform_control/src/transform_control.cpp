/***
1. the model of quad-rotor is too wrong, especially the position from propeller to joint
 ***/

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

  //lqi
  lqi_flag_ = false;

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
  rpy_gain_pub_ = nh_.advertise<aerial_robot_msgs::RollPitchYawGain>(rpy_gain_pub_name_, 1);
  yaw_throttle_gain_pub_ = nh_.advertise<aerial_robot_msgs::YawThrottleGain>("yaw_throttle_gain", 1);

  yaw_gain_sub_ = nh_.subscribe<std_msgs::UInt8>(yaw_pos_gain_sub_name_, 1, &TransformController::yawGainCallback, this, ros::TransportHints().tcpNoDelay());

  if(callback_flag_)
    {// the name callback flag is not correct
      realtime_control_sub_ = nh_.subscribe<std_msgs::UInt8>("realtime_control", 1, &TransformController::realtimeControlCallback, this, ros::TransportHints().tcpNoDelay());

      principal_axis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("orientation_data", 1);

      cog_rotate_pub_ = nh_.advertise<aerial_robot_base::DesireCoord>("/desire_coordinate", 1); //absolute

      control_timer_ = nh_private_.createTimer(ros::Duration(1.0 / control_rate_),
                                               &TransformController::controlFunc, this);

      tf_pub_timer_ = nh_private_.createTimer(ros::Duration(1.0 / tf_pub_rate_),
                                              &TransformController::tfPubFunc, this);

      lqi_thread_ = boost::thread(boost::bind(&TransformController::lqi, this));
    }

}
TransformController::~TransformController()
{
  if(callback_flag_)
    {
      lqi_thread_.interrupt();
      lqi_thread_.join();
    }
}

void TransformController::realtimeControlCallback(const std_msgs::UInt8ConstPtr & msg)
{
  if(msg->data == 1)
    {
      ROS_WARN("start realtime control");
      realtime_control_flag_ = true;
    }
  else if(msg->data == 0)
    {
      ROS_WARN("stop realtime control");
      realtime_control_flag_ = false;
      lqi_flag_ = false;
    }
}

void TransformController::initParam()
{
  nh_private_.param("control_rate", control_rate_, 20.0);
  nh_private_.param("tf_pub_rate", tf_pub_rate_, 60.0);

  nh_private_.param("rpy_gain_pub_name", rpy_gain_pub_name_, std::string("/rpy_gain"));
  nh_private_.param("yaw_pos_gain_sub_name", yaw_pos_gain_sub_name_, std::string("/yaw_pos_gain"));


  nh_private_.param("debug_log", debug_log_, false); 
  nh_private_.param("debug2_log", debug2_log_, false); 


  nh_private_.param("link_num", link_num_, 4);
  links_name_.resize(link_num_); 
  propeller_direction_.resize(link_num_);
  propeller_order_.resize(link_num_);
  link_base_model_.resize(link_num_);
  link_base_rod_mass_.resize(link_num_);
  link_joint_model_.resize(link_num_ -1);
  controller_model_.resize(2); //controller1 + controller2
  all_mass_ = 0;


  nh_private_.param("root_link", root_link_, 3);
  std::stringstream ss;
  ss << root_link_;
  root_link_name_ = std::string("/link") + ss.str();
  printf(" root_link_name_ is %s\n", root_link_name_.c_str());

  //position(m)
  if (!nh_private_.getParam ("link_length", link_length_))
    link_length_ = 0.44;
  printf("link_length_ is %.3f\n", link_length_);
  if (!nh_private_.getParam ("link_base_rod_length", link_base_rod_length_))
    link_base_rod_length_ = 0.4;
  printf("link_base_rod_length_ is %.3f\n", link_base_rod_length_);
  if (!nh_private_.getParam ("link_base_two_end_offset", link_base_two_end_offset_))
    link_base_two_end_offset_ = 0.1425; //
  printf("link_base_two_end_offset_ is %.3f\n", link_base_two_end_offset_);
  if (!nh_private_.getParam ("link_joint_offset", link_joint_offset_))
    link_joint_offset_ = 0.22;
  printf("link_joint_offset_ is %.3f\n", link_joint_offset_);
  if (!nh_private_.getParam ("controller1_offset", controller1_offset_))
    controller1_offset_ = -0.166; //please check
  printf("controller1_offset_ is %.3f\n", controller1_offset_);
  if (!nh_private_.getParam ("controller2_offset", controller2_offset_))
    controller2_offset_ = 0.178;
  printf("controller2_offset_ is %.3f\n", controller2_offset_);
  if (!nh_private_.getParam ("controller2_link", controller2_link_))
    controller2_link_ = 0;
  printf("controller2_link_ is %d\n", controller2_link_);
  if (!nh_private_.getParam ("controller1_link", controller1_link_))
    controller1_link_ = 1;
  printf("controller1_link_ is %d\n", controller1_link_);

  //mass(Kg)
 if (!nh_private_.getParam ("link_base_rod_mid_mass", link_base_rod_mid_mass_))
    link_base_rod_mid_mass_ = 0.1;
  printf("link_base_rod_mid_mass_ is %.3f\n", link_base_rod_mid_mass_);
 if (!nh_private_.getParam ("link_base_ring_mass", link_base_ring_mass_))
    link_base_ring_mass_ = 0.048;
  printf("link_base_ring_mass_ is %.3f\n", link_base_ring_mass_);
 if (!nh_private_.getParam ("link_base_two_end_mass", link_base_two_end_mass_))
   link_base_two_end_mass_ = 0.053; //0.0265 * 2
  printf("link_base_two_end_mass_ is %.3f\n", link_base_two_end_mass_);
 if (!nh_private_.getParam ("link_base_center_mass", link_base_center_mass_))
   link_base_center_mass_ = 0.238; // 
  printf("link_base_center_mass_ is %.3f\n", link_base_center_mass_);
 if (!nh_private_.getParam ("link_joint_mass", link_joint_mass_))
    link_joint_mass_ = 0.0785;
  printf("link_joint_mass_ is %.3f\n", link_joint_mass_);
 if (!nh_private_.getParam ("controller1_mass", controller1_mass_))
    controller1_mass_ = 0.022;
  printf("controller1_mass_ is %.3f\n", controller1_mass_);
 if (!nh_private_.getParam ("controller2_mass", controller2_mass_))
    controller2_mass_ = 0.093;
  printf("controller2_mass_ is %.3f\n", controller2_mass_);

 if (!nh_private_.getParam ("dist_thre", dist_thre_))
    dist_thre_ = 0.05;
  printf("dist_thre_ is %.3f\n", dist_thre_);

 if (!nh_private_.getParam ("f_max", f_max_))
    f_max_ = 8.6;
  printf("f_max_ is %.3f\n", f_max_);

 if (!nh_private_.getParam ("f_min", f_min_))
   f_min_ = 2.0;
  printf("f_min_ is %.3f\n", f_min_);

  //lqi
  r_.resize(link_num_);

  Eigen::Matrix3d zero_inertia = Eigen::Matrix3d::Zero(3,3);
  for(int i = 0; i < link_num_; i++)
    {

      std::stringstream ss;
      ss << i + 1;
      links_name_[i] = std::string("/link") + ss.str(); //link name


      if (!nh_private_.getParam (std::string("link") + ss.str() + std::string("_base_rod_mass"), link_base_rod_mass_[i]))
        link_base_rod_mass_[i] = link_base_rod_mid_mass_;
      printf("link%d_base_rod_mass_ is %.3f\n", i+1, link_base_rod_mass_[i]);

      //model
      /// 1. link base model
      float link_base_model_weight = link_base_rod_mass_[i] + link_base_ring_mass_ + link_base_two_end_mass_ + link_base_center_mass_;

      Eigen::Matrix3d link_base_model_inertia;
      //ring model + two_end model + rod model
      link_base_model_inertia << 
        link_base_ring_mass_ / 2 * link_base_two_end_offset_ * link_base_two_end_offset_, 0, 0, 
         0, (link_base_ring_mass_/2 + link_base_two_end_mass_) * link_base_two_end_offset_ * link_base_two_end_offset_ + link_base_rod_mass_[i] * link_base_rod_length_ * link_base_rod_length_ / 12, 0,
         0, 0, (link_base_ring_mass_ + link_base_two_end_mass_) * link_base_two_end_offset_ * link_base_two_end_offset_ + link_base_rod_mass_[i] * link_base_rod_length_ * link_base_rod_length_ / 12;
      //std::cout << "link base model inertia" << link_base_model_inertia << std::endl;

      ElementModel link_base_model(nh_, nh_private_, link_base_model_weight, link_base_model_inertia);
      link_base_model_[i] = link_base_model;
      all_mass_ += link_base_model.getWeight();

      /// 2. link joint model
      if(i+1 < link_num_)
        {
          Eigen::Vector3d offset_;
          offset_ << link_joint_offset_, 0, 0;
          ElementModel link_joint_model(nh_, nh_private_, link_joint_mass_, zero_inertia, offset_);
          link_joint_model_[i] = link_joint_model;
          all_mass_ += link_joint_model.getWeight();
        }

      //propeller
      nh_private_.param(std::string("propeller") + ss.str() + std::string("_direction"), propeller_direction_[i], 1);
      nh_private_.param(std::string("propeller") + ss.str() + std::string("_order"), propeller_order_[i], i);


      //lqi
      if (!nh_private_.getParam (std::string("r") + ss.str(), r_[i]))
        r_[i] = 1.0;
      printf("R: r_%d is %.3f\n", i, r_[i]);
    }


  /// 3. controller
  Eigen::Vector3d offset_;
  offset_ << controller1_offset_, 0, 0;
  ElementModel controller1_model(nh_, nh_private_, controller1_mass_, zero_inertia, offset_);
  all_mass_ += controller1_model.getWeight();
  controller_model_[0] = controller1_model;

  offset_ << controller2_offset_, 0, 0;
  ElementModel controller2_model(nh_, nh_private_, controller2_mass_, zero_inertia, offset_);
  all_mass_ += controller2_model.getWeight();
  controller_model_[1] = controller2_model;

  ROS_INFO("Mass :%f", all_mass_);

  //lqi
  if (!nh_private_.getParam ("lqi_thread_rate", lqi_thread_rate_))
    lqi_thread_rate_ = 10.0;
  printf("lqi_thread_rate_ is %.3f\n", lqi_thread_rate_);
  if (!nh_private_.getParam ("q_roll", q_roll_))
    q_roll_ = 1.0;
  printf("Q: q_roll_ is %.3f\n", q_roll_);
  if (!nh_private_.getParam ("q_roll_d", q_roll_d_))
    q_roll_d_ = 1.0;
  printf("Q: q_roll_d_ is %.3f\n", q_roll_d_);
  if (!nh_private_.getParam ("q_pitch", q_pitch_))
    q_pitch_ = 1.0;
  printf("Q: q_pitch_ is %.3f\n", q_pitch_);
  if (!nh_private_.getParam ("q_pitch_d", q_pitch_d_))
    q_pitch_d_ = 1.0;
  printf("Q: q_pitch_d_ is %.3f\n", q_pitch_d_);
  if (!nh_private_.getParam ("q_yaw", q_yaw_))
    q_yaw_ = 1.0;
  printf("Q: q_yaw_ is %.3f\n", q_yaw_);
  if (!nh_private_.getParam ("strong_q_yaw", strong_q_yaw_))
    strong_q_yaw_ = 1.0;
  printf("Q: strong_q_yaw_ is %.3f\n", strong_q_yaw_);
  if (!nh_private_.getParam ("q_yaw_d", q_yaw_d_))
    q_yaw_d_ = 1.0;
  printf("Q: q_yaw_d_ is %.3f\n", q_yaw_d_);
  if (!nh_private_.getParam ("q_z", q_z_))
    q_z_ = 1.0;
  printf("Q: q_z_ is %.3f\n", q_z_);
  if (!nh_private_.getParam ("q_z_d", q_z_d_))
    q_z_d_ = 1.0;
  printf("Q: q_z_d_ is %.3f\n", q_z_d_);

  if (!nh_private_.getParam ("q_roll_i", q_roll_i_))
    q_roll_i_ = 1.0;
  printf("Q: q_roll_i_ is %.3f\n", q_roll_i_);
  if (!nh_private_.getParam ("q_pitch_i", q_pitch_i_))
    q_pitch_i_ = 1.0;
  printf("Q: q_pitch_i_ is %.3f\n", q_pitch_i_);
  if (!nh_private_.getParam ("q_yaw_i", q_yaw_i_))
    q_yaw_i_ = 1.0;
  printf("Q: q_yaw_i_ is %.3f\n", q_yaw_i_);
  if (!nh_private_.getParam ("q_z_i", q_z_i_))
    q_z_i_ = 1.0;
  printf("Q: q_z_i_ is %.3f\n", q_z_i_);


  /* TODO: shoudl be deprecated */
  // if (!nh_private_.getParam ("alfa", alfa_))
  //   alfa_ = 0.0;
  // printf("alfa_ is %.3f\n", alfa_);

  //dynamics
  ros::NodeHandle control_node("/motor_info");
  if (!control_node.getParam ("pwm_rate", pwm_rate_))
    pwm_rate_ = 1.0;
  printf("pwm_rate_ is %f\n", pwm_rate_);
  if (!control_node.getParam ("m_f_rate", m_f_rate_))
    m_f_rate_ = 0.01; //-0.016837; //the sgn is right?, should be nagative
  printf("m_f_rate_ is %.3f\n",m_f_rate_);
  if (!control_node.getParam ("f_pwm_rate", f_pwm_rate_))
    f_pwm_rate_ = 1.0; //0.3029; // with the pwm percentage: x / 1800 * 100
  printf("f_pwm_rate_ is %.3f\n",f_pwm_rate_);
  if (!control_node.getParam ("f_pwm_offset", f_pwm_offset_))
    f_pwm_offset_ = 0.0; // -21.196;  // with the pwm percentage: x / 1800 * 100
  printf("f_pwm_offset_ is %.3f\n",f_pwm_offset_);

}

void TransformController::tfPubFunc(const ros::TimerEvent & e)
{
  cogCoordPublish();
}

void TransformController::controlFunc(const ros::TimerEvent & e)
{
  if(realtime_control_flag_)
    {
      //get transform;
      std::vector<tf::StampedTransform>  transforms;
      transforms.resize(link_num_);

      ros::Duration dur (0.02);
      if (tf_.waitForTransform(root_link_name_, links_name_[link_num_ - 1], ros::Time(0),dur))
        {
          for(int i = 0; i < link_num_; i++)
            {
              try
                {
                  tf_.lookupTransform(root_link_name_, links_name_[i], ros::Time(0), transforms[i]);
                }
              catch (tf::TransformException ex)
                {
                  ROS_ERROR("%s",ex.what());
                }
            }
          //time set
          system_tf_time_  = transforms[0].stamp_;

          cogComputation(transforms);
          principalInertiaComputation(transforms);
          visualization();

          if(!lqi_flag_) lqi_flag_ = true;
        }
    }
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


      //1. link base model
      //std::cout << "link" << i + 1 << "_origin :\n" << link_origin << std::endl;
      cog_n += link_origin * link_base_model_[i].getWeight(); 
      if(i > 0)
        {
          //2. link joint model
          // 座標変換
          Eigen::Vector3d joint_origin = link_rotate * link_joint_model_[i -1].getOffset()  + link_origin;
          cog_n += joint_origin * link_joint_model_[i - 1].getWeight();
        }

      //3. controlers
      //if(i == 0)
      if(i == controller2_link_)
        {// controller2
          Eigen::Vector3d controller_origin = link_rotate * controller_model_[1].getOffset()  + link_origin;
          cog_n += controller_origin * controller_model_[1].getWeight(); 
        }

      //if(i == 1)
      if(i == controller1_link_)
        {// controller1
          Eigen::Vector3d controller_origin = link_rotate * controller_model_[0].getOffset()  + link_origin;
          //std::cout << "controller1 :\n" << controller_origin << std::endl;
          cog_n += controller_origin * controller_model_[0].getWeight(); 
        }

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
      //offset
      Eigen::Vector3d origin_from_root_link;
      tf::vectorTFToEigen(transforms[i].getOrigin(), origin_from_root_link);

      //rotate
      Eigen::Quaterniond q;
      tf::quaternionTFToEigen(transforms[i].getRotation(), q);
      Eigen::Matrix3d rotate_m(q);

      //1. link base model
      //rotate(R_t * I * R)
      Eigen::Vector3d origin_from_cog = origin_from_root_link - getCog();
      links_origin_from_cog[i] = origin_from_cog;

      Eigen::Matrix3d link_rotated_inertia = rotate_m.transpose() *  link_base_model_[i].getInertia() * rotate_m;

      //offset http://homepage2.nifty.com/eman/dynamics/mom_tensor.html
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

      // if(debug_log_)
      //   {
      //     std::cout << " link_rotated_inertia :\n" << link_rotated_inertia << std::endl;
      //     std::cout << " link_offset_inertia :\n" << link_offset_inertia << std::endl;
      //   }



      //2. link joint model
      if(i > 0)
        {
          origin_from_cog = rotate_m * link_joint_model_[i -1].getOffset()  + origin_from_root_link - getCog();

          float joint_mass = link_joint_model_[i - 1].getWeight();
          Eigen::Matrix3d link_offset_inertia;
          link_offset_inertia << 
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
          links_inertia = links_inertia_tmp + link_offset_inertia;
        }
      //controller
      if(i == controller2_link_)
      //if(i == 0)
        {// controller2
          origin_from_cog = rotate_m * controller_model_[1].getOffset() + origin_from_root_link - getCog();
          float controller_mass = controller_model_[1].getWeight();
          Eigen::Matrix3d link_offset_inertia;
          link_offset_inertia << 
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
          links_inertia = links_inertia_tmp + link_offset_inertia;
        }
      if(i == controller1_link_)
      //if(i == 1)
        {// controller1
          origin_from_cog = rotate_m * controller_model_[0].getOffset() + origin_from_root_link - getCog();
          float controller_mass = controller_model_[0].getWeight();
          Eigen::Matrix3d link_offset_inertia;
          link_offset_inertia << 
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
          links_inertia = links_inertia_tmp + link_offset_inertia;
        }

    }
  links_inertia_ = links_inertia;


  if(debug2_log_)
     std::cout << "links inertia :\n" << links_inertia_ << std::endl;


  //pricipal inertia
  //eigen solver
  ros::Time start_time = ros::Time::now();
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(links_inertia_);
  //printf("inertia eigen time is: %f\n", ros::Time::now().toSec() - start_time.toSec());
  //std::cout << "The eigenvalues of inertia is:\n"  << eig.eigenvalues() << std::endl;
  //std::cout << "The diagonal is:\n"  << eig.eigenvalues().asDiagonal() << std::endl;

  Eigen::Matrix3d links_principal_inertia = eig.eigenvalues().asDiagonal();

  Eigen::Matrix3d rotate_matrix = eig.eigenvectors();

  //std::cout << "rotate_matrix :\n" << rotate_matrix << std::endl;


  //the reorder of the inertia and rotate matrix (just for 2D)!!
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
          ROS_ERROR("start");
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


  if(debug2_log_)
     std::cout << "pricipal inertia :\n" << getPrincipalInertia() << std::endl;


  //rotate the link origins from cog
  cog_matrix_ = getRotateMatrix().transpose();
  //std::cout << "cog matrix :\n" << cog_matrix_ << std::endl;

  std::vector<Eigen::Vector3d > links_origin_from_principal_cog;
  links_origin_from_principal_cog.resize(link_num_);
  for(int i = 0; i < link_num_; i ++)
    {
      links_origin_from_principal_cog[i] = cog_matrix_ * links_origin_from_cog[i];
       if(debug2_log_)
         std::cout << "link" << i + 1 <<"origin :\n" << links_origin_from_principal_cog[i] << std::endl;

    }
  setLinksOriginFromCog(links_origin_from_principal_cog);





  Eigen::Matrix3d rotate_matrix_tmp = getRotateMatrix();
  rotate_angle_ = atan2(rotate_matrix_tmp(1,0), rotate_matrix_tmp(0,0));
  //ROS_INFO("rotate angle is %f", rotate_angle_);

  // if(callback_flag_)
  //   {
  //     std_msgs::Float32 rotate_msg;
  //     rotate_msg.data = rotate_angle_;
  //     cog_rotate_pub_.publish(rotate_msg);
  //   }

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
  aerial_robot_msgs::RollPitchYawGain rpy_gain_msg;
  aerial_robot_msgs::YawThrottleGain yt_gain_msg;
  aerial_robot_base::DesireCoord desire_coord_msg;

  desire_coord_msg.roll = 0;
  desire_coord_msg.pitch = 0;
  desire_coord_msg.yaw = -rotate_angle_;  // should be reverse (cog coord is parent)

  yt_gain_msg.motor_num = link_num_;

  //double radian_convert_rate = M_PI/ 180 / 10 / f_pwm_rate_ * pwm_rate_ * 10000;
  //0.1deg => rad:  M_PI/180/10; f=> pwm(no_offset); 1e4(10000)x
  //double omega_convert_rate = (2279 * M_PI)/((32767.0 / 4.0 ) * 180) / f_pwm_rate_ * pwm_rate_ * 10000;    //(2279 * M_PI)/((32767.0 / 4.0 ) * 180.0); f =>pwm

  for(int i = 0; i < link_num_; i ++)
    {
      if(lqi_mode_ == LQI_FOUR_AXIS_MODE)
        {
          /* to flight controller */
          rpy_gain_msg.roll_p_gain[i] = K12_(i,0);
          rpy_gain_msg.roll_d_gain[i] = K12_(i,1);
          rpy_gain_msg.roll_i_gain[i] = K12_(i,8);

          rpy_gain_msg.pitch_p_gain[i] = K12_(i,2);
          rpy_gain_msg.pitch_d_gain[i] = K12_(i,3);
          rpy_gain_msg.pitch_i_gain[i] = K12_(i,9);

          rpy_gain_msg.yaw_d_gain[i] = K12_(i,5);

          /* to aerial_robot_base, feedback */
          yt_gain_msg.pos_p_gain_throttle.push_back(K12_(i,6));
          yt_gain_msg.pos_d_gain_throttle.push_back(K12_(i,7));
          yt_gain_msg.pos_i_gain_throttle.push_back(K12_(i,11));

          yt_gain_msg.pos_p_gain_yaw.push_back(K12_(i,4));
          yt_gain_msg.pos_d_gain_yaw.push_back(K12_(i,5));
          yt_gain_msg.pos_i_gain_yaw.push_back(K12_(i,10));

          /* to aerial_robot_base, feedforward */
          yt_gain_msg.roll_vec.push_back(-K12_(i,0));
          yt_gain_msg.pitch_vec.push_back(-K12_(i,2));
          yt_gain_msg.yaw_vec.push_back(-K12_(i,4));

        }
      else if(lqi_mode_ == LQI_THREE_AXIS_MODE)
        {
          rpy_gain_msg.roll_p_gain[i] = K9_(i,0);
          rpy_gain_msg.roll_d_gain[i] = K9_(i,1);
          rpy_gain_msg.roll_i_gain[i] = K9_(i,6);

          rpy_gain_msg.pitch_p_gain[i] = K9_(i,2);
          rpy_gain_msg.pitch_d_gain[i] = K9_(i,3);
          rpy_gain_msg.pitch_i_gain[i] = K9_(i,7);

          rpy_gain_msg.yaw_d_gain[i] = 0;

          yt_gain_msg.pos_p_gain_throttle.push_back(K9_(i,4));
          yt_gain_msg.pos_d_gain_throttle.push_back(K9_(i,5));
          yt_gain_msg.pos_i_gain_throttle.push_back(K9_(i,8));

          yt_gain_msg.pos_p_gain_yaw.push_back(0.0);
          yt_gain_msg.pos_d_gain_yaw.push_back(0.0);
          yt_gain_msg.pos_i_gain_yaw.push_back(0.0);

          //to aerial_robot_base, feedforward
          yt_gain_msg.roll_vec.push_back(-K9_(i,0));
          yt_gain_msg.pitch_vec.push_back(-K9_(i,2));
          yt_gain_msg.yaw_vec.push_back(0);
        }
    }

  cog_rotate_pub_.publish(desire_coord_msg);
  rpy_gain_pub_.publish(rpy_gain_msg);
  yaw_throttle_gain_pub_.publish(yt_gain_msg);

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
      // ROS_WARN("x_plus:%f, x_minus:%f, y_plus:%f, y_minus:%f", x_plus_max_dist, x_minus_max_dist, y_plus_max_dist, y_minus_max_dist);

      return false;
    }

  // ROS_INFO("x_plus:%f, x_minus:%f, y_plus:%f, y_minus:%f", x_plus_max_dist, x_minus_max_dist, y_plus_max_dist, y_minus_max_dist);

  return true;
}


bool TransformController::distThreCheckFromJointValues(const std::vector<double>& joint_values, int joint_offset, bool continous_flag)
{
  std::vector<tf::StampedTransform> transforms = transformsFromJointValues(joint_values, joint_offset);
  cogComputation(transforms);
  principalInertiaComputation(transforms, continous_flag);
  return distThreCheck();
}

bool  TransformController::stabilityCheck(bool debug)
{
  std::vector<Eigen::Vector3d> links_origin_from_cog(link_num_);
  getLinksOriginFromCog(links_origin_from_cog);
  Eigen::Matrix3d links_principal_inertia = getPrincipalInertia();
  
  //std::cout << "inertia :\n" << links_principal_inertia << std::endl;

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

      if(debug_log_ || debug)
        std::cout << "link" << i + 1 <<"origin :\n" << links_origin_from_cog[i] << std::endl;
    }
  i = 0;

  U_.row(0) = p_y / links_principal_inertia(0,0);
  U_.row(1) = p_x / links_principal_inertia(1,1);
  U_.row(2) = p_c / links_principal_inertia(2,2);
  U_.row(3) = p_m;
  if(debug_log_ || debug)
    std::cout << "U_:"  << std::endl << U_ << std::endl;

      ros::Time start_time = ros::Time::now();
  if(link_num_ == 4) 
    {//square mothod
      Eigen::FullPivLU<Eigen::MatrixXd> solver(U_);

      x = solver.solve(g);
      if(debug_log_)
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

      if(debug_log_)
        std::cout << "U det:"  << std::endl << (U_ * U_.transpose()).determinant() << std::endl;
    }

  if(debug_log_)
    ROS_INFO("U solver is: %f\n", ros::Time::now().toSec() - start_time.toSec());

  if(debug_log_ || debug)
    std::cout << "x:"  << std::endl << x << std::endl;

  if(x.maxCoeff() > f_max_ || x.minCoeff() < f_min_)
    {
      lqi_mode_ = LQI_THREE_AXIS_MODE;

      //debug
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
      if(debug_log_)
        std::cout << "x:"  << std::endl << x << std::endl;

#endif

      return false; //can not be stable
    }

  lqi_mode_ = LQI_FOUR_AXIS_MODE;
  return true;
}

void TransformController::lqi()
{
  // x = [roll, roll_d, pitch, pitch_d, yaw, yaw_d, z, z_d]
  ros::Rate loop_rate(lqi_thread_rate_);
  static int i = 0;
  static int cnt = 0;

  while(ros::ok())
    {
      if(lqi_flag_)
        {
          double start_time = ros::Time::now().toSec();
          //check the thre check
            if(!distThreCheck()) //[m]
              {
                ROS_ERROR("(singular pose, can not resolve the lqi control problem");
                loop_rate.sleep();
                continue;
              }

          //check the stability within the range of the motor force
          if(!stabilityCheck()) 
            ROS_ERROR("can not be four axis stable, switch to three axis stable mode");

          if(!hamiltonMatrixSolver(lqi_mode_)){ continue;}

           //just do publishing when link number is 4
          if(link_num_ == 4) param2contoller();

          //ROS_INFO("cal time is %f", ros::Time::now().toSec() - start_time);
        }
      loop_rate.sleep();
    }
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
      if(debug_log_)
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

      //eigen solving
      ros::Time start_time = ros::Time::now();
      Eigen::ComplexEigenSolver<Eigen::MatrixXcd> ces;
      ces.compute(H);

      if(debug_log_)
        ROS_INFO("h eigen time is: %f\n", ros::Time::now().toSec() - start_time.toSec());
      //std::cout << "The eigenvalues of H are:" << std::endl << ces.eigenvalues() << std::endl;
      //std::cout << "The eigenvalues vector of H are:" << std::endl << es.eigenvectors() << std::endl;

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

      start_time = ros::Time::now();
      Eigen::MatrixXcd f_inv  = f.inverse();
      if(debug_log_)
        ROS_INFO("f inverse: %f\n", ros::Time::now().toSec() - start_time.toSec());


      Eigen::MatrixXcd P = g * f_inv;

      //K
      K12_ = -R_inv * B12_aug_.transpose() * P.real();

      if(debug_log_)
        std::cout << "K is:" << std::endl << K12_ << std::endl;

      //check the eigen of new A
      Eigen::MatrixXd A12_dash = Eigen::MatrixXd::Zero(12, 12);
      A12_dash = A12_aug_ + B12_aug_ * K12_;
      // start_time = ros::Time::now();
      Eigen::EigenSolver<Eigen::MatrixXd> esa(A12_dash);
      if(debug_log_)
        std::cout << "The eigenvalues of A_hash are:" << std::endl << esa.eigenvalues() << std::endl;
      // ROS_INFO("A dash: %f\n", ros::Time::now().toSec() - start_time.toSec());

    }
  
  if(lqi_mode_ == LQI_THREE_AXIS_MODE)
    {
      B6_.row(1) = U_.row(0);
      B6_.row(3) = U_.row(1);
      B6_.row(5) = U_.row(3);
      if(debug_log_)
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

      //std::cout << " H  is:" << std::endl << H << std::endl;

      //eigen solving
      ros::Time start_time = ros::Time::now();
      Eigen::ComplexEigenSolver<Eigen::MatrixXcd> ces;
      ces.compute(H);

      if(debug_log_)
        ROS_INFO("h eigen time is: %f\n", ros::Time::now().toSec() - start_time.toSec());
      //std::cout << "The eigenvalues of H are:" << std::endl << ces.eigenvalues() << std::endl;
      //std::cout << "The eigenvalues vector of H are:" << std::endl << es.eigenvectors() << std::endl;

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
      if(debug_log_)
        ROS_INFO("f inverse: %f\n", ros::Time::now().toSec() - start_time.toSec());


      Eigen::MatrixXcd P = g * f_inv;

      //K
      K9_ = -R_inv * B9_aug_.transpose() * P.real();

      if(debug_log_)
        std::cout << "K is:" << std::endl << K9_ << std::endl;



      //check the eigen of new A
      Eigen::MatrixXd A9_dash = Eigen::MatrixXd::Zero(9, 9);
      A9_dash = A9_aug_ + B9_aug_ * K9_;
      // start_time = ros::Time::now();
      Eigen::EigenSolver<Eigen::MatrixXd> esa(A9_dash);
      if(debug_log_)
        std::cout << "The eigenvalues of A_hash are:" << std::endl << esa.eigenvalues() << std::endl;
      // ROS_INFO("A dash: %f\n", ros::Time::now().toSec() - start_time.toSec());
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
