/***
1. the model of quad-rotor is too wrong, especially the position from propeller to joint
 ***/

#include <hydra_transform_control/transform_control.h>


TransformController::TransformController(ros::NodeHandle nh, ros::NodeHandle nh_private, bool callback_flag): nh_(nh),nh_private_(nh_private)
{

  initParam();

  principal_axis_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("orientation_data", 1);

  transform_control_pub_ = nh_.advertise<hydra_transform_control::HydraParam>("kduino/hydra_param", 1);
  cog_rotate_pub_ = nh_.advertise<std_msgs::Float32>("/cog_rotate", 1); //absolute
  cog_(0) = 0;
  cog_(1) = 0;
  cog_(2) = 0;

  links_origin_from_cog_.resize(link_num_);
  rotate_matrix_ = Eigen::Matrix3d::Identity();

  //for(int i = 0; i < link_num_; i++)
  //link_principal_inertia_ << link_i_xx_, 0, 0, 0, link_i_yy_, 0, 0, 0, link_i_zz_;


  //lqi
  lqi_flag_ = false;
  //A
  A_ = Eigen::MatrixXd::Zero(8,8);
  A_(0,1) = 1;
  A_(2,3) = 1;
  A_(4,5) = 1;
  A_(6,7) = 1;

  //A
  B_ = Eigen::MatrixXd::Zero(8,4); //4 is the link number, should change!!


  //C
  C_ = Eigen::MatrixXd::Zero(4,8);
  C_(0,0) = 1;
  C_(1,2) = 1;
  C_(2,4) = 1;
  C_(3,6) = 1;


  //A_aug
  A_aug_ = Eigen::MatrixXd::Zero(12,12);
  A_aug_.block<8,8>(0,0) = A_;
  A_aug_.block<4,8>(8,0) = -C_;


  //A_aug
  C_aug_ = Eigen::MatrixXd::Zero(4,12);
  C_aug_.block<4,8>(0,0) = C_;



  //Q
  Eigen::VectorXd  q_diagonal(12);
  q_diagonal << q_roll_,q_roll_d_,q_pitch_,q_pitch_d_,q_yaw_,q_yaw_d_,q_z_,q_z_d_, q_roll_i_,q_pitch_i_,q_yaw_i_,q_z_i_;



  Q_ = q_diagonal.asDiagonal();



  //R
  R_ = Eigen::Matrix4d(Eigen::Vector4d(r_[0], r_[1],r_[2], r_[3]).asDiagonal()); // bad, 4 should be link num


  std::cout << "A:"  << std::endl << A_ << std::endl;
  std::cout << "C:"  << std::endl << C_ << std::endl;

  std::cout << "A_aug:"  << std::endl << A_aug_ << std::endl;
  std::cout << "C_aug:"  << std::endl << C_aug_ << std::endl;


  std::cout << "Q:"  << std::endl << Q_ << std::endl;
  std::cout << "R:"  << std::endl << R_ << std::endl;



  if(callback_flag)
    {
      control_timer_ = nh_private_.createTimer(ros::Duration(1.0 / control_rate_),
                                               &TransformController::controlFunc, this);

      
      tf_pub_timer_ = nh_private_.createTimer(ros::Duration(1.0 / tf_pub_rate_),
                                              &TransformController::tfPubFunc, this);
      
      
      lqi_thread_ = boost::thread(boost::bind(&TransformController::lqi, this));
    }


}
TransformController::~TransformController()
{
  lqi_thread_.interrupt();
  lqi_thread_.join();

}

void TransformController::initParam()
{
  nh_private_.param("control_rate", control_rate_, 20.0);
  nh_private_.param("tf_pub_rate", tf_pub_rate_, 60.0);

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


  nh_private_.param("root_link_name", root_link_name_, std::string("/link3"));
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

  //dynamics
  if (!nh_private_.getParam ("m_f_rate", m_f_rate_))
    m_f_rate_ = 0.016837; //the sgn is right?, should be nagative
  printf("m_f_rate_ is %.3f\n",m_f_rate_);
  if (!nh_private_.getParam ("f_pwm_rate", f_pwm_rate_))
    f_pwm_rate_ = 0.3029; // with the pwm percentage: x / 1800 * 100
  printf("f_pwm_rate_ is %.3f\n",f_pwm_rate_);
  if (!nh_private_.getParam ("f_pwm_offset", f_pwm_offset_))
    f_pwm_offset_ = -21.196;  // with the pwm percentage: x / 1800 * 100
  printf("f_pwm_offset_ is %.3f\n",f_pwm_offset_);

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

  if (!nh_private_.getParam ("alfa", alfa_))
    alfa_ = 0.0;
  printf("alfa_ is %.3f\n", alfa_);


}

void TransformController::tfPubFunc(const ros::TimerEvent & e)
{
  cogCoordPublish();
}

void TransformController::controlFunc(const ros::TimerEvent & e)
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

void TransformController::cogComputation(std::vector<tf::StampedTransform> transforms)
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
      if(i == 0)
        {// controller2
          Eigen::Vector3d controller_origin = link_rotate * controller_model_[1].getOffset()  + link_origin;
          cog_n += controller_origin * controller_model_[1].getWeight(); 
        }
      if(i == 1)
        {// controller1
          Eigen::Vector3d controller_origin = link_rotate * controller_model_[0].getOffset()  + link_origin;
          //std::cout << "controller1 :\n" << controller_origin << std::endl;
          cog_n += controller_origin * controller_model_[0].getWeight(); 
        }
    }

  Eigen::Vector3d cog = cog_n / all_mass_;
  setCog(cog);
}

void TransformController::principalInertiaComputation(std::vector<tf::StampedTransform> transforms, bool continuous_flag)
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
      if(i == 0)
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
      if(i == 1)
        {// controller1
          origin_from_cog = rotate_m * controller_model_[0].getOffset() + origin_from_root_link - getCog();
          float controller_mass = controller_model_[0].getWeight();
          Eigen::Matrix3d link_offset_inertia;
          link_offset_inertia << 
            controller_mass * origin_from_cog(0) * origin_from_cog(0),
            controller_mass * (-origin_from_cog(0) * origin_from_cog(0)),
            0,
            controller_mass * (-origin_from_cog(0) * origin_from_cog(0)),
            controller_mass * origin_from_cog(0) * origin_from_cog(0),
            0,
            0,
            0,
            controller_mass * (origin_from_cog(0) * origin_from_cog(0) + origin_from_cog(0) * origin_from_cog(0));

          Eigen::Matrix3d links_inertia_tmp = links_inertia;
          links_inertia = links_inertia_tmp + link_offset_inertia;
        }


    }
  links_inertia_ = links_inertia;
   // if(debug_log_)
   //   std::cout << " inertia :\n" << links_inertia_ << std::endl;


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
  std_msgs::Float32 rotate_msg;
  rotate_msg.data = rotate_angle_;
  cog_rotate_pub_.publish(rotate_msg);
  //ROS_INFO("rotate angle is %f", rotate_angle_);

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
  hydra_transform_control::HydraParam param_msg;


  param_msg.rotate_angle[0] = (int16_t)(cog_matrix_(0, 0) * 1024);
  param_msg.rotate_angle[1] = (int16_t)(cog_matrix_(1, 0) * 1024);

  transform_control_pub_.publish(param_msg);

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
          std::vector<Eigen::Vector3d> links_origin_from_cog = getLinksOriginFromCog();
          Eigen::Matrix3d links_principal_inertia = getPrincipalInertia();
          Eigen::Vector4d p_x, p_y, p_c, p_m; 

          //std::cout << "inertia :\n" << links_principal_inertia << std::endl;

          for(; i < link_num_; i++)
            {
              int order = propeller_order_[i];
              p_y(order) = links_origin_from_cog[i](1); 
              p_x(order) = -links_origin_from_cog[i](0);
              p_c(order) = propeller_direction_[i] * m_f_rate_ / links_principal_inertia(2,2);
              p_m(order) = 1 / all_mass_;

              //std::cout << "link" << i + 1 <<"origin :\n" << links_origin_from_cog[i] << std::endl;
            }
          i = 0;

          //check the controllability!!
          if(debug_log_)
            {
              std::cout << "x norm :\n" << p_x.norm() << std::endl;
              std::cout << "y norm :\n" << p_y.norm() << std::endl;
            }

          if(p_y.norm() < 0.1 || p_x.norm() < 0.1) //[m]
            {
              ROS_ERROR("(singular pose, can not resolve the lqi control problem");
              loop_rate.sleep();
              continue;
            }

          B_.row(1) = p_y / links_principal_inertia(0,0);
          B_.row(3) = p_x / links_principal_inertia(1,1);
          B_.row(5) = p_c;
          B_.row(7) = p_m;

          if(debug_log_)
            std::cout << "B:"  << std::endl << B_ << std::endl;

          B_aug_ = Eigen::MatrixXd::Zero(12, 4);
          B_aug_.block<8,4>(0,0) = B_;
          //std::cout << "B_aug:"  << std::endl << B_aug_<< std::endl;
     
          if(!hamiltonMatrixSolver()){ continue;}
        }

      loop_rate.sleep();
    }
}

bool TransformController::hamiltonMatrixSolver()
{
  //for the R which is  diagonal matrix. should be changed to link_num
  Eigen::MatrixXd R_inv = Eigen::Matrix4d(Eigen::Vector4d(1 / r_[0], 1 / r_[1], 1 / r_[2], 1 / r_[3]).asDiagonal());


  // hamilton matrix

  Eigen::MatrixXcd H = Eigen::MatrixXcd::Zero(24,24); //all right?
  H.block<12,12>(0,0) = A_aug_.cast<std::complex<double> >();

  H.block<12,12>(12,0) = -(Q_.cast<std::complex<double> >());
  H.block<12,12>(0,12) = - (B_aug_ * R_inv * B_aug_.transpose()).cast<std::complex<double> >();
  H.block<12,12>(12,12) = - (A_aug_.transpose()).cast<std::complex<double> >();

  //std::cout << " H  is:" << std::endl << H << std::endl;

  //eigen shift
  //Eigen::MatrixXcd H_tmp = H; //all right?
  //H = H_tmp + Eigen::MatrixXcd::Identity(24,24) * alfa_;


  //std::cout << " R inv  is:" << std::endl << - B_aug_ * R_inv * B_aug_.transpose() << std::endl;


  

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


  //std::cout << "f:" << std::endl << f << std::endl;
  //std::cout << "g:" << std::endl << g << std::endl;


  start_time = ros::Time::now();
  Eigen::MatrixXcd f_inv  = f.inverse();
  //std::cout << "f inv is:" << std::endl << f_inv << std::endl;
  if(debug_log_)
    ROS_INFO("f inverse: %f\n", ros::Time::now().toSec() - start_time.toSec());


  Eigen::MatrixXcd P = g * f_inv;

  //std::cout << "P is:" << std::endl << P << std::endl;

  //K
  Eigen::MatrixXd K = -R_inv * B_aug_.transpose() * P.real();

  if(debug_log_)
    std::cout << "K is:" << std::endl << K << std::endl;


  //check the eigen of new A
   Eigen::MatrixXd A_dash = Eigen::MatrixXd::Zero(12, 12);
   A_dash = A_aug_ + B_aug_ * K;
  // start_time = ros::Time::now();
   Eigen::EigenSolver<Eigen::MatrixXd> esa(A_dash);
  if(debug_log_)
    std::cout << "The eigenvalues of A_hash are:" << std::endl << esa.eigenvalues() << std::endl;
  // ROS_INFO("A dash: %f\n", ros::Time::now().toSec() - start_time.toSec());

  return true;

}
