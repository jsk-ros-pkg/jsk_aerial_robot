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

  for(int i = 0; i < link_num_; i++)
    {

      //link_principal_inertia_ << link_i_xx_, 0, 0, 0, link_i_yy_, 0, 0, 0, link_i_zz_;
    }

  if(callback_flag)
    {
      control_timer_ = nh_private_.createTimer(ros::Duration(1.0 / control_rate_),
                                               &TransformController::controlFunc, this);

      
      tf_pub_timer_ = nh_private_.createTimer(ros::Duration(1.0 / tf_pub_rate_),
                                              &TransformController::tfPubFunc, this);
      
    }


}
TransformController::~TransformController()
{
}

void TransformController::initParam()
{
  nh_private_.param("control_rate", control_rate_, 20.0);
  nh_private_.param("tf_pub_rate", tf_pub_rate_, 60.0);

  nh_private_.param("debug_log", debug_log_, false); 

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

      Eigen::Matrix<double, 3, 3> link_base_model_inertia;
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
          Eigen::Matrix<double, 3, 1> offset_;
          offset_ << link_joint_offset_, 0, 0;
          ElementModel link_joint_model(nh_, nh_private_, link_joint_mass_, zero_inertia, offset_);
          link_joint_model_[i] = link_joint_model;
          all_mass_ += link_joint_model.getWeight();
        }

      //propeller
      nh_private_.param(std::string("propeller") + ss.str() + std::string("_direction"), propeller_direction_[i], 1);
      nh_private_.param(std::string("propeller") + ss.str() + std::string("_order"), propeller_order_[i], i);

    }

  /// 3. controller
  Eigen::Matrix<double, 3, 1> offset_;
  offset_ << controller1_offset_, 0, 0;
  ElementModel controller1_model(nh_, nh_private_, controller1_mass_, zero_inertia, offset_);
  all_mass_ += controller1_model.getWeight();
  controller_model_[0] = controller1_model;

  offset_ << controller2_offset_, 0, 0;
  ElementModel controller2_model(nh_, nh_private_, controller2_mass_, zero_inertia, offset_);
  all_mass_ += controller2_model.getWeight();
  controller_model_[1] = controller2_model;

  ROS_INFO("Mass :%f", all_mass_);

  /*
  nh_private_.param("multilink_i_yy", multilink_i_yy_, 0.0807787);
  nh_private_.param("multilink_i_xx", multilink_i_xx_, 0.0809074);
  nh_private_.param("multilink_i_zz", multilink_i_zz_, 0.161686);
  */

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

    }

}

void TransformController::cogComputation(std::vector<tf::StampedTransform> transforms)
{
  Eigen::Matrix<double, 3, 1> cog_n  = Eigen::Vector3d::Zero();
  for(int i = 0; i < link_num_; i++)
    {
      Eigen::Matrix<double, 3, 1> link_origin;
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
          Eigen::Matrix<double, 3, 1> joint_origin = link_rotate * link_joint_model_[i -1].getOffset()  + link_origin;
          cog_n += joint_origin * link_joint_model_[i - 1].getWeight();
        }

      //3. controlers
      if(i == 0)
        {// controller2
          Eigen::Matrix<double, 3, 1> controller_origin = link_rotate * controller_model_[1].getOffset()  + link_origin;
          cog_n += controller_origin * controller_model_[1].getWeight(); 
        }
      if(i == 1)
        {// controller1
          Eigen::Matrix<double, 3, 1> controller_origin = link_rotate * controller_model_[0].getOffset()  + link_origin;
          //std::cout << "controller1 :\n" << controller_origin << std::endl;
          cog_n += controller_origin * controller_model_[0].getWeight(); 
        }
    }

  Eigen::Matrix<double, 3, 1> cog = cog_n / all_mass_;
  setCog(cog);
}

void TransformController::principalInertiaComputation(std::vector<tf::StampedTransform> transforms, bool continuous_flag)
{
  static bool init_flag = false;

  Eigen::Matrix3d links_inertia = Eigen::Matrix3d::Zero(3,3);
  std::vector<Eigen::Matrix<double, 3, 1> > links_origin_from_cog;
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
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(links_inertia_);
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
          links_principal_inertia_ = links_principal_inertia;

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
          links_principal_inertia_ = links_principal_inertia_candidates[no];

        }
    }
  else
    {
      setRotateMatrix(rotate_matrix);
    }

  if(debug_log_)
    std::cout << "pricipal inertia :\n" << links_principal_inertia_ << std::endl;


  //rotate the link origins from cog
  cog_matrix_ = getRotateMatrix().transpose();
  //std::cout << "cog matrix :\n" << cog_matrix_ << std::endl;
  for(int i = 0; i < link_num_; i ++)
    {
      links_origin_from_cog_[i] = cog_matrix_ * links_origin_from_cog[i];
      if(debug_log_)
        {
          
          std::cout << "link" << i + 1 <<"origin :\n" << links_origin_from_cog_[i] << std::endl;
        }
    }


  Eigen::Matrix<double, 3, 3> rotate_matrix_tmp = getRotateMatrix();
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


  Eigen::Matrix<double, 3, 1> cog = getCog();
  Eigen::Matrix<double, 3, 3> rotate_matrix = getRotateMatrix();
  transform.setOrigin( tf::Vector3(cog(0), cog(1), cog(2)));
  Eigen::Quaterniond q_eigen(rotate_matrix);

  tf::Quaternion q_tf;
  tf::quaternionEigenToTF(q_eigen, q_tf);
  transform.setRotation(q_tf);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), root_link_name_, "cog"));

}

void TransformController::visualization()
{
  Eigen::Matrix<double, 3, 1> cog = getCog();
  Eigen::Matrix<double, 3, 3> rotate_matrix = getRotateMatrix();

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

