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
      link_principal_inertia_ << link_i_xx_, 0, 0, 0, link_i_yy_, 0, 0, 0, link_i_zz_;
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
  nh_private_.param("link_num", link_num_, 4);
  nh_private_.param("link_length", link_length_, 0.5); //m, hydra prototye!
  nh_private_.param("propeller_diameter", propeller_diameter_, 0.2); //0.254m(10inch), 0.2(8inch)
  nh_private_.param("link_mass", link_mass_, 0.5); //g

  if (!nh_private_.getParam ("link_i_xx", link_i_xx_))
    link_i_xx_ = 0; //rod model
  printf(" link_i_xx_ is %.3f\n", link_i_xx_);
  if (!nh_private_.getParam ("link_i_yy", link_i_yy_))
    link_i_yy_ = link_mass_ * link_length_ * link_length_ /12; //rod model
  printf(" link_i_yy_ is %.3f\n", link_i_yy_);
  if (!nh_private_.getParam ("link_i_zz", link_i_zz_))
    link_i_zz_ = link_mass_ * link_length_ * link_length_ /12; //rod model
  printf(" link_i_zz_ is %.3f\n", link_i_zz_);

  nh_private_.param("multilink_i_yy", multilink_i_yy_, 0.0807787);
  nh_private_.param("multilink_i_xx", multilink_i_xx_, 0.0809074);
  nh_private_.param("multilink_i_zz", multilink_i_zz_, 0.161686);

  nh_private_.param("root_link_name", root_link_name_, std::string("/link3_abdomen")); 
  printf(" root_link_name_ is %s\n", root_link_name_.c_str());
  links_name_.resize(link_num_);

  for(int i = 0; i < link_num_; i++)
    {
      std::stringstream ss;
      ss << i + 1;
      links_name_[i] = std::string("/link") + ss.str()  + std::string("_abdomen");
    }

  nh_private_.param("x_distribution_scale", x_distribution_scale_, link_length_ / (sqrt(2) *2));
  nh_private_.param("y_distribution_scale", y_distribution_scale_, x_distribution_scale_);
  propeller_direction_.resize(link_num_);
  propeller_order_.resize(link_num_);
  q_vector_scale_.resize(link_num_);

  for(int i = 0; i < link_num_; i++)
    {
      std::stringstream ss;
      ss << i  + 1;
      nh_private_.param(std::string("propeller") + ss.str() + std::string("_direction"), propeller_direction_[i], 1);
      nh_private_.param(std::string("propeller") + ss.str() + std::string("_order"), propeller_order_[i], i);
      //ROS_INFO("propeller %d attribute: direction: %d, order: %d", i + 1, propeller_direction_[i], propeller_order_[i]);
    }

  for(int i = 0; i < 4; i++)
    {
      std::stringstream ss;
      ss << i  + 1;
      nh_private_.param(std::string("q") + ss.str() + std::string("_vector_scale"), q_vector_scale_[i], (double)link_num_);
    }

  nh_private_.param("debug_log", debug_log_, false); 

  Q_ = Eigen::MatrixXd::Zero(4,4);   
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
  ROS_ERROR("OK");
  ros::Duration dur (0.02);
  if (tf_.waitForTransform(root_link_name_, links_name_[link_num_ - 1], ros::Time(0),dur))
    {
      for(int i = 0; i < link_num_; i++)
        {
          try
            {
              //double time0 = ros::Time::now().toSec();
              tf_.lookupTransform(root_link_name_, links_name_[i], ros::Time(0), transforms[i]);
              //ROS_INFO("time %d: %f", i, ros::Time::now().toSec() - time0);
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

      bool q_available = qCompute();
      if(q_available) 
        {
          param2contoller();
        }
      else
        {
          ROS_ERROR("bad q computation");
        }
    }

  ROS_ERROR("OK_ end");

}

void TransformController::cogComputation(std::vector<tf::StampedTransform> transforms)
{
  float x_sum = 0, y_sum = 0, z_sum = 0;
  for(int i = 0; i < link_num_; i++)
    {
      x_sum += transforms[i].getOrigin().x();
      y_sum += transforms[i].getOrigin().y();
      z_sum += transforms[i].getOrigin().z();
    }
  Eigen::Matrix<double, 3, 1> cog;
  cog(0) = x_sum / link_num_ ; //all link have same mass!
  cog(1) = y_sum / link_num_ ; //all link have same mass!
  cog(2) = z_sum / link_num_ ; //all link have same mass!
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
      Eigen::Quaterniond q;
      tf::quaternionTFToEigen(transforms[i].getRotation(), q);
      Eigen::Matrix3d rotate_m(q);
      //rotate
      Eigen::Matrix3d link_rotated_inertia = rotate_m.transpose() *  link_principal_inertia_* rotate_m;
      //offset
      Eigen::Vector3d origin_from_root_link;
      tf::vectorTFToEigen(transforms[i].getOrigin(), origin_from_root_link);
      Eigen::Vector3d origin_from_cog = origin_from_root_link - getCog();
      links_origin_from_cog[i] = origin_from_cog;
      
      Eigen::Matrix3d link_offset_inertia;
      //http://homepage2.nifty.com/eman/dynamics/mom_tensor.html
      link_offset_inertia << 
        link_mass_ * origin_from_cog(1) * origin_from_cog(1),
        link_mass_ * (-origin_from_cog(0) * origin_from_cog(1)),
        0,
        link_mass_ * (-origin_from_cog(0) * origin_from_cog(1)),
        link_mass_ * origin_from_cog(0) * origin_from_cog(0),
        0,
        0,
        0,
        link_mass_ * origin_from_cog(0) * origin_from_cog(0) + link_mass_ * origin_from_cog(1) * origin_from_cog(1);

      Eigen::Matrix3d links_inertia_tmp = links_inertia;
      links_inertia = links_inertia_tmp + link_rotated_inertia + link_offset_inertia;
    }
  links_inertia_ = links_inertia;
  
  //eigen solver
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(links_inertia_);
  Eigen::Matrix3d links_principal_inertia = eig.eigenvalues().asDiagonal();
  Eigen::Matrix3d rotate_matrix = eig.eigenvectors();

  //std::cout << "rotate_matrix :\n" << rotate_matrix << std::endl;

  //the reorder of the inertia and rotate matrix (just for 2D)!!
  if(sgn(rotate_matrix(0,0)) != sgn(rotate_matrix(1,1)))
    {
      //ROS_WARN("sgn");
      rotate_matrix.col(0).swap(rotate_matrix.col(1));
        
      links_principal_inertia.col(0).swap(links_principal_inertia.col(1));
      links_principal_inertia.row(0).swap(links_principal_inertia.row(1));

      //std::cout << "rotate_matrix :\n" << rotate_matrix << std::endl;
    }


  if(continuous_flag)
    {
      int no = 0;

      if(!init_flag)
        {
          setRotateMatrix(rotate_matrix);

          links_principal_inertia_ = links_principal_inertia;


          init_links_principal_inertia_ <<multilink_i_xx_, 0, 0, 0, multilink_i_yy_, 0, 0, 0, multilink_i_zz_;
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

          for(int i = 0; i < link_num_; i++)
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


          //rotate_matrix_ = rotate_matrix_candidates[no];
          setRotateMatrix(rotate_matrix_candidates[no]);

          links_principal_inertia_ = links_principal_inertia_candidates[no];


        }
    }
  else
    {
      //rotate_matrix_ = rotate_matrix;
      setRotateMatrix(rotate_matrix);
    }


  //rotate the link origins from cog
  //cog_matrix_ = rotate_matrix_.transpose();
  cog_matrix_ = getRotateMatrix().transpose();
  //std::cout << "cog matrix :\n" << cog_matrix_ << std::endl;
  for(int i = 0; i < link_num_; i ++)
    {
      links_origin_from_cog_[i] = cog_matrix_ * links_origin_from_cog[i];
      if(debug_log_)
        {

          std::cout << "links origin :\n" << links_origin_from_cog_[i] << std::endl;
        }
    }


  Eigen::Matrix<double, 3, 3> rotate_matrix_tmp = getRotateMatrix();
  rotate_angle_ = atan2(rotate_matrix_tmp(1,0), rotate_matrix_tmp(0,0));
  std_msgs::Float32 rotate_msg;
  rotate_msg.data = rotate_angle_;
  cog_rotate_pub_.publish(rotate_msg);
  //ROS_INFO("rotate angle is %f", rotate_angle_);

}


bool TransformController::qCompute()
{
  

  Eigen::MatrixXd P(link_num_, 4); 
  for(int propeller_order = 0; propeller_order < link_num_; propeller_order++)
    {
      double propeller_x = links_origin_from_cog_[propeller_order](0);
      double propeller_y = links_origin_from_cog_[propeller_order](1);

      Eigen::Vector4d propeller_distribution;
      propeller_distribution << 1, propeller_y, - propeller_x, propeller_direction_[propeller_order];
      int motor_order = propeller_order_[propeller_order];
      P.row(motor_order) = propeller_distribution;
      
    }
  if(debug_log_)
    {
      std::cout << "P :\n" << P << std::endl;
    } 




#if 1 //original dfa
  for(int i = 0; i < link_num_; i++)
    {
      Q_.col(i) = q_vector_scale_[i] * P.col(i) / P.col(i).squaredNorm();
    }

#else //Ax = b => x
  Eigen::Matrix<double, 4, 4> I = Eigen::MatrixXd::Identity(4,4) * 4;
  
  double det = P.determinant();
  ROS_WARN("determinant is %lf", det);
  if(fabs(det) < 1e-6 )
      {
        ROS_ERROR("det is too small: %lf", det);
      }
    else
      {
        for(int i = 0; i < link_num_; i++)
          {
            Eigen::FullPivLU<Eigen::MatrixXd> solver(P.transpose());
            Q_.col(i) = solver.solve(I.col(i));
          }
      }
#endif

  //for the mg log 
  /*
  Eigen::FullPivLU<Eigen::MatrixXd> solver(P.transpose());
  Eigen::Matrix<double, 4, 1> U;
  U << 2100, 0, 0, 0;
  Eigen::Matrix<double, 4, 1> F;
  F = solver.solve(U);
  std::cout << "F :\n" << F << std::endl;
  */
  //test P.transpose() * P
  //std::cout << "P.t * P :\n" << P.transpose() * P  << std::endl;


  if(debug_log_)
    {
      std::cout << "Q :\n" << Q_ << std::endl;
      std::cout << "Pt * Q :\n" << P.transpose() * Q_ << std::endl;
    }
  return true;

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

  //ROS_INFO("arrow1 x:%f, y:%f, z:%f, w:%f", q.x(), q.y(), q.z(),q.w());
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
  //ROS_INFO("arrow2 x:%f, y:%f, z:%f, w:%f", q2.x(), q2.y(), q2.z(),q2.w());
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

  //inertial rate => pid gain rate
  Eigen::Vector3d i_prncipal_rate_f;
  i_prncipal_rate_f << 
    links_principal_inertia_(0, 0) / init_links_principal_inertia_(0, 0),
    links_principal_inertia_(1, 1) / init_links_principal_inertia_(1, 1),
    links_principal_inertia_(2, 2) / init_links_principal_inertia_(2, 2);
  
  Eigen::Matrix4d Q_inertia_fact = Q_;
  for(int j = 0; j < 3; j++)
    {
   
      if(j != 2)
        {//no yaw
          Q_inertia_fact.col(j + 1) = Q_.col(j + 1) * i_prncipal_rate_f(j);
        }
      else
        {
          Q_inertia_fact.col(j + 1) = Q_.col(j + 1) ;
        }
    }
  if(debug_log_)
    {

  std::cout << "links_principal_inertia_ :\n" << links_principal_inertia_ << std::endl;
  std::cout << "init_links_principal_inertia_ :\n" << init_links_principal_inertia_ << std::endl;
  std::cout << "i_prncipal_rate_f :\n" << i_prncipal_rate_f << std::endl;
  std::cout << "Q_inertia_fact :\n" << Q_inertia_fact << std::endl;
    }
  for(int j = 0; j < 3; j++)
    {
      for(int i = 0; i < link_num_; i++)
        param_msg.q_matrix[j * link_num_ + i] = (uint16_t)(Q_inertia_fact(i, j + 1) * 1024);
    }
  


  //param_msg.i_principal_rate[0] = (uint16_t)(i_prncipal_rate_f(0) * 1024);
  //param_msg.i_principal_rate[1] = (uint16_t)(i_prncipal_rate_f(1) * 1024);
  //param_msg.i_principal_rate[2] = (uint16_t)(i_prncipal_rate_f(2) * 1024);

  //rotate angle for the attitude and gyro
  param_msg.rotate_angle[0] = (uint16_t)(cog_matrix_(0, 0) * 1024);
  param_msg.rotate_angle[1] = (uint16_t)(cog_matrix_(1, 0) * 1024);

  transform_control_pub_.publish(param_msg);
  ROS_WARN("ok");
}

double TransformController::getLinkLength()
{
  return link_length_;
}

double TransformController::getPropellerDiameter()
{
  return propeller_diameter_;
}

int TransformController::getLinkNum()
{
  return link_num_;
}

