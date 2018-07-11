#include <hydrus/transform_control.h>

TransformController::TransformController(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh), nh_private_(nh_private),
  kinematics_flag_(false),
  p_det_(0), stability_margin_(0)
{
  nh_private_.param("verbose", verbose_, false);
  ROS_ERROR("ns is %s", nh_private_.getNamespace().c_str());

  /* param for kinematics */
  nh_private_.param("baselink", baselink_, std::string("link1"));
  if(verbose_) std::cout << "baselink: " << baselink_ << std::endl;
  nh_private_.param("thrust_link", thrust_link_, std::string("thrust"));
  if(verbose_) std::cout << "thrust_link: " << thrust_link_ << std::endl;
  nh_private_.param("kinematic_verbose", kinematic_verbose_, false);

  kinematic_model_ = aerial_robot_model::RobotModel(baselink_, thrust_link_, kinematic_verbose_);
  rotor_num_ = kinematic_model_.getRotorNum();
  initParam();

  //publisher
  //those publisher is published from func param2controller
  rpy_gain_pub_ = nh_private_.advertise<spinal::RollPitchYawTerms>("/rpy_gain", 1);
  four_axis_gain_pub_ = nh_.advertise<aerial_robot_msgs::FourAxisGain>("/four_axis_gain", 1);
  transform_pub_ = nh_.advertise<geometry_msgs::TransformStamped>("/cog2baselink", 1);
  p_matrix_pseudo_inverse_inertia_pub_ = nh_.advertise<spinal::PMatrixPseudoInverseWithInertia>("p_matrix_pseudo_inverse_inertia", 1);

  //subscriber
  desire_coordinate_sub_ = nh_.subscribe("/desire_coordinate", 1, &TransformController::desireCoordinateCallback, this);
  actuator_state_sub_ = nh_private_.subscribe("joint_states", 1, &TransformController::actuatorStateCallback, this);

  //dynamic reconfigure server
  dynamic_reconf_func_lqi_ = boost::bind(&TransformController::cfgLQICallback, this, _1, _2);
  lqi_server_.setCallback(dynamic_reconf_func_lqi_);

  //ros service for extra module
  //add_extra_module_service_ = nh_.advertiseService("add_extra_module", &aerial_robot_model::RobotModel::addExtraModuleCallback, &kinematic_model_);

  control_thread_ = std::thread(boost::bind(&TransformController::control, this));

  /* Linear Quadratic Control */
  //U //TODO U? P?
  P_ = Eigen::MatrixXd::Zero(4, rotor_num_);

  //Q
  q_diagonal_ = Eigen::VectorXd::Zero(LQI_FOUR_AXIS_MODE * 3);
  q_diagonal_ << q_roll_,q_roll_d_,q_pitch_,q_pitch_d_,q_z_,q_z_d_,q_yaw_,q_yaw_d_, q_roll_i_,q_pitch_i_,q_z_i_,q_yaw_i_;
  //std::cout << "Q elements :"  << std::endl << q_diagonal_ << std::endl;

  lqi_mode_ = LQI_FOUR_AXIS_MODE;
}

TransformController::~TransformController()
{
  control_thread_.join();
}

void TransformController::initParam()
{
  nh_private_.param("control_rate", control_rate_, 15.0);
  if(verbose_) std::cout << "control_rate: " << std::setprecision(3) << control_rate_ << std::endl;

  nh_private_.param("only_three_axis_mode", only_three_axis_mode_, false);
  nh_private_.param("gyro_moment_compensation", gyro_moment_compensation_, false);
  nh_private_.param("control_verbose", control_verbose_, false);
  nh_private_.param("debug_verbose", debug_verbose_, false);
  nh_private_.param("a_dash_eigen_calc_flag", a_dash_eigen_calc_flag_, false);

  /* propeller direction and lqi R */
  r_.resize(rotor_num_);
  for(int i = 0; i < rotor_num_; ++i) {
    std::stringstream ss;
    ss << i + 1;
    /* R */
    nh_private_.param(std::string("r") + ss.str(), r_[i], 1.0);
    if(verbose_) std::cout << std::string("r") + ss.str() << ": " << r_[i] << std::endl;
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

  /* dynamics: motor */
  ros::NodeHandle control_node("/motor_info");
  control_node.param("m_f_rate", m_f_rate_, 0.01);
  if(verbose_) std::cout << "m_f_rate: " << std::setprecision(3) << m_f_rate_ << std::endl;
}

void TransformController::control()
{
  ros::Rate loop_rate(control_rate_);

  while(ros::ok())
    {
      if(debug_verbose_) ROS_ERROR("start lqi");
      lqi();
      if(debug_verbose_) ROS_ERROR("finish lqi");
      loop_rate.sleep();
    }
}

void TransformController::desireCoordinateCallback(const spinal::DesireCoordConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  kinematic_model_.setCogDesireOrientation(msg->roll, msg->pitch, msg->yaw);
}

void TransformController::actuatorStateCallback(const sensor_msgs::JointStateConstPtr& state)
{
  std::lock_guard<std::mutex> lock(mutex_);
  current_actuator_state_ = *state;

  if(kinematic_model_.getActuatorJointMap().empty()) kinematic_model_.setActuatorJointMap(current_actuator_state_);

  if(debug_verbose_) ROS_ERROR("start kinematics");
  kinematic_model_.forwardKinematics(current_actuator_state_);
  if(debug_verbose_) ROS_ERROR("finish kinematics");

  geometry_msgs::TransformStamped tf = kinematic_model_.getCog<geometry_msgs::TransformStamped>();
  tf.header = state->header;
  tf.header.frame_id = kinematic_model_.getRootFrameName();
  tf.child_frame_id = "cog";
  br_.sendTransform(tf);

  if(!kinematics_flag_)
    {
      ROS_ERROR("the total mass is %f", kinematic_model_.getMass());
      kinematics_flag_ = true;
    }
}

void TransformController::lqi()
{
  if(!kinematics_flag_) return;
  std::lock_guard<std::mutex> lock(mutex_);

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

  const std::vector<Eigen::Vector3d> rotors_origin_from_cog = kinematic_model_.getRotorsOriginFromCog<Eigen::Vector3d>();

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

  double link_length = kinematic_model_.getLinkLength();
  assert(link_length > 0);
  stability_margin_ = sqrt(es.eigenvalues()[0]) / link_length;
  if(verbose) ROS_INFO("stability_margin: %f", stability_margin_);
  if(stability_margin_ < stability_margin_thre_) return false;
  return true;

  //TODO need?
#if 0 // correlation coefficient
  double correlation_coefficient = fabs(s_xy / sqrt(s_xx * s_yy));
  //ROS_INFO("correlation_coefficient: %f", correlation_coefficient);

  if(correlation_coefficient > correlation_thre_ ) return false;
  return true;
#endif
}

bool TransformController::modelling(bool verbose)
{
  const std::vector<Eigen::Vector3d> rotors_origin_from_cog = kinematic_model_.getRotorsOriginFromCog<Eigen::Vector3d>();
  Eigen::Matrix3d links_inertia = kinematic_model_.getInertia<Eigen::Matrix3d>();

  Eigen::VectorXd g(4);
  g << 0, 0, 9.8, 0;
  Eigen::VectorXd p_x(rotor_num_), p_y(rotor_num_), p_c(rotor_num_), p_m(rotor_num_);

  auto rotor_direction = kinematic_model_.getRotorDirection();
  for(int i = 0; i < rotor_num_; i++)
    {
      p_y(i) =  rotors_origin_from_cog[i](1);
      p_x(i) = -rotors_origin_from_cog[i](0);
      p_c(i) = rotor_direction.at(i + 1) * m_f_rate_ ;
      p_m(i) = 1 / kinematic_model_.getMass();
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
  spinal::PMatrixPseudoInverseWithInertia p_pseudo_inverse_with_inertia_msg;

  four_axis_gain_msg.motor_num = rotor_num_;
  rpy_gain_msg.motors.resize(rotor_num_);
  p_pseudo_inverse_with_inertia_msg.pseudo_inverse.resize(rotor_num_);

  /* the transform from cog to baselink */
  geometry_msgs::TransformStamped transform_msg = kinematic_model_.getCog2Baselink<geometry_msgs::TransformStamped>();
  transform_msg.header.stamp = current_actuator_state_.header.stamp;
  transform_msg.header.frame_id = std::string("cog");
  transform_msg.child_frame_id = baselink_;
  transform_pub_.publish(transform_msg);

  for(int i = 0; i < rotor_num_; ++i)
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
  Eigen::Matrix3d inertia = kinematic_model_.getInertia<Eigen::Matrix3d>();
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

//TODO need?
// bool TransformController::defaultDistanceFunction(CollisionObject<double>* o1, CollisionObject<double>* o2, void* cdata_, double& dist)
// {
//   auto* cdata = static_cast<DistanceData*>(cdata_);
//   const DistanceRequest<double>& request = cdata->request;
//   DistanceResult<double>& result = cdata->result;

//   if(cdata->done) { dist = result.min_distance; return true; }

//   distance(o1, o2, request, result);

//   dist = result.min_distance;

//   if(dist <= 0) return true; // in collision or in touch

//   return cdata->done;
// }
