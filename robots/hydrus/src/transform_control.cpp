#include <hydrus/transform_control.h>

TransformController::TransformController(ros::NodeHandle nh, ros::NodeHandle nh_private, std::unique_ptr<HydrusRobotModel> robot_model):
  RobotModelRos(nh, nh_private, std::move(robot_model)),
  nh_(nh),
  nh_private_(nh_private)
{
  nh_private_.param("verbose", verbose_, false);
  ROS_ERROR("ns is %s", nh_private_.getNamespace().c_str());

  initParam();

  //publisher
  //those publisher is published from func param2controller
  rpy_gain_pub_ = nh_private_.advertise<spinal::RollPitchYawTerms>("/rpy_gain", 1);
  four_axis_gain_pub_ = nh_.advertise<aerial_robot_msgs::FourAxisGain>("/four_axis_gain", 1);
  p_matrix_pseudo_inverse_inertia_pub_ = nh_.advertise<spinal::PMatrixPseudoInverseWithInertia>("p_matrix_pseudo_inverse_inertia", 1);

  //dynamic reconfigure server
  dynamic_reconf_func_lqi_ = boost::bind(&TransformController::cfgLQICallback, this, _1, _2);
  lqi_server_.setCallback(dynamic_reconf_func_lqi_);

  control_thread_ = std::thread(boost::bind(&TransformController::control, this));

  //Q
  q_diagonal_ = Eigen::VectorXd::Zero(HydrusRobotModel::LQI_FOUR_AXIS_MODE * 3);
  q_diagonal_ << q_roll_,q_roll_d_,q_pitch_,q_pitch_d_,q_z_,q_z_d_,q_yaw_,q_yaw_d_, q_roll_i_,q_pitch_i_,q_z_i_,q_yaw_i_;
  //std::cout << "Q elements :"  << std::endl << q_diagonal_ << std::endl;
}

TransformController::~TransformController()
{
  control_thread_.join();
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

bool TransformController::hamiltonMatrixSolver()
{
  /* for the R which is  diagonal matrix. should be changed to rotor_num */
  const int rotor_num = getRobotModel().getRotorNum();
  const int lqi_mode = getRobotModel().getLqiMode();
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(lqi_mode * 3, lqi_mode * 3);
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(lqi_mode * 3, rotor_num);
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(lqi_mode, lqi_mode * 3);

  for(int i = 0; i < lqi_mode; i++)
    {
      A(2 * i, 2 * i + 1) = 1;
      B.row(2 * i + 1) = getRobotModel().getP().row(i);
      C(i, 2 * i) = 1;
    }
  A.block(lqi_mode * 2, 0, lqi_mode, lqi_mode * 3) = -C;

  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(lqi_mode * 3, lqi_mode * 3);
  if(lqi_mode == HydrusRobotModel::LQI_THREE_AXIS_MODE)
    {
      // revise matrix B:
      // 1. the third row of B should be z axis
      B.row(5) = getRobotModel().getP().row(5);

      Eigen::MatrixXd Q_tmp = q_diagonal_.asDiagonal();
      Q.block(0, 0, 6, 6) = Q_tmp.block(0, 0, 6, 6);
      Q.block(6, 6, 3, 3) = Q_tmp.block(8, 8, 3, 3);
    }
  if(lqi_mode == HydrusRobotModel::LQI_FOUR_AXIS_MODE)
    {
      // revise matrix B:
      // 1. the third row of B should be z axis
      B.row(5) = getRobotModel().getP().row(5);
      // 2. the forth row of B should be yaw axis
      B.row(7) = getRobotModel().getP().row(2);

      Q = q_diagonal_.asDiagonal();
    }

  if(control_verbose_) std::cout << "B:"  << std::endl << B << std::endl;

  Eigen::MatrixXd R_inv  = Eigen::MatrixXd::Zero(rotor_num, rotor_num);
  for(int i = 0; i < rotor_num; ++i)
    R_inv(i,i) = 1/r_[i];

  Eigen::MatrixXcd H = Eigen::MatrixXcd::Zero(lqi_mode * 6, lqi_mode * 6);
  H.block(0,0, lqi_mode * 3, lqi_mode * 3) = A.cast<std::complex<double> >();
  H.block(lqi_mode * 3, 0, lqi_mode * 3, lqi_mode * 3) = -(Q.cast<std::complex<double> >());
  H.block(0, lqi_mode * 3, lqi_mode * 3, lqi_mode * 3) = - (B * R_inv * B.transpose()).cast<std::complex<double> >();
  H.block(lqi_mode * 3, lqi_mode * 3, lqi_mode * 3, lqi_mode * 3) = - (A.transpose()).cast<std::complex<double> >();

  if(debug_verbose_) ROS_INFO("  start H eigen compute");
  //eigen solving
  ros::Time start_time = ros::Time::now();
  Eigen::ComplexEigenSolver<Eigen::MatrixXcd> ces;
  ces.compute(H);
  if(debug_verbose_) ROS_INFO("  finish H eigen compute");

  if(control_verbose_)
    ROS_INFO("h eigen time is: %f\n", ros::Time::now().toSec() - start_time.toSec());

  Eigen::MatrixXcd phy = Eigen::MatrixXcd::Zero(lqi_mode * 6, lqi_mode * 3);
  int j = 0;

  for(int i = 0; i < lqi_mode * 6; i++)
    {
      if(ces.eigenvalues()[i].real() < 0)
        {
          if(j >= lqi_mode * 3)
            {
              ROS_ERROR("nagativa sigular amount is larger");
              return false;
            }

          phy.col(j) = ces.eigenvectors().col(i);
          j++;
        }
    }

  if(j != lqi_mode * 3)
    {
      ROS_ERROR("nagativa sigular value amount is not enough");
      return false;
    }

  Eigen::MatrixXcd f = phy.block(0, 0, lqi_mode * 3, lqi_mode * 3);
  Eigen::MatrixXcd g = phy.block(lqi_mode * 3, 0, lqi_mode * 3, lqi_mode * 3);

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
      Eigen::MatrixXd A_dash = Eigen::MatrixXd::Zero(lqi_mode * 3, lqi_mode * 3);
      A_dash = A + B * K_;
      Eigen::EigenSolver<Eigen::MatrixXd> esa(A_dash);
      if(debug_verbose_) ROS_INFO("  finish A eigen compute");
      if(control_verbose_)
        std::cout << "The eigenvalues of A_hash are:" << std::endl << esa.eigenvalues() << std::endl;
    }
  return true;
}

void TransformController::initParam()
{
  nh_private_.param("control_rate", control_rate_, 15.0);
  if(verbose_) std::cout << "control_rate: " << std::setprecision(3) << control_rate_ << std::endl;

  nh_private_.param("gyro_moment_compensation", gyro_moment_compensation_, false);
  nh_private_.param("control_verbose", control_verbose_, false);
  nh_private_.param("debug_verbose", debug_verbose_, false);
  nh_private_.param("a_dash_eigen_calc_flag", a_dash_eigen_calc_flag_, false);

  /* propeller direction and lqi R */
  const int rotor_num = getRobotModel().getRotorNum();
  r_.resize(rotor_num);
  for(int i = 0; i < rotor_num; ++i) {
    std::stringstream ss;
    ss << i + 1;
    /* R */
    nh_private_.param(std::string("r") + ss.str(), r_[i], 1.0);
    if(verbose_) std::cout << std::string("r") + ss.str() << ": " << r_[i] << std::endl;
  }

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
}


void TransformController::lqi()
{
  if(!getKinematicsUpdated()) {
    if(debug_verbose_) ROS_WARN("lqi return");
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);

  /* check the thre check */
  if(debug_verbose_) ROS_WARN(" start dist thre check");
  if(!getRobotModel().stabilityMarginCheck(control_verbose_)) //[m]
    {
      ROS_ERROR("LQI: invalid pose, cannot pass the distance thresh check");
      return;
    }
  if(debug_verbose_) ROS_WARN(" finish dist thre check");

  /* check the propeller overlap */
  if(debug_verbose_) ROS_WARN(" start overlap check");
  if(!getRobotModel().overlapCheck(control_verbose_)) //[m]
    {
      ROS_ERROR("LQI: invalid pose, some propellers overlap");
      return;
    }
  if(debug_verbose_) ROS_WARN(" finish dist thre check");

  /* modelling the multilink based on the inertia assumption */
  if(debug_verbose_) ROS_WARN(" start modelling");
  if(!getRobotModel().modelling(false, control_verbose_))
    ROS_ERROR("LQI: invalid pose, can not be four axis stable, switch to three axis stable mode");

  if(debug_verbose_) ROS_WARN(" finish modelling");

  if(debug_verbose_) ROS_WARN(" start ARE calc");
  if(!hamiltonMatrixSolver())
    {
      ROS_ERROR("LQI: can not solve hamilton matrix");
      return;
    }
  if(debug_verbose_) ROS_WARN(" finish ARE calc");

  param2controller();
  if(debug_verbose_) ROS_WARN(" finish param2controller");
}


void TransformController::param2controller()
{
  aerial_robot_msgs::FourAxisGain four_axis_gain_msg;
  spinal::RollPitchYawTerms rpy_gain_msg; //for rosserial
  spinal::PMatrixPseudoInverseWithInertia p_pseudo_inverse_with_inertia_msg;

  const int rotor_num = getRobotModel().getRotorNum();
  four_axis_gain_msg.motor_num = rotor_num;
  rpy_gain_msg.motors.resize(rotor_num);
  p_pseudo_inverse_with_inertia_msg.pseudo_inverse.resize(rotor_num);

  const int lqi_mode = getRobotModel().getLqiMode();
  const Eigen::MatrixXd P_orig_pseudo_inverse = getRobotModel().getPOrigPseudoInverse();

  /* hard-coding: to avoid the violation of 16int_t range because of spinal::RollPitchYawTerms */
  /* this is rare case */
  double max_gain_thresh = 32.767;
  double max_roll_p_gain = 0, max_roll_d_gain = 0, max_pitch_p_gain = 0, max_pitch_d_gain = 0, max_yaw_d_gain = 0;
  for(int i = 0; i < rotor_num; ++i)
    {
      if(max_roll_p_gain < fabs(K_(i,0))) max_roll_p_gain = fabs(K_(i,0));
      if(max_roll_d_gain < fabs(K_(i,1))) max_roll_d_gain = fabs(K_(i,1));
      if(max_pitch_p_gain < fabs(K_(i,2))) max_pitch_p_gain = fabs(K_(i,2));
      if(max_pitch_d_gain < fabs(K_(i,3))) max_pitch_d_gain = fabs(K_(i,3));
      if(max_yaw_d_gain < fabs(K_(i,7))) max_yaw_d_gain = fabs(K_(i,7));
    }

  double roll_p_gain_scale = 1, roll_d_gain_scale = 1, pitch_p_gain_scale = 1, pitch_d_gain_scale = 1, yaw_d_gain_scale = 1;
  if(max_roll_p_gain > max_gain_thresh)
    {
      ROS_WARN("LQI: the max roll p gain violate the range of int16_t: %f", max_roll_p_gain);
      roll_p_gain_scale = max_gain_thresh / max_roll_p_gain;
    }
  if(max_roll_d_gain > max_gain_thresh)
    {
      ROS_WARN("LQI: the max roll d gain violate the range of int16_t: %f", max_roll_d_gain);
      roll_d_gain_scale = max_gain_thresh / max_roll_d_gain;
    }
  if(max_pitch_p_gain > max_gain_thresh)
    {
      ROS_WARN("LQI: the max pitch p gain violate the range of int16_t: %f", max_pitch_p_gain);
      pitch_p_gain_scale = max_gain_thresh / max_pitch_p_gain;
    }
  if(max_pitch_d_gain > max_gain_thresh)
    {
      ROS_WARN("LQI: the max pitch d gain violate the range of int16_t: %f", max_pitch_d_gain);
      pitch_d_gain_scale = max_gain_thresh / max_pitch_d_gain;
    }
  if(max_yaw_d_gain > max_gain_thresh)
    {
      ROS_WARN("LQI: the max yaw d gain violate the range of int16_t: %f", max_yaw_d_gain);
      yaw_d_gain_scale = max_gain_thresh / max_yaw_d_gain;
    }

  for(int i = 0; i < rotor_num; ++i)
    {
      /* to flight controller via rosserial */
      rpy_gain_msg.motors[i].roll_p = K_(i,0) * 1000 * roll_p_gain_scale; //scale: x 1000
      rpy_gain_msg.motors[i].roll_d = K_(i,1) * 1000 * roll_d_gain_scale;  //scale: x 1000
      rpy_gain_msg.motors[i].roll_i = K_(i, lqi_mode * 2) * 1000; //scale: x 1000

      rpy_gain_msg.motors[i].pitch_p = K_(i,2) * 1000 * pitch_p_gain_scale; //scale: x 1000
      rpy_gain_msg.motors[i].pitch_d = K_(i,3) * 1000 * pitch_d_gain_scale; //scale: x 1000
      rpy_gain_msg.motors[i].pitch_i = K_(i,lqi_mode * 2 + 1) * 1000; //scale: x 1000

      /* to aerial_robot_base, feedback */
      four_axis_gain_msg.pos_p_gain_roll.push_back(K_(i,0) * roll_p_gain_scale);
      four_axis_gain_msg.pos_d_gain_roll.push_back(K_(i,1) * roll_d_gain_scale);
      four_axis_gain_msg.pos_i_gain_roll.push_back(K_(i,lqi_mode * 2));

      four_axis_gain_msg.pos_p_gain_pitch.push_back(K_(i,2) * pitch_p_gain_scale);
      four_axis_gain_msg.pos_d_gain_pitch.push_back(K_(i,3) * pitch_d_gain_scale);
      four_axis_gain_msg.pos_i_gain_pitch.push_back(K_(i,lqi_mode * 2 + 1));

      four_axis_gain_msg.pos_p_gain_alt.push_back(K_(i,4));
      four_axis_gain_msg.pos_d_gain_alt.push_back(K_(i,5));
      four_axis_gain_msg.pos_i_gain_alt.push_back(K_(i, lqi_mode * 2 + 2));

      if(lqi_mode == HydrusRobotModel::LQI_FOUR_AXIS_MODE)
        {
          /* to flight controller via rosserial */
          rpy_gain_msg.motors[i].yaw_d = K_(i,7) * 1000 * yaw_d_gain_scale; //scale: x 1000

          /* to aerial_robot_base, feedback */
          four_axis_gain_msg.pos_p_gain_yaw.push_back(K_(i,6));
          four_axis_gain_msg.pos_d_gain_yaw.push_back(K_(i,7) * yaw_d_gain_scale);
          four_axis_gain_msg.pos_i_gain_yaw.push_back(K_(i,11));

        }
      else if(lqi_mode == HydrusRobotModel::LQI_THREE_AXIS_MODE)
        {
          rpy_gain_msg.motors[i].yaw_d = 0;

          /* to aerial_robot_base, feedback */
          four_axis_gain_msg.pos_p_gain_yaw.push_back(0.0);
          four_axis_gain_msg.pos_d_gain_yaw.push_back(0.0);
          four_axis_gain_msg.pos_i_gain_yaw.push_back(0.0);
        }

      /* the p matrix pseudo inverse and inertia */
      p_pseudo_inverse_with_inertia_msg.pseudo_inverse[i].r = P_orig_pseudo_inverse(i, 0) * 1000;
      p_pseudo_inverse_with_inertia_msg.pseudo_inverse[i].p = P_orig_pseudo_inverse(i, 1) * 1000;
      p_pseudo_inverse_with_inertia_msg.pseudo_inverse[i].y = P_orig_pseudo_inverse(i, 3) * 1000;
    }
  rpy_gain_pub_.publish(rpy_gain_msg);
  four_axis_gain_pub_.publish(four_axis_gain_msg);


  /* the multilink inertia */
  Eigen::Matrix3d inertia = getRobotModel().getInertia<Eigen::Matrix3d>();
  p_pseudo_inverse_with_inertia_msg.inertia[0] = inertia(0, 0) * 1000;
  p_pseudo_inverse_with_inertia_msg.inertia[1] = inertia(1, 1) * 1000;
  p_pseudo_inverse_with_inertia_msg.inertia[2] = inertia(2, 2) * 1000;
  p_pseudo_inverse_with_inertia_msg.inertia[3] = inertia(0, 1) * 1000;
  p_pseudo_inverse_with_inertia_msg.inertia[4] = inertia(1, 2) * 1000;
  p_pseudo_inverse_with_inertia_msg.inertia[5] = inertia(0, 2) * 1000;

  if(gyro_moment_compensation_)
    p_matrix_pseudo_inverse_inertia_pub_.publish(p_pseudo_inverse_with_inertia_msg);
}
