#include <hydrus/transform_control.h>

// TODO: LQI Q matrix -> M, R matrix -> N

TransformController::TransformController(ros::NodeHandle nh, ros::NodeHandle nhp, std::unique_ptr<HydrusRobotModel> robot_model):
  RobotModelRos(nh, nhp, std::move(robot_model)),
  nh_(nh),
  nhp_(nhp)
{
  initParam();

  //publisher
  rpy_gain_pub_ = nhp_.advertise<spinal::RollPitchYawTerms>("/rpy_gain", 1);
  four_axis_gain_pub_ = nh_.advertise<aerial_robot_msgs::FourAxisGain>("/four_axis_gain", 1);
  p_matrix_pseudo_inverse_inertia_pub_ = nh_.advertise<spinal::PMatrixPseudoInverseWithInertia>("p_matrix_pseudo_inverse_inertia", 1);

  //dynamic reconfigure server
  dynamic_reconf_func_lqi_ = boost::bind(&TransformController::cfgLQICallback, this, _1, _2);
  lqi_server_.setCallback(dynamic_reconf_func_lqi_);

  main_thread_ = std::thread(boost::bind(&TransformController::mainFunc, this));//debug

  //gains
  const int rotor_num = getRobotModel().getRotorNum();
  pitch_gains_.assign(rotor_num, PID());
  roll_gains_.assign(rotor_num, PID());
  yaw_gains_.assign(rotor_num, PID());
  z_gains_.assign(rotor_num, PID());
}

TransformController::~TransformController()
{
  main_thread_.join();
}

void TransformController::cfgLQICallback(hydrus::LQIConfig &config, uint32_t level)
{
  if(config.lqi_gain_flag)
    {
      switch(level)
        {
        case LQI_RP_P_GAIN:
          ROS_INFO_STREAM_NAMED("LQI gain generator", "LQI gain generator: change the p gain weight of roll and pitch from " << q_roll_ <<  " to "  << config.q_roll);
          q_roll_ = config.q_roll;
          q_pitch_ = config.q_roll;
          break;
        case LQI_RP_I_GAIN:
          ROS_INFO_STREAM_NAMED("LQI gain generator", "LQI gain generator: change the i gain weight of roll and pitch from " << q_roll_i_ <<  " to "  << config.q_roll_i);
          q_roll_i_ = config.q_roll_i;
          q_pitch_i_ = config.q_roll_i;
          break;
        case LQI_RP_D_GAIN:
          ROS_INFO_STREAM_NAMED("LQI gain generator", "LQI gain generator: change the d gain weight of roll and pitch from " << q_roll_d_ <<  " to "  << config.q_roll_d);
          q_roll_d_ = config.q_roll_d;
          q_pitch_d_ = config.q_roll_d;
          break;
        case LQI_Y_P_GAIN:
          ROS_INFO_STREAM_NAMED("LQI gain generator", "LQI gain generator: change the p gain weight of yaw from " << q_yaw_ <<  " to "  << config.q_yaw);
          q_yaw_ = config.q_yaw;
          break;
        case LQI_Y_I_GAIN:
          ROS_INFO_STREAM_NAMED("LQI gain generator", "LQI gain generator: change the i gain weight of yaw from " << q_yaw_i_ <<  " to "  << config.q_yaw_i);
          q_yaw_i_ = config.q_yaw_i;
          break;
        case LQI_Y_D_GAIN:
          ROS_INFO_STREAM_NAMED("LQI gain generator", "LQI gain generator: change the d gain weight of yaw from " << q_yaw_d_ <<  " to "  << config.q_yaw_d);
          q_yaw_d_ = config.q_yaw_d;
          break;
        case LQI_Z_P_GAIN:
          ROS_INFO_STREAM_NAMED("LQI gain generator", "LQI gain generator: change the p gain weight of z from " << q_z_ <<  " to "  << config.q_z);
          q_z_ = config.q_z;
          break;
        case LQI_Z_I_GAIN:
          ROS_INFO_STREAM_NAMED("LQI gain generator", "LQI gain generator: change the i gain weight of z from " << q_z_i_ <<  " to "  << config.q_z_i);
          q_z_i_ = config.q_z_i;
          break;
        case LQI_Z_D_GAIN:
          ROS_INFO_STREAM_NAMED("LQI gain generator", "LQI gain generator: change the d gain weight of z from " << q_z_d_ <<  " to "  << config.q_z_d);
          q_z_d_ = config.q_z_d;
          break;
        default :
          break;
        }
    }
}

void TransformController::mainFunc()
{
  ros::Rate loop_rate(control_rate_);

  while(ros::ok())
    {
      if(updateRobotModel())
        {
          if(optimalGain())
            param2controller();
          else
            ROS_ERROR_NAMED("LQI gain generator", "LQI gain generator: can not solve hamilton matrix");
        }
      else
        {
          resetGain();
        }

      loop_rate.sleep();
    }
}

bool TransformController::updateRobotModel()
{
  lqi_mode_ = getRobotModel().getWrenchDof();

  if(!getKinematicsUpdated())
    {
      ROS_DEBUG_NAMED("LQI gain generator", "LQI gain generator: robot model is not initiliazed");
      return false;
    }

  if(!getRobotModel().stabilityCheck(verbose_))
    {
      ROS_ERROR_NAMED("LQI gain generator", "LQI gain generator: invalid pose, stability is invalid");
      if(getRobotModel().getWrenchDof() == 4 && getRobotModel().getWrenchMarginRollPitchMin() > getRobotModel().getWrenchMarginRollPitchMinThre())
        {
          ROS_WARN_NAMED("LQI gain generator", "LQI gain generator: change to three axis stable mode");
          lqi_mode_ = 3;
          return true;
        }

      return false;
    }
  return true;
}

bool TransformController::optimalGain()
{
  // referece:
  // M, Zhao, et.al, "Transformable multirotor with two-dimensional multilinks: modeling, control, and whole-body aerial manipulation"
  // Sec. 3.2

  const int rotor_num = getRobotModel().getRotorNum();

  Eigen::MatrixXd P = getRobotModel().calcWrenchMatrixOnCoG();
  Eigen::MatrixXd P_dash = Eigen::MatrixXd::Zero(lqi_mode_, rotor_num);

  P_dash.row(0) = P.row(2) / getRobotModel().getMass(); // z
  P_dash.bottomRows(lqi_mode_ - 1) = (getRobotModel().getInertia<Eigen::Matrix3d>().inverse() * P.bottomRows(3)).topRows(lqi_mode_ - 1); // roll, pitch, yaw

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(lqi_mode_ * 3, lqi_mode_ * 3);
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(lqi_mode_ * 3, rotor_num);
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(lqi_mode_, lqi_mode_ * 3);
  for(int i = 0; i < lqi_mode_; i++)
    {
      A(2 * i, 2 * i + 1) = 1;
      B.row(2 * i + 1) = P_dash.row(i);
      C(i, 2 * i) = 1;
    }
  A.block(lqi_mode_ * 2, 0, lqi_mode_, lqi_mode_ * 3) = -C;

  ROS_DEBUG_STREAM_NAMED("LQI gain generator", "LQI gain generator: B: \n"  <<  B );

  Eigen::VectorXd q_diagonals(lqi_mode_ * 3);
  if(lqi_mode_ == 3)
    q_diagonals << q_z_, q_z_d_, q_roll_, q_roll_d_, q_pitch_, q_pitch_d_, q_z_i_, q_roll_i_, q_pitch_i_;
  else
    q_diagonals << q_z_, q_z_d_, q_roll_, q_roll_d_, q_pitch_, q_pitch_d_, q_yaw_, q_yaw_d_, q_z_i_, q_roll_i_, q_pitch_i_, q_yaw_i_;

  Eigen::MatrixXd Q = q_diagonals.asDiagonal();

  Eigen::MatrixXd R  = Eigen::MatrixXd::Zero(rotor_num, rotor_num);
  for(int i = 0; i < rotor_num; ++i) R(i,i) = r_[i];

  /* solve continuous-time algebraic Ricatti equation */
  double t = ros::Time::now().toSec();

  if(K_.cols() != lqi_mode_ * 3)
    {
      resetGain(); // four axis -> three axis and vice versa
    }

  bool use_kleinman_method = true;
  if(K_.cols() == 0 || K_.rows() == 0)
    {
      ROS_DEBUG_STREAM_NAMED("LQI gain generator",  "LQI gain generator: do not use kleinman method");
      use_kleinman_method = false;
    }
  if(!control_utils::care(A, B, R, Q, K_, use_kleinman_method))
    {
      ROS_ERROR_STREAM_NAMED("LQI gain generator",  "LQI gain generator: error in solver of continuous-time algebraic riccati equation");
      return false;
    }

  ROS_DEBUG_STREAM_NAMED("LQI gain generator",  "LQI gain generator: CARE: %f sec" << ros::Time::now().toSec() - t);
  ROS_DEBUG_STREAM_NAMED("LQI gain generator",  "LQI gain generator:  K \n" <<  K_);

  // convert to gains
  for(int i = 0; i < rotor_num; ++i)
    {
      z_gains_.at(i).p = K_(i,0);
      z_gains_.at(i).i = K_(i, lqi_mode_ * 2);
      z_gains_.at(i).d = K_(i,1);

      roll_gains_.at(i).p = K_(i,2);
      roll_gains_.at(i).i = K_(i, lqi_mode_ * 2 + 1);
      roll_gains_.at(i).d = K_(i,3);

      pitch_gains_.at(i).p = K_(i,4);
      pitch_gains_.at(i).i = K_(i, lqi_mode_ * 2 + 2);
      pitch_gains_.at(i).d = K_(i,5);

      if(lqi_mode_ == 3)
        {
          yaw_gains_.at(i).p = 0;
          yaw_gains_.at(i).i = 0;
          yaw_gains_.at(i).d = 0;
        }

      if(lqi_mode_ == 4)
        {
          yaw_gains_.at(i).p = K_(i,6);
          yaw_gains_.at(i).i = K_(i, lqi_mode_ * 2 + 3);
          yaw_gains_.at(i).d = K_(i,7);
        }
    }

  // compensation for gyro moment
  p_mat_pseudo_inv_ = aerial_robot_model::pseudoinverse(P.middleRows(2, lqi_mode_));
  return true;
}

void TransformController::initParam()
{
  nhp_.param("verbose", verbose_, false);
  nhp_.param("control_rate", control_rate_, 15.0);
  if(verbose_) std::cout << "control_rate: " << std::setprecision(3) << control_rate_ << std::endl;

  nhp_.param("gyro_moment_compensation", gyro_moment_compensation_, false);

  /* propeller direction and lqi R */
  const int rotor_num = getRobotModel().getRotorNum();
  r_.resize(rotor_num);
  for(int i = 0; i < rotor_num; ++i) {
    std::stringstream ss;
    ss << i + 1;
    /* R */
    nhp_.param(std::string("r") + ss.str(), r_[i], 1.0);
    if(verbose_) std::cout << std::string("r") + ss.str() << ": " << r_[i] << std::endl;
  }

  nhp_.param ("q_roll", q_roll_, 1.0);
  if(verbose_) std::cout << "Q: q_roll: " << std::setprecision(3) << q_roll_ << std::endl;
  nhp_.param ("q_roll_d", q_roll_d_, 1.0);
  if(verbose_) std::cout << "Q: q_roll_d: " << std::setprecision(3) << q_roll_d_ << std::endl;
  nhp_.param ("q_pitch", q_pitch_, 1.0);
  if(verbose_) std::cout << "Q: q_pitch: " << std::setprecision(3) << q_pitch_ << std::endl;
  nhp_.param ("q_pitch_d", q_pitch_d_,  1.0);
  if(verbose_) std::cout << "Q: q_pitch_d: " << std::setprecision(3) << q_pitch_d_ << std::endl;
  nhp_.param ("q_yaw", q_yaw_, 1.0);
  if(verbose_) std::cout << "Q: q_yaw: " << std::setprecision(3) << q_yaw_ << std::endl;
  nhp_.param ("q_yaw_d", q_yaw_d_, 1.0);
  if(verbose_) std::cout << "Q: q_yaw_d: " << std::setprecision(3) << q_yaw_d_ << std::endl;
  nhp_.param ("q_z", q_z_, 1.0);
  if(verbose_) std::cout << "Q: q_z: " << std::setprecision(3) << q_z_ << std::endl;
  nhp_.param ("q_z_d", q_z_d_, 1.0);
  if(verbose_) std::cout << "Q: q_z_d: " << std::setprecision(3) << q_z_d_ << std::endl;

  nhp_.param ("q_roll_i", q_roll_i_, 1.0);
  if(verbose_) std::cout << "Q: q_roll_i: " << std::setprecision(3) << q_roll_i_ << std::endl;
  nhp_.param ("q_pitch_i", q_pitch_i_, 1.0);
  if(verbose_) std::cout << "Q: q_pitch_i: " << std::setprecision(3) << q_pitch_i_ << std::endl;
  nhp_.param ("q_yaw_i", q_yaw_i_, 1.0);
  if(verbose_) std::cout << "Q: q_yaw_i: " << std::setprecision(3) << q_yaw_i_ << std::endl;
  nhp_.param ("q_z_i", q_z_i_, 1.0);
  if(verbose_) std::cout << "Q: q_z_i: " << std::setprecision(3) << q_z_i_ << std::endl;
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

  /* hard-coding: to avoid the violation of 16int_t range because of spinal::RollPitchYawTerms */
  /* this is rare case */
  double max_gain_thresh = 32.767;
  double max_roll_p_gain = 0, max_roll_d_gain = 0, max_pitch_p_gain = 0, max_pitch_d_gain = 0, max_yaw_d_gain = 0;
  for(int i = 0; i < rotor_num; ++i)
    {
      if(max_roll_p_gain < fabs(roll_gains_.at(i).p)) max_roll_p_gain = fabs(roll_gains_.at(i).p);
      if(max_roll_d_gain < fabs(roll_gains_.at(i).d)) max_roll_d_gain = fabs(roll_gains_.at(i).d);
      if(max_pitch_p_gain < fabs(pitch_gains_.at(i).p)) max_pitch_p_gain = fabs(pitch_gains_.at(i).p);
      if(max_pitch_d_gain < fabs(pitch_gains_.at(i).d)) max_pitch_d_gain = fabs(pitch_gains_.at(i).d);
      if(max_yaw_d_gain < fabs(yaw_gains_.at(i).d)) max_yaw_d_gain = fabs(yaw_gains_.at(i).d);
    }

  double roll_p_gain_scale = 1, roll_d_gain_scale = 1, pitch_p_gain_scale = 1, pitch_d_gain_scale = 1, yaw_d_gain_scale = 1;
  if(max_roll_p_gain > max_gain_thresh)
    {
      ROS_WARN_STREAM_NAMED("LQI gain generator", "LQI gain generator: the max roll p gain violate the range of int16_t: " << max_roll_p_gain);
      roll_p_gain_scale = max_gain_thresh / max_roll_p_gain;
    }
  if(max_roll_d_gain > max_gain_thresh)
    {
      ROS_WARN_STREAM_NAMED("LQI gain generator", "LQI gain generator: the max roll d gain violate the range of int16_t: " << max_roll_d_gain);
      roll_d_gain_scale = max_gain_thresh / max_roll_d_gain;
    }
  if(max_pitch_p_gain > max_gain_thresh)
    {
      ROS_WARN_STREAM_NAMED("LQI gain generator", "LQI gain generator: the max pitch p gain violate the range of int16_t: " << max_pitch_p_gain);
      pitch_p_gain_scale = max_gain_thresh / max_pitch_p_gain;
    }
  if(max_pitch_d_gain > max_gain_thresh)
    {
      ROS_WARN_STREAM_NAMED("LQI gain generator", "LQI gain generator: the max pitch d gain violate the range of int16_t: " << max_pitch_d_gain);
      pitch_d_gain_scale = max_gain_thresh / max_pitch_d_gain;
    }
  if(max_yaw_d_gain > max_gain_thresh)
    {
      ROS_WARN_STREAM_NAMED("LQI gain generator", "LQI gain generator: the max yaw d gain violate the range of int16_t: " << max_yaw_d_gain);
      yaw_d_gain_scale = max_gain_thresh / max_yaw_d_gain;
    }

  for(int i = 0; i < rotor_num; ++i)
    {
      /* to flight controller via rosserial scaling by 1000 */
      rpy_gain_msg.motors[i].roll_p = roll_gains_.at(i).p * 1000 * roll_p_gain_scale;
      rpy_gain_msg.motors[i].roll_i = roll_gains_.at(i).i * 1000;
      rpy_gain_msg.motors[i].roll_d = roll_gains_.at(i).d * 1000 * roll_d_gain_scale;

      rpy_gain_msg.motors[i].pitch_p = pitch_gains_.at(i).p * 1000 * pitch_p_gain_scale;
      rpy_gain_msg.motors[i].pitch_i = pitch_gains_.at(i).i * 1000;
      rpy_gain_msg.motors[i].pitch_d = pitch_gains_.at(i).d * 1000 * pitch_d_gain_scale;

      rpy_gain_msg.motors[i].yaw_d = yaw_gains_.at(i).d * 1000 * yaw_d_gain_scale; //scale: x 1000

      /* to aerial_robot_base, feedback */
      four_axis_gain_msg.pos_p_gain_roll.push_back(roll_gains_.at(i).p * roll_p_gain_scale);
      four_axis_gain_msg.pos_i_gain_roll.push_back(roll_gains_.at(i).i);
      four_axis_gain_msg.pos_d_gain_roll.push_back(roll_gains_.at(i).d * roll_d_gain_scale);

      four_axis_gain_msg.pos_p_gain_pitch.push_back(pitch_gains_.at(i).p * pitch_p_gain_scale);
      four_axis_gain_msg.pos_i_gain_pitch.push_back(pitch_gains_.at(i).i);
      four_axis_gain_msg.pos_d_gain_pitch.push_back(pitch_gains_.at(i).d * pitch_d_gain_scale);

      four_axis_gain_msg.pos_p_gain_alt.push_back(z_gains_.at(i).p);
      four_axis_gain_msg.pos_i_gain_alt.push_back(z_gains_.at(i).i);
      four_axis_gain_msg.pos_d_gain_alt.push_back(z_gains_.at(i).d);

      four_axis_gain_msg.pos_p_gain_yaw.push_back(yaw_gains_.at(i).p);
      four_axis_gain_msg.pos_i_gain_yaw.push_back(yaw_gains_.at(i).i);
      four_axis_gain_msg.pos_d_gain_yaw.push_back(yaw_gains_.at(i).d * yaw_d_gain_scale);

      /* the p matrix pseudo inverse and inertia */
      p_pseudo_inverse_with_inertia_msg.pseudo_inverse[i].r = p_mat_pseudo_inv_(i, 1) * 1000;
      p_pseudo_inverse_with_inertia_msg.pseudo_inverse[i].p = p_mat_pseudo_inv_(i, 2) * 1000;
      if(lqi_mode_ = 4)
        p_pseudo_inverse_with_inertia_msg.pseudo_inverse[i].y = p_mat_pseudo_inv_(i, 3) * 1000;
      else
        p_pseudo_inverse_with_inertia_msg.pseudo_inverse[i].y = 0;
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
