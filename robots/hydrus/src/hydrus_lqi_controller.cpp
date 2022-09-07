#include <hydrus/hydrus_lqi_controller.h>

using namespace aerial_robot_control;

HydrusLQIController::HydrusLQIController():
  target_roll_(0), target_pitch_(0), candidate_yaw_term_(0)
{
  lqi_roll_pitch_weight_.setZero();
  lqi_yaw_weight_.setZero();
  lqi_z_weight_.setZero();
}


void HydrusLQIController::initialize(ros::NodeHandle nh,
                                     ros::NodeHandle nhp,
                                     boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                     boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                     boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                     double ctrl_loop_rate)
{
  PoseLinearController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);

  rosParamInit();

  hydrus_robot_model_ = boost::dynamic_pointer_cast<HydrusRobotModel>(robot_model);

  //publisher
  rpy_gain_pub_ = nh_.advertise<spinal::RollPitchYawTerms>("rpy/gain", 1);
  flight_cmd_pub_ = nh_.advertise<spinal::FourAxisCommand>("four_axes/command", 1);
  four_axis_gain_pub_ = nh_.advertise<aerial_robot_msgs::FourAxisGain>("debug/four_axes/gain", 1);
  p_matrix_pseudo_inverse_inertia_pub_ = nh_.advertise<spinal::PMatrixPseudoInverseWithInertia>("p_matrix_pseudo_inverse_inertia", 1);

  //dynamic reconfigure server
  ros::NodeHandle control_nh(nh_, "controller");
  lqi_server_ = boost::make_shared<dynamic_reconfigure::Server<hydrus::LQIConfig> >(ros::NodeHandle(control_nh, "lqi"));
  dynamic_reconf_func_lqi_ = boost::bind(&HydrusLQIController::cfgLQICallback, this, _1, _2);
  lqi_server_->setCallback(dynamic_reconf_func_lqi_);
  gain_generator_thread_ = std::thread(boost::bind(&HydrusLQIController::gainGeneratorFunc, this));

  //gains
  pitch_gains_.resize(motor_num_, Eigen::Vector3d(0,0,0));
  roll_gains_.resize(motor_num_, Eigen::Vector3d(0,0,0));
  z_gains_.resize(motor_num_, Eigen::Vector3d(0,0,0));
  yaw_gains_.resize(motor_num_, Eigen::Vector3d(0,0,0));

  //message
  target_base_thrust_.resize(motor_num_);
  pid_msg_.z.total.resize(motor_num_);
  pid_msg_.z.p_term.resize(motor_num_);
  pid_msg_.z.i_term.resize(motor_num_);
  pid_msg_.z.d_term.resize(motor_num_);
  pid_msg_.yaw.total.resize(motor_num_);
  pid_msg_.yaw.p_term.resize(motor_num_);
  pid_msg_.yaw.i_term.resize(motor_num_);
  pid_msg_.yaw.d_term.resize(motor_num_);
}

HydrusLQIController::~HydrusLQIController()
{
  gain_generator_thread_.join();
}


void HydrusLQIController::sendCmd()
{
  PoseLinearController::sendCmd();

  sendFourAxisCommand();
}

void HydrusLQIController::sendFourAxisCommand()
{
  spinal::FourAxisCommand flight_command_data;
  flight_command_data.angles[0] = target_roll_;
  flight_command_data.angles[1] = target_pitch_;
  flight_command_data.angles[2] = candidate_yaw_term_;
  flight_command_data.base_thrust = target_base_thrust_;
  flight_cmd_pub_.publish(flight_command_data);
}

void HydrusLQIController::controlCore()
{
  PoseLinearController::controlCore();

  tf::Vector3 target_acc_w(pid_controllers_.at(X).result(),
                           pid_controllers_.at(Y).result(),
                           pid_controllers_.at(Z).result());
  tf::Vector3 target_acc_dash = (tf::Matrix3x3(tf::createQuaternionFromYaw(rpy_.z()))).inverse() * target_acc_w;

  target_pitch_ = target_acc_dash.x() / aerial_robot_estimation::G;
  target_roll_ = -target_acc_dash.y() / aerial_robot_estimation::G;

  Eigen::VectorXd target_thrust_z_term = Eigen::VectorXd::Zero(motor_num_);
  for(int i = 0; i < motor_num_; i++)
    {
      double p_term = z_gains_.at(i)[0] * pid_controllers_.at(Z).getErrP();
      double i_term = z_gains_.at(i)[1] * pid_controllers_.at(Z).getErrI();
      double d_term = z_gains_.at(i)[2] * pid_controllers_.at(Z).getErrD();
      target_thrust_z_term(i) = p_term + i_term + d_term;
      pid_msg_.z.p_term.at(i) = p_term;
      pid_msg_.z.i_term.at(i) = i_term;
      pid_msg_.z.d_term.at(i) = d_term;
    }

  // constraint z (also  I term)
  int index;
  double max_term = target_thrust_z_term.cwiseAbs().maxCoeff(&index);
  double residual = max_term - pid_controllers_.at(Z).getLimitSum();
  if(residual > 0)
    {
      pid_controllers_.at(Z).setErrI(pid_controllers_.at(Z).getPrevErrI());
      target_thrust_z_term *= (1 - residual / max_term);
    }

  for(int i = 0; i < motor_num_; i++)
    {
      target_base_thrust_.at(i) = target_thrust_z_term(i);
      pid_msg_.z.total.at(i) =  target_thrust_z_term(i);
    }

  allocateYawTerm();
}

void HydrusLQIController::allocateYawTerm()
{
  Eigen::VectorXd target_thrust_yaw_term = Eigen::VectorXd::Zero(motor_num_);
  for(int i = 0; i < motor_num_; i++)
    {
      double p_term = yaw_gains_.at(i)[0] * pid_controllers_.at(YAW).getErrP();
      double i_term = yaw_gains_.at(i)[1] * pid_controllers_.at(YAW).getErrI();
      double d_term = yaw_gains_.at(i)[2] * pid_controllers_.at(YAW).getErrD();
      target_thrust_yaw_term(i) = p_term + i_term + d_term;
      pid_msg_.yaw.p_term.at(i) = p_term;
      pid_msg_.yaw.i_term.at(i) = i_term;
      pid_msg_.yaw.d_term.at(i) = d_term;
    }
  // constraint yaw (also  I term)
  int index;
  double max_term = target_thrust_yaw_term.cwiseAbs().maxCoeff(&index);
  double residual = max_term - pid_controllers_.at(YAW).getLimitSum();
  if(residual > 0)
    {
      pid_controllers_.at(YAW).setErrI(pid_controllers_.at(YAW).getPrevErrI());
      target_thrust_yaw_term *= (1 - residual / max_term);
    }

  // special process for yaw since the bandwidth between PC and spinal
  double max_yaw_scale = 0; // for reconstruct yaw control term in spinal
  for(int i = 0; i < motor_num_; i++)
    {
      pid_msg_.yaw.total.at(i) =  target_thrust_yaw_term(i);

      if(yaw_gains_[i][2] > max_yaw_scale)
        {
          max_yaw_scale = yaw_gains_[i][2];
          candidate_yaw_term_ = target_thrust_yaw_term(i);
        }
    }
}

void HydrusLQIController::gainGeneratorFunc()
{
  double rate;
  ros::NodeHandle control_nh(nh_, "controller");
  ros::NodeHandle lqi_nh(control_nh, "lqi");
  lqi_nh.param("gain_generate_rate", rate, 15.0);
  ros::Rate loop_rate(rate);

  while(ros::ok())
    {
      if(checkRobotModel())
        {
          if(optimalGain())
            {
              clampGain();
              publishGain();
            }
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

bool HydrusLQIController::checkRobotModel()
{
  lqi_mode_ = hydrus_robot_model_->getWrenchDof();

  if(!robot_model_->initialized())
    {
      ROS_DEBUG_NAMED("LQI gain generator", "LQI gain generator: robot model is not initiliazed");
      return false;
    }


  if(!robot_model_->stabilityCheck(verbose_))
    {
      ROS_ERROR_NAMED("LQI gain generator", "LQI gain generator: invalid pose, stability is invalid");
      if(hydrus_robot_model_->getWrenchDof() == 4 && hydrus_robot_model_->getFeasibleControlRollPitchMin() > hydrus_robot_model_->getFeasibleControlRollPitchMinThre())
        {
          ROS_WARN_NAMED("LQI gain generator", "LQI gain generator: change to three axis stable mode");
          lqi_mode_ = 3;
          return true;
        }

      return false;
    }
  return true;
}

bool HydrusLQIController::optimalGain()
{
  // referece:
  // M, Zhao, et.al, "Transformable multirotor with two-dimensional multilinks: modeling, control, and whole-body aerial manipulation"
  // Sec. 3.2

  Eigen::MatrixXd P = robot_model_->calcWrenchMatrixOnCoG();
  Eigen::MatrixXd P_dash = Eigen::MatrixXd::Zero(lqi_mode_, motor_num_);
  Eigen::MatrixXd inertia = robot_model_->getInertia<Eigen::Matrix3d>();
  P_dash.row(0) = P.row(2) / robot_model_->getMass(); // z
  P_dash.bottomRows(lqi_mode_ - 1) = (inertia.inverse() * P.bottomRows(3)).topRows(lqi_mode_ - 1); // roll, pitch, yaw

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(lqi_mode_ * 3, lqi_mode_ * 3);
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(lqi_mode_ * 3, motor_num_);
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
    {
      q_diagonals << lqi_z_weight_(0), lqi_z_weight_(2), lqi_roll_pitch_weight_(0), lqi_roll_pitch_weight_(2), lqi_roll_pitch_weight_(0), lqi_roll_pitch_weight_(2), lqi_z_weight_(1), lqi_roll_pitch_weight_(1), lqi_roll_pitch_weight_(1);
    }
  else
    {
      q_diagonals << lqi_z_weight_(0), lqi_z_weight_(2), lqi_roll_pitch_weight_(0), lqi_roll_pitch_weight_(2), lqi_roll_pitch_weight_(0), lqi_roll_pitch_weight_(2), lqi_yaw_weight_(0), lqi_yaw_weight_(2), lqi_z_weight_(1), lqi_roll_pitch_weight_(1), lqi_roll_pitch_weight_(1), lqi_yaw_weight_(1);
    }
  Eigen::MatrixXd Q = q_diagonals.asDiagonal();

  Eigen::MatrixXd R  = Eigen::MatrixXd::Zero(motor_num_, motor_num_);
  for(int i = 0; i < motor_num_; ++i) R(i,i) = r_.at(i);

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


  for(int i = 0; i < motor_num_; ++i)
    {
      roll_gains_.at(i) = Eigen::Vector3d(-K_(i,2),  K_(i, lqi_mode_ * 2 + 1), -K_(i,3));
      pitch_gains_.at(i) = Eigen::Vector3d(-K_(i,4), K_(i, lqi_mode_ * 2 + 2), -K_(i,5));
      z_gains_.at(i) = Eigen::Vector3d(-K_(i,0), K_(i, lqi_mode_ * 2), -K_(i,1));
      if(lqi_mode_ == 4) yaw_gains_.at(i) = Eigen::Vector3d(-K_(i,6), K_(i, lqi_mode_ * 2 + 3), -K_(i,7));
      else yaw_gains_.at(i).setZero();
    }

  // compensation for gyro moment
  p_mat_pseudo_inv_ = aerial_robot_model::pseudoinverse(P.middleRows(2, lqi_mode_));
  return true;
}

void HydrusLQIController::clampGain()
{
  /* avoid the violation of 16int_t range because of spinal::RollPitchYawTerms */
  double max_gain_thresh = 32.767;
  double max_roll_p_gain = 0, max_roll_d_gain = 0, max_pitch_p_gain = 0, max_pitch_d_gain = 0, max_yaw_d_gain = 0;
  for(int i = 0; i < motor_num_; ++i)
    {
      if(max_roll_p_gain < fabs(roll_gains_.at(i)[0])) max_roll_p_gain = fabs(roll_gains_.at(i)[0]);
      if(max_roll_d_gain < fabs(roll_gains_.at(i)[2])) max_roll_d_gain = fabs(roll_gains_.at(i)[2]);
      if(max_pitch_p_gain < fabs(pitch_gains_.at(i)[0])) max_pitch_p_gain = fabs(pitch_gains_.at(i)[0]);
      if(max_pitch_d_gain < fabs(pitch_gains_.at(i)[2])) max_pitch_d_gain = fabs(pitch_gains_.at(i)[2]);
      if(max_yaw_d_gain < fabs(yaw_gains_.at(i)[2])) max_yaw_d_gain = fabs(yaw_gains_.at(i)[2]);
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

  for(int i = 0; i < motor_num_; ++i)
    {
      roll_gains_.at(i)[0] *= roll_p_gain_scale;
      roll_gains_.at(i)[2] *= roll_d_gain_scale;

      pitch_gains_.at(i)[0] *= pitch_p_gain_scale;
      pitch_gains_.at(i)[2] *= pitch_d_gain_scale;

      yaw_gains_.at(i)[2] *= yaw_d_gain_scale;
    }
}

void HydrusLQIController::rosParamInit()
{
  ros::NodeHandle control_nh(nh_, "controller");
  ros::NodeHandle lqi_nh(control_nh, "lqi");
  getParam<bool>(lqi_nh, "gyro_moment_compensation", gyro_moment_compensation_, false);
  getParam<bool>(lqi_nh, "clamp_gain", clamp_gain_, true);

  /* propeller direction and lqi R */
  r_.resize(motor_num_); // motor_num is not set
  for(int i = 0; i < robot_model_->getRotorNum(); ++i) {
    std::stringstream ss;
    ss << i + 1;
    /* R */
    getParam<double>(lqi_nh, std::string("r") + ss.str(), r_.at(i), 1.0);
  }

  getParam<double>(lqi_nh, "roll_pitch_p", lqi_roll_pitch_weight_[0], 1.0);
  getParam<double>(lqi_nh, "roll_pitch_i", lqi_roll_pitch_weight_[1], 1.0);
  getParam<double>(lqi_nh, "roll_pitch_d", lqi_roll_pitch_weight_[2], 1.0);
  getParam<double>(lqi_nh, "yaw_p", lqi_yaw_weight_[0], 1.0);
  getParam<double>(lqi_nh, "yaw_i", lqi_yaw_weight_[1], 1.0);
  getParam<double>(lqi_nh, "yaw_d", lqi_yaw_weight_[2], 1.0);
  getParam<double>(lqi_nh, "z_p", lqi_z_weight_[0], 1.0);
  getParam<double>(lqi_nh, "z_i", lqi_z_weight_[1], 1.0);
  getParam<double>(lqi_nh, "z_d", lqi_z_weight_[2], 1.0);
}

void HydrusLQIController::publishGain()
{
  aerial_robot_msgs::FourAxisGain four_axis_gain_msg;
  spinal::RollPitchYawTerms rpy_gain_msg; // to spinal
  spinal::PMatrixPseudoInverseWithInertia p_pseudo_inverse_with_inertia_msg; // to spinal

  rpy_gain_msg.motors.resize(motor_num_);
  p_pseudo_inverse_with_inertia_msg.pseudo_inverse.resize(motor_num_);

  for(int i = 0; i < motor_num_; ++i)
    {
      four_axis_gain_msg.roll_p_gain.push_back(roll_gains_.at(i)[0]);
      four_axis_gain_msg.roll_i_gain.push_back(roll_gains_.at(i)[1]);
      four_axis_gain_msg.roll_d_gain.push_back(roll_gains_.at(i)[2]);

      four_axis_gain_msg.pitch_p_gain.push_back(pitch_gains_.at(i)[0]);
      four_axis_gain_msg.pitch_i_gain.push_back(pitch_gains_.at(i)[1]);
      four_axis_gain_msg.pitch_d_gain.push_back(pitch_gains_.at(i)[2]);

      four_axis_gain_msg.yaw_p_gain.push_back(yaw_gains_.at(i)[0]);
      four_axis_gain_msg.yaw_i_gain.push_back(yaw_gains_.at(i)[1]);
      four_axis_gain_msg.yaw_d_gain.push_back(yaw_gains_.at(i)[2]);

      four_axis_gain_msg.z_p_gain.push_back(z_gains_.at(i)[0]);
      four_axis_gain_msg.z_i_gain.push_back(z_gains_.at(i)[1]);
      four_axis_gain_msg.z_d_gain.push_back(z_gains_.at(i)[2]);

      /* to flight controller via rosserial scaling by 1000 */
      rpy_gain_msg.motors[i].roll_p = roll_gains_.at(i)[0] * 1000;
      rpy_gain_msg.motors[i].roll_i = roll_gains_.at(i)[1] * 1000;
      rpy_gain_msg.motors[i].roll_d = roll_gains_.at(i)[2] * 1000;

      rpy_gain_msg.motors[i].pitch_p = pitch_gains_.at(i)[0] * 1000;
      rpy_gain_msg.motors[i].pitch_i = pitch_gains_.at(i)[1] * 1000;
      rpy_gain_msg.motors[i].pitch_d = pitch_gains_.at(i)[2] * 1000;

      rpy_gain_msg.motors[i].yaw_d = yaw_gains_.at(i)[2] * 1000;

      /* the p matrix pseudo inverse and inertia */
      p_pseudo_inverse_with_inertia_msg.pseudo_inverse[i].r = p_mat_pseudo_inv_(i, 1) * 1000;
      p_pseudo_inverse_with_inertia_msg.pseudo_inverse[i].p = p_mat_pseudo_inv_(i, 2) * 1000;
      if(lqi_mode_ == 4)
        p_pseudo_inverse_with_inertia_msg.pseudo_inverse[i].y = p_mat_pseudo_inv_(i, 3) * 1000;
      else
        p_pseudo_inverse_with_inertia_msg.pseudo_inverse[i].y = 0;
    }
  rpy_gain_pub_.publish(rpy_gain_msg);
  four_axis_gain_pub_.publish(four_axis_gain_msg);


  /* the multilink inertia */
  Eigen::Matrix3d inertia = robot_model_->getInertia<Eigen::Matrix3d>();
  p_pseudo_inverse_with_inertia_msg.inertia[0] = inertia(0, 0) * 1000;
  p_pseudo_inverse_with_inertia_msg.inertia[1] = inertia(1, 1) * 1000;
  p_pseudo_inverse_with_inertia_msg.inertia[2] = inertia(2, 2) * 1000;
  p_pseudo_inverse_with_inertia_msg.inertia[3] = inertia(0, 1) * 1000;
  p_pseudo_inverse_with_inertia_msg.inertia[4] = inertia(1, 2) * 1000;
  p_pseudo_inverse_with_inertia_msg.inertia[5] = inertia(0, 2) * 1000;

  if(gyro_moment_compensation_)
    p_matrix_pseudo_inverse_inertia_pub_.publish(p_pseudo_inverse_with_inertia_msg);
}

void HydrusLQIController::cfgLQICallback(hydrus::LQIConfig &config, uint32_t level)
{
  using Levels = aerial_robot_msgs::DynamicReconfigureLevels;
  if(config.lqi_flag)
    {
      switch(level)
        {
        case Levels::RECONFIGURE_LQI_ROLL_PITCH_P:
          ROS_INFO_STREAM_NAMED("LQI gain generator", "LQI gain generator: change the p gain weight of roll and pitch from " << lqi_roll_pitch_weight_.x() <<  " to "  << config.roll_pitch_p);
          lqi_roll_pitch_weight_.x() = config.roll_pitch_p;
          break;
        case Levels::RECONFIGURE_LQI_ROLL_PITCH_I:
          ROS_INFO_STREAM_NAMED("LQI gain generator", "LQI gain generator: change the i gain weight of roll and pitch from " << lqi_roll_pitch_weight_.y() <<  " to "  << config.roll_pitch_i);
          lqi_roll_pitch_weight_.y() = config.roll_pitch_i;
          break;
        case Levels::RECONFIGURE_LQI_ROLL_PITCH_D:
          ROS_INFO_STREAM_NAMED("LQI gain generator", "LQI gain generator: change the d gain weight of roll and pitch from " << lqi_roll_pitch_weight_.z() <<  " to "  << config.roll_pitch_d);
          lqi_roll_pitch_weight_.z() = config.roll_pitch_d;
          break;
        case Levels::RECONFIGURE_LQI_YAW_P:
          ROS_INFO_STREAM_NAMED("LQI gain generator", "LQI gain generator: change the p gain weight of yaw from " << lqi_yaw_weight_.x() <<  " to "  << config.yaw_p);
          lqi_yaw_weight_.x() = config.yaw_p;
          break;
        case Levels::RECONFIGURE_LQI_YAW_I:
          ROS_INFO_STREAM_NAMED("LQI gain generator", "LQI gain generator: change the i gain weight of yaw from " << lqi_yaw_weight_.y() <<  " to "  << config.yaw_i);
          lqi_yaw_weight_.y() = config.yaw_i;
          break;
        case Levels::RECONFIGURE_LQI_YAW_D:
          ROS_INFO_STREAM_NAMED("LQI gain generator", "LQI gain generator: change the d gain weight of yaw from " << lqi_yaw_weight_.z() <<  " to "  << config.yaw_d);
          lqi_yaw_weight_.z() = config.yaw_d;
          break;
        case Levels::RECONFIGURE_LQI_Z_P:
          ROS_INFO_STREAM_NAMED("LQI gain generator", "LQI gain generator: change the p gain weight of z from " << lqi_z_weight_.x() <<  " to "  << config.z_p);
          lqi_z_weight_.x() = config.z_p;
          break;
        case Levels::RECONFIGURE_LQI_Z_I:
          ROS_INFO_STREAM_NAMED("LQI gain generator", "LQI gain generator: change the i gain weight of z from " << lqi_z_weight_.y() <<  " to "  << config.z_i);
          lqi_z_weight_.y() = config.z_i;
          break;
        case Levels::RECONFIGURE_LQI_Z_D:
          ROS_INFO_STREAM_NAMED("LQI gain generator", "LQI gain generator: change the d gain weight of z from " << lqi_z_weight_.z() <<  " to "  << config.z_d);
          lqi_z_weight_.z() = config.z_d;
          break;
        default :
          break;
        }
    }
}


/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::HydrusLQIController, aerial_robot_control::ControlBase);
