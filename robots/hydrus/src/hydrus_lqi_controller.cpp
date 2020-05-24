#include <hydrus/hydrus_lqi_controller.h>

// TODO: LQI Q matrix -> M, R matrix -> N

using namespace control_plugin;


void HydrusLQIController::initialize(ros::NodeHandle nh,
                                     ros::NodeHandle nhp,
                                     boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                     boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                     Navigator* navigator,
                                     double ctrl_loop_rate)
{
  FlatnessPid::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);

  hydrus_robot_model_ = boost::dynamic_pointer_cast<HydrusRobotModel>(robot_model);

  //publisher
  rpy_gain_pub_ = nh_.advertise<spinal::RollPitchYawTerms>("rpy/gain", 1);
  four_axis_gain_pub_ = nh_.advertise<aerial_robot_msgs::FourAxisGain>("four_axes/gain", 1);
  p_matrix_pseudo_inverse_inertia_pub_ = nh_.advertise<spinal::PMatrixPseudoInverseWithInertia>("p_matrix_pseudo_inverse_inertia", 1);

  //dynamic reconfigure server
  lqi_server_ = boost::make_shared<dynamic_reconfigure::Server<hydrus::LQIConfig> >(ros::NodeHandle(nh, "gain_generator/lqi"));
  dynamic_reconf_func_lqi_ = boost::bind(&HydrusLQIController::cfgLQICallback, this, _1, _2);
  lqi_server_->setCallback(dynamic_reconf_func_lqi_);

  gain_generator_thread_ = std::thread(boost::bind(&HydrusLQIController::gainGeneratorFunc, this));

  //gains
  motor_num_ = robot_model->getRotorNum();
  pitch_gains_.assign(motor_num_, PID());
  roll_gains_.assign(motor_num_, PID());

  z_gains_.resize(motor_num_, tf::Vector3(0,0,0));
  z_control_terms_.resize(motor_num_);
  if(hydrus_robot_model_->getWrenchDof() == 4)
    {
      yaw_gains_.resize(motor_num_, tf::Vector3(0,0,0));
      yaw_control_terms_.resize(motor_num_);
    }
}

HydrusLQIController::~HydrusLQIController()
{
  gain_generator_thread_.join();
}

void HydrusLQIController::cfgLQICallback(hydrus::LQIConfig &config, uint32_t level)
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

  if(robot_model_->getMass() == 0)
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
    q_diagonals << q_z_, q_z_d_, q_roll_, q_roll_d_, q_pitch_, q_pitch_d_, q_z_i_, q_roll_i_, q_pitch_i_;
  else
    q_diagonals << q_z_, q_z_d_, q_roll_, q_roll_d_, q_pitch_, q_pitch_d_, q_yaw_, q_yaw_d_, q_z_i_, q_roll_i_, q_pitch_i_, q_yaw_i_;

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
      z_gains_.at(i).setValue(K_(i,0), K_(i, lqi_mode_ * 2), K_(i,1)); // direct allocate

      roll_gains_.at(i).p = K_(i,2);
      roll_gains_.at(i).i = K_(i, lqi_mode_ * 2 + 1);
      roll_gains_.at(i).d = K_(i,3);

      pitch_gains_.at(i).p = K_(i,4);
      pitch_gains_.at(i).i = K_(i, lqi_mode_ * 2 + 2);
      pitch_gains_.at(i).d = K_(i,5);

      if(lqi_mode_ == 4) yaw_gains_.at(i).setValue(K_(i,6), K_(i, lqi_mode_ * 2 + 3), K_(i,7));
      else
        {
          if(hydrus_robot_model_->getWrenchDof() == 4)
            {
              yaw_gains_.at(i).setValue(0, 0, 0); // TODO: hard-coding
            }
        }
    }

  // compensation for gyro moment
  p_mat_pseudo_inv_ = aerial_robot_model::pseudoinverse(P.middleRows(2, lqi_mode_));
  return true;
}

void HydrusLQIController::clampGain()
{
  /* avoid the violation of 16int_t range because of spinal::RollPitchYawTerms */
  double max_gain_thresh = 32.767;
  double max_roll_p_gain = 0, max_roll_d_gain = 0, max_pitch_p_gain = 0, max_pitch_d_gain = 0;
  for(int i = 0; i < motor_num_; ++i)
    {
      if(max_roll_p_gain < fabs(roll_gains_.at(i).p)) max_roll_p_gain = fabs(roll_gains_.at(i).p);
      if(max_roll_d_gain < fabs(roll_gains_.at(i).d)) max_roll_d_gain = fabs(roll_gains_.at(i).d);
      if(max_pitch_p_gain < fabs(pitch_gains_.at(i).p)) max_pitch_p_gain = fabs(pitch_gains_.at(i).p);
      if(max_pitch_d_gain < fabs(pitch_gains_.at(i).d)) max_pitch_d_gain = fabs(pitch_gains_.at(i).d);
    }

  double roll_p_gain_scale = 1, roll_d_gain_scale = 1, pitch_p_gain_scale = 1, pitch_d_gain_scale = 1;
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

  for(int i = 0; i < motor_num_; ++i)
    {
      roll_gains_.at(i).p *= roll_p_gain_scale;
      roll_gains_.at(i).d *= roll_d_gain_scale;

      pitch_gains_.at(i).p *= pitch_p_gain_scale;
      pitch_gains_.at(i).d *= pitch_d_gain_scale;
    }

  // TODO: hard-coding
  if(lqi_mode_ == 4)
    {
      double max_yaw_d_gain = 0;
      for(int i = 0; i < motor_num_; ++i)
        if(max_yaw_d_gain < fabs(yaw_gains_.at(i).z())) max_yaw_d_gain = fabs(yaw_gains_.at(i).z());

      double yaw_d_gain_scale = 1;
      if(max_yaw_d_gain > max_gain_thresh)
        {
          ROS_WARN_STREAM_NAMED("LQI gain generator", "LQI gain generator: the max yaw d gain violate the range of int16_t: " << max_yaw_d_gain);
          yaw_d_gain_scale = max_gain_thresh / max_yaw_d_gain;
        }
      for(int i = 0; i < motor_num_; ++i) yaw_gains_.at(i)[2] *= yaw_d_gain_scale;
    }
}


void HydrusLQIController::rosParamInit()
{
  FlatnessPid::rosParamInit();

  ros::NodeHandle control_nh(nh_, "controller");
  ros::NodeHandle lqi_nh(control_nh, "lqi");
  lqi_nh.param("gyro_moment_compensation", gyro_moment_compensation_, false);
  lqi_nh.param("clamp_gain", clamp_gain_, true);

  /* propeller direction and lqi R */
  r_.resize(robot_model_->getRotorNum()); // motor_num is not set
  for(int i = 0; i < robot_model_->getRotorNum(); ++i) {
    std::stringstream ss;
    ss << i + 1;
    /* R */
    getParam<double>(lqi_nh, std::string("r") + ss.str(), r_.at(i), 1.0);
  }

  getParam<double>(lqi_nh, "q_roll", q_roll_, 1.0);
  getParam<double>(lqi_nh, "q_roll_d", q_roll_d_, 1.0);
  getParam<double>(lqi_nh, "q_pitch", q_pitch_, 1.0);
  getParam<double>(lqi_nh, "q_pitch_d", q_pitch_d_,  1.0);
  getParam<double>(lqi_nh, "q_yaw", q_yaw_, 1.0);
  getParam<double>(lqi_nh, "q_yaw_d", q_yaw_d_, 1.0);
  getParam<double>(lqi_nh, "q_z", q_z_, 1.0);
  getParam<double>(lqi_nh, "q_z_d", q_z_d_, 1.0);

  getParam<double>(lqi_nh, "q_roll_i", q_roll_i_, 1.0);
  getParam<double>(lqi_nh, "q_pitch_i", q_pitch_i_, 1.0);
  getParam<double>(lqi_nh, "q_yaw_i", q_yaw_i_, 1.0);
  getParam<double>(lqi_nh, "q_z_i", q_z_i_, 1.0);
}

void HydrusLQIController::publishGain()
{
  aerial_robot_msgs::FourAxisGain four_axis_gain_msg;
  spinal::RollPitchYawTerms rpy_gain_msg; //for rosserial
  spinal::PMatrixPseudoInverseWithInertia p_pseudo_inverse_with_inertia_msg;

  rpy_gain_msg.motors.resize(motor_num_);
  p_pseudo_inverse_with_inertia_msg.pseudo_inverse.resize(motor_num_);

  for(int i = 0; i < motor_num_; ++i)
    {
      four_axis_gain_msg.roll_p_gain.push_back(roll_gains_.at(i).p);
      four_axis_gain_msg.roll_i_gain.push_back(roll_gains_.at(i).i);
      four_axis_gain_msg.roll_d_gain.push_back(roll_gains_.at(i).d);

      four_axis_gain_msg.pitch_p_gain.push_back(pitch_gains_.at(i).p);
      four_axis_gain_msg.pitch_i_gain.push_back(pitch_gains_.at(i).i);
      four_axis_gain_msg.pitch_d_gain.push_back(pitch_gains_.at(i).d);

      // TODO: temporary, consider the DragonRobotModel derived
      if(lqi_mode_ == 4)
        {
          four_axis_gain_msg.yaw_p_gain.push_back(yaw_gains_.at(i)[0]);
          four_axis_gain_msg.yaw_i_gain.push_back(yaw_gains_.at(i)[1]);
          four_axis_gain_msg.yaw_d_gain.push_back(yaw_gains_.at(i)[2]);
        }
      else
        {
          four_axis_gain_msg.yaw_p_gain.push_back(0);
          four_axis_gain_msg.yaw_i_gain.push_back(0);
          four_axis_gain_msg.yaw_d_gain.push_back(0);
        }

      four_axis_gain_msg.z_p_gain.push_back(z_gains_.at(i)[0]);
      four_axis_gain_msg.z_i_gain.push_back(z_gains_.at(i)[1]);
      four_axis_gain_msg.z_d_gain.push_back(z_gains_.at(i)[2]);

      /* to flight controller via rosserial scaling by 1000 */
      rpy_gain_msg.motors[i].roll_p = roll_gains_.at(i).p * 1000;
      rpy_gain_msg.motors[i].roll_i = roll_gains_.at(i).i * 1000;
      rpy_gain_msg.motors[i].roll_d = roll_gains_.at(i).d * 1000;

      rpy_gain_msg.motors[i].pitch_p = pitch_gains_.at(i).p * 1000;
      rpy_gain_msg.motors[i].pitch_i = pitch_gains_.at(i).i * 1000;
      rpy_gain_msg.motors[i].pitch_d = pitch_gains_.at(i).d * 1000;

      // TODO: temporary, consider the DragonRobotModel derived
      if(lqi_mode_ == 4)
        rpy_gain_msg.motors[i].yaw_d = yaw_gains_.at(i)[2] * 1000;
      else
        rpy_gain_msg.motors[i].yaw_d = 0;

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

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(control_plugin::HydrusLQIController, control_plugin::ControlBase);
