#include <dragon/lqi_gimbal_control.h>

using namespace aerial_robot_model;
using namespace control_plugin;

DragonLQIGimbalController::DragonLQIGimbalController():
  HydrusLQIController(),
  gimbal_roll_i_term_(0), gimbal_pitch_i_term_(0),
  gimbal_roll_control_stamp_(0), gimbal_pitch_control_stamp_(0)
{
  need_yaw_d_control_ = true;
}

void DragonLQIGimbalController::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                           boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                           boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                           boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                           double ctrl_loop_rate)
{
  /* initialize the flight control */
  HydrusLQIController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);

  dragon_robot_model_ = boost::dynamic_pointer_cast<DragonRobotModel>(robot_model);

  /* initialize the matrix */
  P_xy_ = Eigen::MatrixXd::Zero(2, motor_num_ * 2);
  for(int i = 0; i < motor_num_; i++)
    {
      P_xy_(0, i * 2) = 1;
      P_xy_(1, 1 + i * 2) = 1;
    }

  target_thrust_terms_.resize(motor_num_);
  lqi_att_terms_.resize(motor_num_);

  /* initialize the gimbal target angles */
  target_gimbal_angles_.resize(motor_num_ * 2, 0);
  /* additional vectoring force for grasping */
  extra_vectoring_force_ = Eigen::VectorXd::Zero(3 * motor_num_);

  gimbal_control_pub_ = nh_.advertise<sensor_msgs::JointState>("gimbals_ctrl", 1);
  gimbal_target_force_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("debug/gimbals_target_force", 1);
  roll_pitch_pid_pub_ = nh_.advertise<aerial_robot_msgs::FlatnessPid>("debug/roll_pitch_gimbal_control", 1);

  att_control_feedback_state_sub_ = nh_.subscribe("rpy/feedback_state", 1, &DragonLQIGimbalController::attControlFeedbackStateCallback, this);
  extra_vectoring_force_sub_ = nh_.subscribe("extra_vectoring_force", 1, &DragonLQIGimbalController::extraVectoringForceCallback, this);

  std::string service_name;
  nh_.param("apply_external_wrench", service_name, std::string("apply_external_wrench"));
  add_external_wrench_service_ = nh_.advertiseService(service_name, &DragonLQIGimbalController::addExternalWrenchCallback, this);
  nh_.param("clear_external_wrench", service_name, std::string("clear_external_wrench"));
  clear_external_wrench_service_ = nh_.advertiseService(service_name, &DragonLQIGimbalController::clearExternalWrenchCallback, this);

  //dynamic reconfigure server
  gimbal_roll_pitch_pid_server_ = boost::make_shared<dynamic_reconfigure::Server<aerial_robot_control::XYPidControlConfig> >(ros::NodeHandle(nhp_, "gain_generator/gimbal/pitch_roll"));
  dynamic_reconf_func_gimbal_roll_pitch_pid_ = boost::bind(&DragonLQIGimbalController::cfgGimbalPitchRollPidCallback, this, _1, _2);
  gimbal_roll_pitch_pid_server_->setCallback(dynamic_reconf_func_gimbal_roll_pitch_pid_);

  gimbal_yaw_pid_server_ = boost::make_shared<dynamic_reconfigure::Server<aerial_robot_control::XYPidControlConfig> >(ros::NodeHandle(nhp_, "gain_generator/gimbal/yaw"));
  dynamic_reconf_func_gimbal_yaw_pid_ = boost::bind(&DragonLQIGimbalController::cfgGimbalYawPidCallback, this, _1, _2);
  gimbal_yaw_pid_server_->setCallback(dynamic_reconf_func_gimbal_yaw_pid_);
}


void DragonLQIGimbalController::attControlFeedbackStateCallback(const spinal::RollPitchYawTermConstPtr& msg)
{
  if(motor_num_ == 0) return;

  if(!add_lqi_result_) return;

  /* reproduce the control term about attitude in spinal based on LQI */
  /* -- only consider the P term and I term, since D term (angular velocity from gyro) in current Dragon platform is too noisy -- */

  for(int i = 0; i < motor_num_; i++)
    {
      lqi_att_terms_.at(i) = -roll_gains_.at(i).p * (msg->roll_p / 1000.0) + roll_gains_.at(i).i * (msg->roll_i / 1000.0) + (-pitch_gains_.at(i).p) * (msg->pitch_p / 1000.0) + pitch_gains_.at(i).i * (msg->pitch_i / 1000.0);
    }
}

bool DragonLQIGimbalController::update()
{
  if(!ControlBase::update() || gimbal_vectoring_check_flag_) return false;


  stateError();

  pidUpdate(); //LQI thrust control
  gimbalControl(); //gimbal vectoring control
  sendCmd();
}

void DragonLQIGimbalController::gimbalControl()
{
  if (control_timestamp_ < 0) return;

  /* get roll/pitch angle */
  double roll_angle = estimator_->getState(State::ROLL_COG, estimate_mode_)[0];
  double pitch_angle = estimator_->getState(State::PITCH_COG, estimate_mode_)[0];
  double yaw_angle = estimator_->getState(State::YAW_COG, estimate_mode_)[0];

  int rotor_num = motor_num_;

  std::vector<Eigen::Vector3d> rotors_origin_from_cog = robot_model_->getRotorsOriginFromCog<Eigen::Vector3d>();
  Eigen::Matrix3d links_inertia = robot_model_->getInertia<Eigen::Matrix3d>();

  /* roll pitch condition */
  double max_x = 1e-6;
  double max_y = 1e-6;
  double max_z = 1e-6;

  Eigen::MatrixXd P_att = Eigen::MatrixXd::Zero(3, rotor_num  * 2);
  double acc_z = 0;
  for(int i = 0; i < rotor_num; i++)
    {
      P_att(0, 2 * i + 1) = -rotors_origin_from_cog[i](2); //x(roll)
      P_att(1, 2 * i) = rotors_origin_from_cog[i](2); //y(pitch)
      P_att(2, 2 * i) = -rotors_origin_from_cog[i](1);
      P_att(2, 2 * i + 1) = rotors_origin_from_cog[i](0);

      /* roll pitch condition */
      if(fabs(rotors_origin_from_cog[i](0)) > max_x) max_x = fabs(rotors_origin_from_cog[i](0));
      if(fabs(rotors_origin_from_cog[i](1)) > max_y) max_y = fabs(rotors_origin_from_cog[i](1));
      if(fabs(rotors_origin_from_cog[i](2)) > max_z) max_z = fabs(rotors_origin_from_cog[i](2));

      acc_z += (lqi_att_terms_.at(i) + z_control_terms_.at(i));
    }
  acc_z /= robot_model_->getMass();

  Eigen::MatrixXd P_att_orig = P_att;
  P_att = links_inertia.inverse() * P_att_orig;


  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(5, rotor_num  * 2);
  P.block(0, 0, 3, rotor_num * 2) = P_att;
  P.block(3, 0, 2, rotor_num * 2) = P_xy_ / robot_model_->getMass();

  double P_det = (P * P.transpose()).determinant();

  if(control_verbose_)
    {
      std::cout << "gimbal P original: \n"  << std::endl << P_att_orig << std::endl;
      std::cout << "gimbal P: \n"  << std::endl << P << std::endl;
      std::cout << "P det: "  << std::endl << P_det << std::endl;
      std::cout << "acc_z: " << acc_z  << std::endl;
    }

  Eigen::VectorXd f_xy;

  if(P_det < gimbal_pitch_roll_control_p_det_thresh_)
    {
      // no pitch roll
      if(control_verbose_) ROS_ERROR("bad P_det: %f", P_det);
      P = Eigen::MatrixXd::Zero(3, rotor_num  * 2);
      P.block(0, 0, 1, rotor_num * 2) = P_att.block(2, 0, 1, rotor_num * 2);
      P.block(1, 0, 2, rotor_num * 2) = P_xy_ / robot_model_->getMass();

      f_xy = pseudoinverse(P) * Eigen::Vector3d(yaw_control_terms_[0], target_pitch_ - (pitch_angle * acc_z), (-target_roll_) - (-roll_angle * acc_z));

      // reset gimbal roll pitch control
      gimbal_roll_i_term_ = 0;
      gimbal_pitch_i_term_ = 0;
      gimbal_roll_control_stamp_ = 0;
      gimbal_pitch_control_stamp_ = 0;
    }
  else
    {
      /* roll & pitch gimbal additional control */
      double target_gimbal_roll = 0, target_gimbal_pitch = 0;

      /* ros pub */
      aerial_robot_msgs::FlatnessPid pid_msg;
      pid_msg.header.stamp = ros::Time::now();

      /* roll addition control */
      if(max_z > gimbal_pitch_roll_control_rate_thresh_ * max_y)
        {
          if(gimbal_roll_control_stamp_ == 0)
            gimbal_roll_control_stamp_ = ros::Time::now().toSec();
          double du = ros::Time::now().toSec() - gimbal_roll_control_stamp_;

          if(control_verbose_)
            ROS_WARN("roll gimbal control, max_z: %f, max_y: %f", max_z, max_y);

          double p_term, d_term;
          double state_roll = estimator_->getState(State::ROLL_COG, estimate_mode_)[0];
          double state_roll_vel = estimator_->getState(State::ROLL_COG, estimate_mode_)[1];

          /* P */
          p_term = clamp(gimbal_pitch_roll_gains_[0] * (-state_roll),  -gimbal_pitch_roll_terms_limits_[0], gimbal_pitch_roll_terms_limits_[0]);

          /* I */
          gimbal_roll_i_term_ += (-state_roll * du * gimbal_pitch_roll_gains_[1]);
          gimbal_roll_i_term_ = clamp(gimbal_roll_i_term_, -gimbal_pitch_roll_terms_limits_[1], gimbal_pitch_roll_terms_limits_[1]);

          /* D */
          d_term = clamp(gimbal_pitch_roll_gains_[2] * (-state_roll_vel),  -gimbal_pitch_roll_terms_limits_[2], gimbal_pitch_roll_terms_limits_[2]);

          target_gimbal_roll = clamp(p_term + gimbal_roll_i_term_ + d_term, -gimbal_pitch_roll_limit_, gimbal_pitch_roll_limit_);

          pid_msg.roll.total.push_back(target_gimbal_roll);
          pid_msg.roll.p_term.push_back(p_term);
          pid_msg.roll.i_term.push_back(gimbal_roll_i_term_);
          pid_msg.roll.d_term.push_back(d_term);

          pid_msg.roll.pos_err = state_roll;
          pid_msg.roll.target_pos = 0;
          pid_msg.roll.vel_err = state_roll_vel;
          pid_msg.roll.target_vel = 0;

          gimbal_roll_control_stamp_ = ros::Time::now().toSec();
        }
      else
        {
          //reset
          gimbal_roll_i_term_ = 0;
          gimbal_roll_control_stamp_ = 0;
        }

      /* pitch addition control */
      if(max_z > gimbal_pitch_roll_control_rate_thresh_ * max_x)
        {
          if(gimbal_pitch_control_stamp_ == 0)
            gimbal_pitch_control_stamp_ = ros::Time::now().toSec();
          double du = ros::Time::now().toSec() - gimbal_pitch_control_stamp_;


          if(control_verbose_)
            ROS_WARN("pitch gimbal control, max_z: %f, max_x: %f", max_z, max_y);

          double p_term, d_term;
          double state_pitch = estimator_->getState(State::PITCH_COG, estimate_mode_)[0];
          double state_pitch_vel = estimator_->getState(State::PITCH_COG, estimate_mode_)[1];

          /* P */
          p_term = clamp(gimbal_pitch_roll_gains_[0] * (-state_pitch),  -gimbal_pitch_roll_terms_limits_[0], gimbal_pitch_roll_terms_limits_[0]);

          /* I */
          gimbal_pitch_i_term_ += (-state_pitch * du * gimbal_pitch_roll_gains_[1]);
          gimbal_pitch_i_term_ = clamp(gimbal_pitch_i_term_, -gimbal_pitch_roll_terms_limits_[1], gimbal_pitch_roll_terms_limits_[1]);

          /* D */
          d_term = clamp(gimbal_pitch_roll_gains_[2] * (-state_pitch_vel),  -gimbal_pitch_roll_terms_limits_[2], gimbal_pitch_roll_terms_limits_[2]);

          target_gimbal_pitch = clamp(p_term + gimbal_pitch_i_term_ + d_term, -gimbal_pitch_roll_limit_, gimbal_pitch_roll_limit_);

          pid_msg.pitch.total.push_back(target_gimbal_pitch);
          pid_msg.pitch.p_term.push_back(p_term);
          pid_msg.pitch.i_term.push_back(gimbal_pitch_i_term_);
          pid_msg.pitch.d_term.push_back(d_term);

          pid_msg.pitch.pos_err = state_pitch;
          pid_msg.pitch.target_pos = 0;
          pid_msg.pitch.vel_err = state_pitch_vel;
          pid_msg.pitch.target_vel = 0;

          gimbal_pitch_control_stamp_ = ros::Time::now().toSec();
        }
      else
        {
          //reset
          gimbal_pitch_i_term_ = 0;
          gimbal_pitch_control_stamp_ = 0;
        }


      roll_pitch_pid_pub_.publish(pid_msg);

      Eigen::VectorXd pid_values(5);
      /* F = P# * [roll_pid, pitch_pid, yaw_pid, x_pid, y_pid] */
      pid_values << target_gimbal_roll, target_gimbal_pitch, yaw_control_terms_[0], target_pitch_ - (pitch_angle * acc_z), (-target_roll_) - (-roll_angle * acc_z);
      f_xy = pseudoinverse(P) * pid_values;
    }

  if(control_verbose_)
    {
      std::cout << "gimbal P_pseudo_inverse:"  << std::endl << pseudoinverse(P) << std::endl;
      std::cout << "gimbal force for horizontal control:"  << std::endl << f_xy << std::endl;
    }

  /* external wrench compensation */
  if(boost::dynamic_pointer_cast<aerial_robot_navigation::DragonNavigator>(navigator_)->getLandingFlag())
    {
      dragon_robot_model_->resetExternalStaticWrench(); // clear the external wrench
      extra_vectoring_force_.setZero(); // clear the extra vectoring force
    }

  Eigen::MatrixXd cog_rot_inv = aerial_robot_model::kdlToEigen(KDL::Rotation::RPY(roll_angle, pitch_angle, yaw_angle).Inverse());
  Eigen::MatrixXd extended_cog_rot_inv = Eigen::MatrixXd::Zero(6, 6);
  extended_cog_rot_inv.topLeftCorner(3,3) = cog_rot_inv;
  extended_cog_rot_inv.bottomRightCorner(3,3) = cog_rot_inv;
  std::map<std::string, DragonRobotModel::ExternalWrench> external_wrench_map = dragon_robot_model_->getExternalWrenchMap();
  for(auto& wrench: external_wrench_map) wrench.second.wrench = extended_cog_rot_inv * wrench.second.wrench;

  dragon_robot_model_->calcExternalWrenchCompThrust(external_wrench_map);
  const Eigen::VectorXd& wrench_comp_thrust = dragon_robot_model_->getExWrenchCompensateVectoringThrust();
  if(control_verbose_)
    {
      std::cout << "external wrench  compensate vectoring thrust: " << wrench_comp_thrust.transpose() << std::endl;
    }

  std_msgs::Float32MultiArray target_force_msg;

  for(int i = 0; i < rotor_num; i++)
    {
      /* vectoring force */
      tf::Vector3 f_i(f_xy(2 * i) + wrench_comp_thrust(3 * i) + extra_vectoring_force_(3 * i),
                      f_xy(2 * i + 1) + wrench_comp_thrust(3 * i + 1) + extra_vectoring_force_(3 * i + 1),
                      z_control_terms_.at(i) + lqi_att_terms_.at(i) + wrench_comp_thrust(3 * i + 2) + extra_vectoring_force_(3 * i + 2));


      /* calculate ||f||, but omit pitch and roll term, which will be added in spinal */
      target_thrust_terms_.at(i) = (f_i - tf::Vector3(0, 0, lqi_att_terms_.at(i))).length();
      if(control_verbose_)
        ROS_INFO("[gimbal control]: rotor%d, target_thrust vs target_z: [%f vs %f]", i+1, target_thrust_terms_.at(i), z_control_terms_.at(i));

      if(!start_rp_integration_) // in the early stage of takeoff avoiding the large tilt angle,
        f_i.setValue(f_xy(2 * i), f_xy(2 * i + 1), robot_model_->getStaticThrust()[i]);

      tf::Vector3 f_i_normalized = f_i.normalize();
      if(control_verbose_) ROS_INFO("gimbal%d f normaizled: [%f, %f, %f]", i + 1, f_i_normalized.x(), f_i_normalized.y(), f_i_normalized.z());


      /* f -> gimbal angle */
      std::vector<KDL::Rotation> links_frame_from_cog = dragon_robot_model_->getLinksRotationFromCog<KDL::Rotation>();

      /* [S_pitch, -S_roll * C_pitch, C_roll * C_roll]^T = R.transpose * f_i_normalized */
      tf::Quaternion q;  tf::quaternionKDLToTF(links_frame_from_cog[i], q);
      tf::Vector3 r_f_i_normalized = tf::Matrix3x3(q).inverse() * f_i_normalized;
      if(control_verbose_) ROS_INFO("gimbal%d r f normaizled: [%f, %f, %f]", i + 1, r_f_i_normalized.x(), r_f_i_normalized.y(), r_f_i_normalized.z());

      double gimbal_i_roll = atan2(-r_f_i_normalized[1], r_f_i_normalized[2]);
      double gimbal_i_pitch = atan2(r_f_i_normalized[0], -r_f_i_normalized[1] * sin(gimbal_i_roll) + r_f_i_normalized[2] * cos(gimbal_i_roll));
      /* TODO: check the singularity: gimbal pitch = pi/2 */

      target_gimbal_angles_[2 * i] = gimbal_i_roll;
      target_gimbal_angles_[2 * i + 1] = gimbal_i_pitch;

      if(control_verbose_) std::cout << "gimbal" << i + 1 <<"r & p: " << gimbal_i_roll << ", "<< gimbal_i_pitch  << std::endl;

      /* ros publish */
      target_force_msg.data.push_back(f_xy(2 * i));
      target_force_msg.data.push_back(f_xy(2 * i + 1));
    }
  gimbal_target_force_pub_.publish(target_force_msg);
}

void DragonLQIGimbalController::sendCmd()
{
  /* send base throttle command */
  spinal::FourAxisCommand flight_command_data;
  flight_command_data.angles[0] =  0;
  flight_command_data.angles[1] =  0;

  flight_command_data.base_throttle.resize(motor_num_);

  for(int i = 0; i < motor_num_; i++)
    {
      /* ----------------------------------------
         1. need to send ||f||, instead of LQI(z).
         2. exclude the attitude part (LQI(roll), LQI(pitch), which will be calculated in spinal.
         ----------------------------------------- */
      flight_command_data.base_throttle[i] = target_thrust_terms_.at(i);
    }
  flight_cmd_pub_.publish(flight_command_data);

  /* send gimbal control command */
  sensor_msgs::JointState gimbal_control_msg;
  gimbal_control_msg.header.stamp = ros::Time::now();

  if (gimbal_vectoring_check_flag_)
    {
      gimbal_control_msg.position = dragon_robot_model_->getGimbalNominalAngles();
    }
  else
    {
      for(int i = 0; i < motor_num_ * 2; i++)
        gimbal_control_msg.position.push_back(target_gimbal_angles_.at(i));
    }
  gimbal_control_pub_.publish(gimbal_control_msg);
}

/* external wrench */
bool DragonLQIGimbalController::addExternalWrenchCallback(gazebo_msgs::ApplyBodyWrench::Request& req, gazebo_msgs::ApplyBodyWrench::Response& res)
{
  if(dragon_robot_model_->addExternalStaticWrench(req.body_name, req.reference_frame, req.reference_point, req.wrench))
    res.success  = true;
  else
    res.success  = false;

  return true;
}

bool DragonLQIGimbalController::clearExternalWrenchCallback(gazebo_msgs::BodyRequest::Request& req, gazebo_msgs::BodyRequest::Response& res)
{
  dragon_robot_model_->removeExternalStaticWrench(req.body_name);
  return true;
}

/* extra vectoring force  */
void DragonLQIGimbalController::extraVectoringForceCallback(const std_msgs::Float32MultiArrayConstPtr& msg)
{
  if(navigator_->getNaviState() != aerial_robot_navigation::HOVER_STATE || navigator_->getForceLandingFlag() || landing_flag_) return;

  if(extra_vectoring_force_.size() != msg->data.size())
    {
      ROS_ERROR_STREAM("gimbal control: can not assign the extra vectroing force, the size is wrong: " << msg->data.size() << "; reset");
      extra_vectoring_force_.setZero();
      return;
    }

  extra_vectoring_force_ = (Eigen::Map<const Eigen::VectorXf>(msg->data.data(), msg->data.size())).cast<double>();

  ROS_INFO_STREAM("add extra vectoring force is: \n" << extra_vectoring_force_.transpose());
}

void DragonLQIGimbalController::cfgGimbalPitchRollPidCallback(aerial_robot_control::XYPidControlConfig &config, uint32_t level)
{
  if(config.xy_pid_control_flag)
    {
      printf("Pitch Roll Pid Param:");
      switch(level)
        {
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_P_GAIN:
          gimbal_pitch_roll_gains_[0] = config.p_gain;
          printf("change the p gain\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_I_GAIN:
          gimbal_pitch_roll_gains_[1] = config.i_gain;
          printf("change the i gain\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_D_GAIN:
          gimbal_pitch_roll_gains_[2] = config.d_gain;
          printf("change the d gain\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_LIMIT:
          gimbal_pitch_roll_limit_ = config.limit;
          printf("change the limit\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_P_LIMIT:
          gimbal_pitch_roll_terms_limits_[0] = config.p_limit;
          printf("change the p limit\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_I_LIMIT:
          gimbal_pitch_roll_terms_limits_[1] = config.i_limit;
          printf("change the i limit\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_D_LIMIT:
          gimbal_pitch_roll_terms_limits_[2] = config.d_limit;
          printf("change the d limit\n");
          break;
        default :
          printf("\n");
          break;
        }
    }
}

void DragonLQIGimbalController::cfgGimbalYawPidCallback(aerial_robot_control::XYPidControlConfig &config, uint32_t level)
{
  if(config.xy_pid_control_flag)
    {
      printf("Yaw Pid Param:");
      switch(level)
        {
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_P_GAIN:
          yaw_gains_[0][0] = -config.p_gain;
          printf("change the p gain\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_I_GAIN:
          yaw_gains_[0][1] = config.i_gain;
          printf("change the i gain\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_D_GAIN:
          yaw_gains_[0][2] = config.d_gain;
          printf("change the d gain\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_LIMIT:
          yaw_limit_ = config.limit;
          printf("change the limit\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_P_LIMIT:
          yaw_terms_limits_[0] = config.p_limit;
          printf("change the p limit\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_I_LIMIT:
          yaw_terms_limits_[1] = config.i_limit;
          printf("change the i limit\n");
          break;
        case aerial_robot_msgs::DynamicReconfigureLevels::RECONFIGURE_D_LIMIT:
          yaw_terms_limits_[2] = config.d_limit;
          printf("change the d limit\n");
          break;
        default :
          printf("\n");
          break;
        }
    }
}

void DragonLQIGimbalController::rosParamInit()
{
  HydrusLQIController::rosParamInit();

  ros::NodeHandle control_nh(nh_, "controller");
  getParam<bool>(control_nh, "control_verbose", control_verbose_, false);
  getParam<bool>(control_nh, "add_lqi_result", add_lqi_result_, false);
  getParam<bool>(control_nh, "gimbal_vectoring_check_flag", gimbal_vectoring_check_flag_, false); // check the gimbal vectoring function without position and yaw control

  ros::NodeHandle gimbal_pitch_roll_nh(control_nh, "gimbal_pitch_roll");
  getParam<double>(gimbal_pitch_roll_nh, "control_rate_thresh", gimbal_pitch_roll_control_rate_thresh_, 1.0);
  getParam<double>(gimbal_pitch_roll_nh, "control_p_det_thresh", gimbal_pitch_roll_control_p_det_thresh_, 1e-3);
  getParam<double>(gimbal_pitch_roll_nh, "limit", gimbal_pitch_roll_limit_, 1.0e6);
  getParam<double>(gimbal_pitch_roll_nh, "p_term_limit", gimbal_pitch_roll_terms_limits_[0], 1.0e6);
  getParam<double>(gimbal_pitch_roll_nh, "i_term_limit", gimbal_pitch_roll_terms_limits_[1], 1.0e6);
  getParam<double>(gimbal_pitch_roll_nh, "d_term_limit", gimbal_pitch_roll_terms_limits_[2], 1.0e6);
  getParam<double>(gimbal_pitch_roll_nh, "p_gain", gimbal_pitch_roll_gains_[0], 0.0);
  getParam<double>(gimbal_pitch_roll_nh, "i_gain", gimbal_pitch_roll_gains_[1], 0.0);
  getParam<double>(gimbal_pitch_roll_nh, "d_gain", gimbal_pitch_roll_gains_[2], 0.0);

}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(control_plugin::DragonLQIGimbalController, control_plugin::ControlBase);
