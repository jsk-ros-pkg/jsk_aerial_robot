#include <dragon/control/lqi_gimbal_control.h>

using namespace aerial_robot_model;
using namespace aerial_robot_control;

DragonLQIGimbalController::DragonLQIGimbalController():
  HydrusLQIController()
{
}

void DragonLQIGimbalController::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                           boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                           boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                           boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                           double ctrl_loop_rate)
{
  /* initialize the flight control */
  HydrusLQIController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);

  dragon_robot_model_ = boost::dynamic_pointer_cast<Dragon::HydrusLikeRobotModel>(robot_model);

  /* initialize the matrix */
  P_xy_ = Eigen::MatrixXd::Zero(2, motor_num_ * 2);
  for(int i = 0; i < motor_num_; i++)
    {
      P_xy_(0, i * 2) = 1;
      P_xy_(1, 1 + i * 2) = 1;
    }

  lqi_att_terms_.resize(motor_num_);

  /* initialize the gimbal target angles */
  target_gimbal_angles_.resize(motor_num_ * 2, 0);
  /* additional vectoring force for grasping */
  extra_vectoring_force_ = Eigen::VectorXd::Zero(3 * motor_num_);

  gimbal_control_pub_ = nh_.advertise<sensor_msgs::JointState>("gimbals_ctrl", 1);
  gimbal_target_force_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("debug/gimbals_target_force", 1);

  att_control_feedback_state_sub_ = nh_.subscribe("rpy/feedback_state", 1, &DragonLQIGimbalController::attControlFeedbackStateCallback, this);
  extra_vectoring_force_sub_ = nh_.subscribe("extra_vectoring_force", 1, &DragonLQIGimbalController::extraVectoringForceCallback, this);

  std::string service_name;
  nh_.param("apply_external_wrench", service_name, std::string("apply_external_wrench"));
  add_external_wrench_service_ = nh_.advertiseService(service_name, &DragonLQIGimbalController::addExternalWrenchCallback, this);
  nh_.param("clear_external_wrench", service_name, std::string("clear_external_wrench"));
  clear_external_wrench_service_ = nh_.advertiseService(service_name, &DragonLQIGimbalController::clearExternalWrenchCallback, this);

  //message
  pid_msg_.yaw.total.resize(1);
  pid_msg_.yaw.p_term.resize(1);
  pid_msg_.yaw.i_term.resize(1);
  pid_msg_.yaw.d_term.resize(1);
}

void DragonLQIGimbalController::attControlFeedbackStateCallback(const spinal::RollPitchYawTermConstPtr& msg)
{
  if(motor_num_ == 0) return;

  if(!add_lqi_result_) return;

  /* reproduce the control term about attitude in spinal based on LQI, instead of the roll/pitch control from pose linear controller  */
  /* -- only consider the P term and I term, since D term (angular velocity from gyro) in current Dragon platform is too noisy -- */

  for(int i = 0; i < motor_num_; i++)
    {
      lqi_att_terms_.at(i) = -roll_gains_.at(i)[0] * (msg->roll_p / 1000.0) + roll_gains_.at(i)[1] * (msg->roll_i / 1000.0) + (-pitch_gains_.at(i)[0]) * (msg->pitch_p / 1000.0) + pitch_gains_.at(i)[1] * (msg->pitch_i / 1000.0);
    }
}

bool DragonLQIGimbalController::update()
{
  if(gimbal_vectoring_check_flag_) return false;

  HydrusLQIController::update();

  return true;
}


void DragonLQIGimbalController::controlCore()
{
  HydrusLQIController::controlCore();
  target_pitch_ = 0; // reset
  target_roll_ = 0;  // reset

  gimbalControl();
}

void DragonLQIGimbalController::gimbalControl()
{
  std::vector<Eigen::Vector3d> rotors_origin_from_cog = robot_model_->getRotorsOriginFromCog<Eigen::Vector3d>();
  Eigen::Matrix3d links_inertia = robot_model_->getInertia<Eigen::Matrix3d>();

  const std::vector<float>& z_control_terms = target_base_thrust_;

  double max_x = 1e-6;
  double max_y = 1e-6;
  double max_z = 1e-6;

  Eigen::MatrixXd P_att = Eigen::MatrixXd::Zero(3, motor_num_  * 2);
  double acc_z = 0;
  for(int i = 0; i < motor_num_; i++)
    {
      P_att(0, 2 * i + 1) = -rotors_origin_from_cog.at(i)(2); //x(roll)
      P_att(1, 2 * i) = rotors_origin_from_cog.at(i)(2); //y(pitch)
      P_att(2, 2 * i) = -rotors_origin_from_cog.at(i)(1);
      P_att(2, 2 * i + 1) = rotors_origin_from_cog.at(i)(0);

      /* roll pitch condition */
      if(fabs(rotors_origin_from_cog.at(i)(0)) > max_x) max_x = fabs(rotors_origin_from_cog.at(i)(0));
      if(fabs(rotors_origin_from_cog.at(i)(1)) > max_y) max_y = fabs(rotors_origin_from_cog.at(i)(1));
      if(fabs(rotors_origin_from_cog.at(i)(2)) > max_z) max_z = fabs(rotors_origin_from_cog.at(i)(2));

      acc_z += (lqi_att_terms_.at(i) + z_control_terms.at(i));
    }
  acc_z /= robot_model_->getMass();

  Eigen::MatrixXd P_att_orig = P_att;
  P_att = links_inertia.inverse() * P_att_orig;

  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(5, motor_num_  * 2);
  P.block(0, 0, 3, motor_num_ * 2) = P_att;
  P.block(3, 0, 2, motor_num_ * 2) = P_xy_ / robot_model_->getMass();

  double P_det = (P * P.transpose()).determinant();

  if(control_verbose_)
    {
      std::cout << "gimbal P original: \n"  << std::endl << P_att_orig << std::endl;
      std::cout << "gimbal P: \n"  << std::endl << P << std::endl;
      std::cout << "P det: "  << std::endl << P_det << std::endl;
      std::cout << "acc_z: " << acc_z  << std::endl;
    }

  Eigen::VectorXd f_xy;
  tf::Vector3 target_linear_acc_w(pid_controllers_.at(X).result(),
                                  pid_controllers_.at(Y).result(),
                                  pid_controllers_.at(Z).result());
  tf::Vector3 target_linear_acc_cog = (tf::Matrix3x3(tf::createQuaternionFromYaw(rpy_.z()))).inverse() * target_linear_acc_w;

  double target_ang_acc_x = pid_controllers_.at(ROLL).result();
  double target_ang_acc_y = pid_controllers_.at(PITCH).result();
  double target_ang_acc_z = pid_controllers_.at(YAW).result();

  pid_msg_.roll.p_term.at(0) = 0;
  pid_msg_.roll.i_term.at(0) = 0;
  pid_msg_.roll.d_term.at(0) = 0;
  pid_msg_.roll.target_p = target_rpy_.x();
  pid_msg_.roll.err_p = target_rpy_.x() - rpy_.x();
  pid_msg_.roll.target_d = target_omega_.x();
  pid_msg_.roll.err_d = target_omega_.x() - omega_.x();
  pid_msg_.pitch.p_term.at(0) = 0;
  pid_msg_.pitch.i_term.at(0) = 0;
  pid_msg_.pitch.d_term.at(0) = 0;
  pid_msg_.pitch.target_p = target_rpy_.y();
  pid_msg_.pitch.err_p = target_rpy_.y() - rpy_.y();
  pid_msg_.pitch.target_d = target_omega_.y();
  pid_msg_.pitch.err_d = target_omega_.y() - omega_.y();

  if(P_det < gimbal_roll_pitch_control_p_det_thresh_)
    {
      // no pitch roll
      if(control_verbose_) ROS_ERROR("low P_det: %f", P_det);
      P = Eigen::MatrixXd::Zero(3, motor_num_  * 2);
      P.block(0, 0, 1, motor_num_ * 2) = P_att.block(2, 0, 1, motor_num_ * 2);
      P.block(1, 0, 2, motor_num_ * 2) = P_xy_ / robot_model_->getMass();

      f_xy = pseudoinverse(P) * Eigen::Vector3d(target_ang_acc_z, target_linear_acc_cog.x() - (rpy_.y() * acc_z), target_linear_acc_cog.y() - (-rpy_.x() * acc_z));

      // reset  roll pitch control
      pid_controllers_.at(ROLL).reset();
      pid_controllers_.at(PITCH).reset();
      target_ang_acc_x = 0;
      target_ang_acc_y = 0;
    }
  else
    {
      if(max_z > gimbal_roll_pitch_control_rate_thresh_ * max_y)
        {
          if(control_verbose_) ROS_INFO("perform gimbal roll control, max_z: %f, max_y: %f", max_z, max_y);
          pid_msg_.roll.p_term.at(0) = pid_controllers_.at(ROLL).getPTerm();
          pid_msg_.roll.i_term.at(0) = pid_controllers_.at(ROLL).getITerm();
          pid_msg_.roll.d_term.at(0) = pid_controllers_.at(ROLL).getDTerm();
        }
      else
        {
          pid_controllers_.at(ROLL).reset(); // reset
          target_ang_acc_x = 0;
        }

      if(max_z > gimbal_roll_pitch_control_rate_thresh_ * max_x)
        {
          if(control_verbose_) ROS_INFO("perform gimbal pitch control, max_z: %f, max_x: %f", max_z, max_y);
          pid_msg_.pitch.p_term.at(0) = pid_controllers_.at(PITCH).getPTerm();
          pid_msg_.pitch.i_term.at(0) = pid_controllers_.at(PITCH).getITerm();
          pid_msg_.pitch.d_term.at(0) = pid_controllers_.at(PITCH).getDTerm();
        }
      else
        {
          pid_controllers_.at(PITCH).reset(); // reset
          target_ang_acc_y = 0;
        }

      Eigen::VectorXd pid_values(5);
      /* F = P# * [roll_pid, pitch_pid, yaw_pid, x_pid, y_pid] */
      pid_values << target_ang_acc_x, target_ang_acc_y, target_ang_acc_z, target_linear_acc_cog.x() - (rpy_.y() * acc_z), target_linear_acc_cog.y() - (-rpy_.x() * acc_z);
      f_xy = pseudoinverse(P) * pid_values;
    }

  pid_msg_.roll.total.at(0) = target_ang_acc_x;
  pid_msg_.pitch.total.at(0) = target_ang_acc_y;

  std_msgs::Float32MultiArray target_force_msg;
  for(int i = 0; i < motor_num_; i++)
    {
      target_force_msg.data.push_back(f_xy(2 * i));
      target_force_msg.data.push_back(f_xy(2 * i + 1));
    }
  gimbal_target_force_pub_.publish(target_force_msg);

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

  Eigen::MatrixXd cog_rot_inv = aerial_robot_model::kdlToEigen(KDL::Rotation::RPY(rpy_.x(), rpy_.y(), rpy_.z()).Inverse());
  Eigen::MatrixXd extended_cog_rot_inv = Eigen::MatrixXd::Zero(6, 6);
  extended_cog_rot_inv.topLeftCorner(3,3) = cog_rot_inv;
  extended_cog_rot_inv.bottomRightCorner(3,3) = cog_rot_inv;
  std::map<std::string, Dragon::ExternalWrench> external_wrench_map = dragon_robot_model_->getExternalWrenchMap();
  for(auto& wrench: external_wrench_map) wrench.second.wrench = extended_cog_rot_inv * wrench.second.wrench;

  dragon_robot_model_->calcExternalWrenchCompThrust(external_wrench_map);
  const Eigen::VectorXd& wrench_comp_thrust = dragon_robot_model_->getExWrenchCompensateVectoringThrust();
  if(control_verbose_)
    {
      std::cout << "external wrench  compensate vectoring thrust: " << wrench_comp_thrust.transpose() << std::endl;
    }


  // integrate
  for(int i = 0; i < motor_num_; i++)
    {
      /* vectoring force */
      tf::Vector3 f_i(f_xy(2 * i) + wrench_comp_thrust(3 * i) + extra_vectoring_force_(3 * i),
                      f_xy(2 * i + 1) + wrench_comp_thrust(3 * i + 1) + extra_vectoring_force_(3 * i + 1),
                      z_control_terms.at(i) + lqi_att_terms_.at(i) + wrench_comp_thrust(3 * i + 2) + extra_vectoring_force_(3 * i + 2));


      /* calculate ||f||, but omit pitch and roll term, which will be added in spinal */
      target_base_thrust_.at(i) = (f_i - tf::Vector3(0, 0, lqi_att_terms_.at(i))).length();
      if(control_verbose_)
        ROS_INFO("[gimbal control]: rotor%d, target_thrust vs target_z: [%f vs %f]", i+1, target_base_thrust_.at(i), z_control_terms.at(i));

      if(!start_rp_integration_ || navigator_->getForceLandingFlag())
        f_i.setValue(f_xy(2 * i), f_xy(2 * i + 1), robot_model_->getStaticThrust()[i]);

      /* f -> gimbal angle */
      std::vector<KDL::Rotation> links_frame_from_cog = dragon_robot_model_->getLinksRotationFromCog<KDL::Rotation>();

      /* [S_pitch, -S_roll * C_pitch, C_roll * C_roll]^T = R.transpose * f_i / |f_i| */
      tf::Quaternion q;  tf::quaternionKDLToTF(links_frame_from_cog.at(i), q);
      tf::Vector3 r_f_i = tf::Matrix3x3(q).transpose() * f_i;
      if(control_verbose_) ROS_INFO("gimbal%d r f: [%f, %f, %f]", i + 1, r_f_i.x(), r_f_i.y(), r_f_i.z());

      double gimbal_i_roll = atan2(-r_f_i[1], r_f_i[2]);
      double gimbal_i_pitch = atan2(r_f_i[0], -r_f_i[1] * sin(gimbal_i_roll) + r_f_i[2] * cos(gimbal_i_roll));

      target_gimbal_angles_.at(2 * i) = gimbal_i_roll;
      target_gimbal_angles_.at(2 * i + 1) = gimbal_i_pitch;

      if(control_verbose_) std::cout << "gimbal" << i + 1 <<"r & p: " << gimbal_i_roll << ", "<< gimbal_i_pitch  << std::endl;
    }
}

void DragonLQIGimbalController::sendCmd()
{
  HydrusLQIController::sendCmd();

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
  if(navigator_->getNaviState() != aerial_robot_navigation::HOVER_STATE || navigator_->getForceLandingFlag() || boost::dynamic_pointer_cast<aerial_robot_navigation::DragonNavigator>(navigator_)->getLandingFlag()) return;

  if(extra_vectoring_force_.size() != msg->data.size())
    {
      ROS_ERROR_STREAM("gimbal control: can not assign the extra vectroing force, the size is wrong: " << msg->data.size() << "; reset");
      extra_vectoring_force_.setZero();
      return;
    }

  extra_vectoring_force_ = (Eigen::Map<const Eigen::VectorXf>(msg->data.data(), msg->data.size())).cast<double>();

  ROS_INFO_STREAM("add extra vectoring force is: \n" << extra_vectoring_force_.transpose());
}

void DragonLQIGimbalController::rosParamInit()
{
  HydrusLQIController::rosParamInit();

  ros::NodeHandle control_nh(nh_, "controller");
  getParam<bool>(control_nh, "add_lqi_result", add_lqi_result_, false);
  getParam<bool>(control_nh, "gimbal_vectoring_check_flag", gimbal_vectoring_check_flag_, false); // check the gimbal vectoring function without position and yaw control

  ros::NodeHandle roll_pitch_nh(control_nh, "roll_pitch");
  getParam<double>(roll_pitch_nh, "gimbal_control_rate_thresh", gimbal_roll_pitch_control_rate_thresh_, 1.0);
  getParam<double>(roll_pitch_nh, "gimbal_control_p_det_thresh", gimbal_roll_pitch_control_p_det_thresh_, 1e-3);
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::DragonLQIGimbalController, aerial_robot_control::ControlBase);
