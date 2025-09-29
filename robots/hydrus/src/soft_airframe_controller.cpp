#include <hydrus/soft_airframe_controller.h>
using namespace aerial_robot_control;

SoftAirframeController::SoftAirframeController() : PoseLinearController()
{
}

void SoftAirframeController::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                        boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                        boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                        boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                        double ctrl_loop_rate)
{
  PoseLinearController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);
  
  rosParamInit();
  target_base_thrust_.resize(motor_num_);

  //publisher
  rpy_gain_pub_ = nh_.advertise<spinal::RollPitchYawTerms>("rpy/gain", 1);
  flight_cmd_pub_ = nh_.advertise<spinal::FourAxisCommand>("four_axes/command", 1);
  gimbal_control_pub_ = nh_.advertise<sensor_msgs::JointState>("gimbals_ctrl", 1);
  torque_allocation_matrix_inv_pub_ = nh_.advertise<spinal::TorqueAllocationMatrixInv>("torque_allocation_matrix_inv", 1);
  
  // subscriber
  joint_state_sub_ = nh_.subscribe("joint_states", 1, &SoftAirframeController::jointStateCallback, this);
  
  // note: it might be better to use gimbal_link1
  rotor5_pose_sub_ = nh_.subscribe("thrust5/mocap/pose", 1, &SoftAirframeController::Rotor5MocapCallback, this);
  body_pose_sub_ = nh_.subscribe("mocap/pose", 1, &SoftAirframeController::BodyMocapCallback, this);

  torque_allocation_matrix_inv_pub_stamp_ = 0.0;
}

void SoftAirframeController::controlCore()
{
  PoseLinearController::controlCore();

  tf::Vector3 target_acc_w(pid_controllers_.at(X).result(),
                           pid_controllers_.at(Y).result(),
                           pid_controllers_.at(Z).result());
  tf::Vector3 target_acc_dash = (tf::Matrix3x3(tf::createQuaternionFromYaw(rpy_.z()))).inverse() * target_acc_w;

  if(navigator_->getForceLandingFlag())
  {
    target_pitch_ = 0;
    target_roll_ = 0;
  }

  // allocation of thrust
  Eigen::MatrixXd full_q_mat_ = getFullQMat(); // 4 x virtual_motor_num_
  Eigen::MatrixXd full_q_mat_inv_ = aerial_robot_model::pseudoinverse(full_q_mat_);

  std::cout << "full_q_mat_ :" << full_q_mat_ << std::endl;
  std::cout << "full_q_mat_inv_ :" << full_q_mat_inv_ << std::endl;

  std::cout << std::endl;
  Eigen::VectorXd target_vectoring_f_ = Eigen::VectorXd::Zero(virtual_motor_num_); // virtual motor number
  if(hovering_approximate_)
    {
      target_pitch_ = target_acc_dash.x() / aerial_robot_estimation::G;
      target_roll_ = -target_acc_dash.y() / aerial_robot_estimation::G;
      target_vectoring_f_ = full_q_mat_inv_.col(0) * target_acc_w.z(); // todo: add some kind of constraints
    }
  else
    {
      target_pitch_ = atan2(target_acc_dash.x(), target_acc_dash.z());
      target_roll_ = atan2(-target_acc_dash.y(), sqrt(target_acc_dash.x() * target_acc_dash.x() + target_acc_dash.z() * target_acc_dash.z()));
      target_vectoring_f_ = full_q_mat_inv_.col(0) * target_acc_w.length();
    }
  ROS_DEBUG_STREAM("target vectoring f: \n" << target_vectoring_f_.transpose());

  for(int i = 0; i < motor_num_; i++)
  {
    // when using gimbal
    if (i == 4){
      target_base_thrust_.at(i) = Eigen::Vector2d(target_vectoring_f_(4), target_vectoring_f_(5)).norm();
      gimbal_angle_diff_ = atan2(target_vectoring_f_(5), target_vectoring_f_(4));
    }
    else {
      target_base_thrust_.at(i) = target_vectoring_f_(i);
    }
  }

  q_mat_ = getQMat();
  q_mat_inv_ = aerial_robot_model::pseudoinverse(q_mat_);

  std::cout << "q_mat_ :" << q_mat_ << std::endl;
  std::cout << "q_mat_inv_ :" << q_mat_inv_ << std::endl;
  std::cout << std::endl;

  // special process for yaw since the bandwidth between PC and spinal
  double max_yaw_scale = 0; // for reconstruct yaw control term in spinal
  for (unsigned int i = 0; i < motor_num_; i++)
    {
      if(q_mat_inv_(i, YAW - 2) > max_yaw_scale) max_yaw_scale = q_mat_inv_(i, YAW - 2);
    }
  candidate_yaw_term_ = pid_controllers_.at(YAW).result() * max_yaw_scale;


  navigator_->setTargetPitch(target_pitch_);
  navigator_->setTargetRoll(target_roll_);
  pid_msg_.roll.total.at(0) = pid_controllers_.at(ROLL).result();
  pid_msg_.roll.p_term.at(0) = pid_controllers_.at(ROLL).getPTerm();
  pid_msg_.roll.i_term.at(0) = pid_controllers_.at(ROLL).getITerm();
  pid_msg_.roll.d_term.at(0) = pid_controllers_.at(ROLL).getDTerm();
  pid_msg_.roll.target_p = target_rpy_.x();
  pid_msg_.roll.err_p = pid_controllers_.at(ROLL).getErrP();
  pid_msg_.roll.target_d = target_omega_.x();
  pid_msg_.roll.err_d = pid_controllers_.at(ROLL).getErrD();
  pid_msg_.pitch.total.at(0) = pid_controllers_.at(PITCH).result();
  pid_msg_.pitch.p_term.at(0) = pid_controllers_.at(PITCH).getPTerm();
  pid_msg_.pitch.i_term.at(0) = pid_controllers_.at(PITCH).getITerm();
  pid_msg_.pitch.d_term.at(0) = pid_controllers_.at(PITCH).getDTerm();
  pid_msg_.pitch.target_p = target_rpy_.y();
  pid_msg_.pitch.err_p = pid_controllers_.at(PITCH).getErrP();
  pid_msg_.pitch.target_d = target_omega_.y();
  pid_msg_.pitch.err_d = pid_controllers_.at(PITCH).getErrD();

  Eigen::Vector3d tau_rpy;
  tau_rpy << pid_controllers_.at(ROLL).result(),
            pid_controllers_.at(PITCH).result(),
            candidate_yaw_term_;
  Eigen::VectorXd thrust_for_rpy = q_mat_inv_.rightCols(3) * tau_rpy;

  std::cout << "target_base_thrust_: " << std::endl;
  for (unsigned int i = 0; i < motor_num_; ++i) {
    std::cout << " motor" << (i + 1) << " thrust: " << target_base_thrust_.at(i) << ",";
  }
  std::cout << std::endl;

  std::cout << "thrust_for_rpy: " << std::endl;
  for (unsigned int i = 0; i < motor_num_; ++i) {
    std::cout << " motor" << (i + 1) << " thrust: " << thrust_for_rpy(i)<< ",";
  }
  std::cout << std::endl;
  
  std::cout << "total thrust: " << std::endl;
  for (unsigned int i = 0; i < motor_num_; ++i) {
    std::cout << " motor" << (i + 1) << " thrust: " << target_base_thrust_.at(i) + thrust_for_rpy(i)<< ",";
  }
  std::cout << std::endl;
  std::cout << std::endl;

  ROS_INFO_STREAM_THROTTLE(0.5, "[SoftAirframeController] controlCore");
}

Eigen::MatrixXd SoftAirframeController::getFullQMat()
{
  // wrench allocation matrix
  std::vector<Eigen::Vector3d> rotors_origin = robot_model_->getRotorsOriginFromCog<Eigen::Vector3d>();
  std::vector<Eigen::Vector3d> rotors_normal = robot_model_->getRotorsNormalFromCog<Eigen::Vector3d>();
  auto rotor_direction = robot_model_->getRotorDirection();

  // if (ros::Time::now().toSec() - rotor5_pose_update_time_.toSec() < 1.0 && 
  //     ros::Time::now().toSec() - body_pose_update_time_.toSec() < 1.0){
  //   KDL::Frame body_pose_from_root_ = robot_model_ -> getSegmentsTf().at("fc");
  //   KDL::Frame rotor5_pose_from_root = body_pose_from_root_ * body_pose_from_world_.Inverse() * rotor5_pose_from_world_;
  //   KDL::Frame cog = robot_model_->getCog<KDL::Frame>();
  //   rotors_origin.at(4) = aerial_robot_model::kdlToEigen((cog.Inverse() * rotor5_pose_from_root).p);
  //   rotors_normal.at(4) = aerial_robot_model::kdlToEigen((cog.Inverse() * rotor5_pose_from_root).M * KDL::Vector(0,0,1));
  // }

  // expand for virtual motors
  rotors_origin.push_back(rotors_origin.at(4));
  Eigen::Vector3d v = rotors_normal.at(4);
  rotors_normal.push_back(Eigen::Vector3d(v.x(), -v.z(), v.y()));; // rotate 90 deg around x axis
  rotor_direction.insert(std::make_pair(6, rotor_direction.at(5)));

  Eigen::MatrixXd q_mat = Eigen::MatrixXd::Zero(4, virtual_motor_num_);
  for (unsigned int i = 0; i < virtual_motor_num_; ++i) {
    std::cout << i << "-th rotor origin: " << rotors_origin.at(i).transpose() << std::endl;
    std::cout << i << "-th rotor normla: " <<  rotors_normal.at(i).transpose() << std::endl;
    double m_f_rate = robot_model_->getMFRate(std::min(i, 4u)); // 5th motor is virtual motor
    q_mat(0, i) = rotors_normal.at(i).z();
    q_mat.block(1, i, 3, 1) = (rotors_origin.at(i).cross(rotors_normal.at(i)) + m_f_rate * rotor_direction.at(i + 1) * rotors_normal.at(i));
  }
  double mass_inv = 1.0 / robot_model_->getMass();
  Eigen::Matrix3d inertia_inv = robot_model_->getInertia<Eigen::Matrix3d>().inverse();
  q_mat.topRows(1) =  mass_inv * q_mat.topRows(1) ;
  q_mat.bottomRows(3) =  inertia_inv * q_mat.bottomRows(3);
  return q_mat;
}

Eigen::MatrixXd SoftAirframeController::getQMat()
{
  // wrench allocation matrix
  std::vector<Eigen::Vector3d> rotors_origin = robot_model_->getRotorsOriginFromCog<Eigen::Vector3d>();
  std::vector<Eigen::Vector3d> rotors_normal = robot_model_->getRotorsNormalFromCog<Eigen::Vector3d>();
  auto& rotor_direction = robot_model_->getRotorDirection();

  // if (ros::Time::now().toSec() - rotor5_pose_update_time_.toSec() < 1.0 && 
  //     ros::Time::now().toSec() - body_pose_update_time_.toSec() < 1.0){
  //   KDL::Frame body_pose_from_root_ = robot_model_ -> getSegmentsTf().at("fc");
  //   KDL::Frame rotor5_pose_from_root = body_pose_from_root_ * body_pose_from_world_.Inverse() * rotor5_pose_from_world_;
  //   KDL::Frame cog = robot_model_->getCog<KDL::Frame>();
  //   rotors_origin.at(4) = aerial_robot_model::kdlToEigen((cog.Inverse() * rotor5_pose_from_root).p);
  //   rotors_normal.at(4) = aerial_robot_model::kdlToEigen((cog.Inverse() * rotor5_pose_from_root).M * KDL::Vector(0,0,1));
  // }

  Eigen::MatrixXd q_mat = Eigen::MatrixXd::Zero(4, motor_num_);
  for (unsigned int i = 0; i < motor_num_; ++i) {
    double m_f_rate = robot_model_->getMFRate(i);
    q_mat(0, i) = rotors_normal.at(i).z();
    q_mat.block(1, i, 3, 1) = (rotors_origin.at(i).cross(rotors_normal.at(i)) + m_f_rate * rotor_direction.at(i + 1) * rotors_normal.at(i));
  }
  double mass_inv = 1.0 / robot_model_->getMass();
  Eigen::Matrix3d inertia_inv = robot_model_->getInertia<Eigen::Matrix3d>().inverse();
  q_mat.topRows(1) =  mass_inv * q_mat.topRows(1) ;
  q_mat.bottomRows(3) =  inertia_inv * q_mat.bottomRows(3);
  return q_mat;
}

void SoftAirframeController::sendCmd()
{
  PoseLinearController::sendCmd();

  sendFourAxisCommand();
  sendGimbalCommand();
  sendTorqueAllocationMatrixInv();
  ROS_INFO_STREAM_THROTTLE(0.5, "[SoftAirframeController] sendCmd");
}

void SoftAirframeController::reset()
{
  PoseLinearController::reset();

  setAttitudeGains();
}

void SoftAirframeController::sendFourAxisCommand()
{
  spinal::FourAxisCommand flight_command_data;
  flight_command_data.angles[0] = target_roll_;
  flight_command_data.angles[1] = target_pitch_;
  flight_command_data.angles[2] = candidate_yaw_term_;
  flight_command_data.base_thrust = target_base_thrust_;
  flight_cmd_pub_.publish(flight_command_data);
}

void SoftAirframeController::jointStateCallback(const sensor_msgs::JointState& msg)
{
  gimbal_current_angle = msg.position.at(0); // todo: think a robust implementation
  gimbal_update_time = ros::Time::now();
}

void SoftAirframeController::Rotor5MocapCallback(const geometry_msgs::PoseStamped& msg)
{
  tf2::fromMsg(msg.pose, rotor5_pose_from_world_);
  rotor5_pose_update_time_ = ros::Time::now();
}

void SoftAirframeController::BodyMocapCallback(const geometry_msgs::PoseStamped& msg)
{
  tf2::fromMsg(msg.pose, body_pose_from_world_);
  body_pose_update_time_ = ros::Time::now();
}

void SoftAirframeController::sendGimbalCommand()
{
  if (ros::Time::now().toSec() - gimbal_update_time.toSec() > 1.0) return;

  sensor_msgs::JointState gimbal_control_msg;
  gimbal_control_msg.name.resize(1);
  gimbal_control_msg.name.at(0) = "gimbal_joint1";
  gimbal_control_msg.position.resize(1);
  double dest_pos = gimbal_current_angle + gimbal_angle_diff_;
  if (dest_pos > M_PI/4) dest_pos = M_PI/4;
  if (dest_pos < -M_PI/4) dest_pos = -M_PI/4;
  gimbal_control_msg.position.at(0) = dest_pos; // gimbal joint positive direction is opposite

  gimbal_control_pub_.publish(gimbal_control_msg);
}

void SoftAirframeController::sendTorqueAllocationMatrixInv()
{
  if (ros::Time::now().toSec() - torque_allocation_matrix_inv_pub_stamp_ > torque_allocation_matrix_inv_pub_interval_)
    {
      torque_allocation_matrix_inv_pub_stamp_ = ros::Time::now().toSec();

      spinal::TorqueAllocationMatrixInv torque_allocation_matrix_inv_msg;
      torque_allocation_matrix_inv_msg.rows.resize(motor_num_);
      Eigen::MatrixXd torque_allocation_matrix_inv = q_mat_inv_.rightCols(3);
      if (torque_allocation_matrix_inv.cwiseAbs().maxCoeff() > INT16_MAX * 0.001f)
        ROS_ERROR("Torque Allocation Matrix overflow");
      for (unsigned int i = 0; i < motor_num_; i++)
        {
          torque_allocation_matrix_inv_msg.rows.at(i).x = torque_allocation_matrix_inv(i,0) * 1000;
          torque_allocation_matrix_inv_msg.rows.at(i).y = torque_allocation_matrix_inv(i,1) * 1000;
          torque_allocation_matrix_inv_msg.rows.at(i).z = torque_allocation_matrix_inv(i,2) * 1000;
        }
      torque_allocation_matrix_inv_pub_.publish(torque_allocation_matrix_inv_msg);
    }
}

void SoftAirframeController::rosParamInit()
{
  ros::NodeHandle control_nh(nh_, "controller");
  getParam<bool>(control_nh, "hovering_approximate", hovering_approximate_, false);
  getParam<double>(control_nh, "torque_allocation_matrix_inv_pub_interval", torque_allocation_matrix_inv_pub_interval_, 0.05);
}

void SoftAirframeController::setAttitudeGains()
{
  spinal::RollPitchYawTerms rpy_gain_msg; //for rosserial
  /* to flight controller via rosserial scaling by 1000 */
  rpy_gain_msg.motors.resize(1);
  rpy_gain_msg.motors.at(0).roll_p = pid_controllers_.at(ROLL).getPGain() * 1000;
  rpy_gain_msg.motors.at(0).roll_i = pid_controllers_.at(ROLL).getIGain() * 1000;
  rpy_gain_msg.motors.at(0).roll_d = pid_controllers_.at(ROLL).getDGain() * 1000;
  rpy_gain_msg.motors.at(0).pitch_p = pid_controllers_.at(PITCH).getPGain() * 1000;
  rpy_gain_msg.motors.at(0).pitch_i = pid_controllers_.at(PITCH).getIGain() * 1000;
  rpy_gain_msg.motors.at(0).pitch_d = pid_controllers_.at(PITCH).getDGain() * 1000;
  rpy_gain_msg.motors.at(0).yaw_d = pid_controllers_.at(YAW).getDGain() * 1000;
  rpy_gain_pub_.publish(rpy_gain_msg);
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::SoftAirframeController, aerial_robot_control::ControlBase);
