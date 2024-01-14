#include <delta/control/delta_controller.h>

using namespace aerial_robot_model;
using namespace aerial_robot_control;

RollingController::RollingController():
  PoseLinearController(),
  torque_allocation_matrix_inv_pub_stamp_(0)
{
}

void RollingController::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                   boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                   boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                   boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                   double ctrl_loop_rate)
{
  PoseLinearController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);

  rolling_navigator_ = boost::dynamic_pointer_cast<aerial_robot_navigation::RollingNavigator>(navigator_);
  rolling_robot_model_ = boost::dynamic_pointer_cast<RollingRobotModel>(robot_model_);
  rolling_robot_model_for_opt_ = boost::make_shared<RollingRobotModel>();
  robot_model_for_control_ = boost::make_shared<aerial_robot_model::RobotModel>();

  rotor_tilt_.resize(motor_num_);
  target_base_thrust_.resize(motor_num_);
  target_gimbal_angles_.resize(motor_num_, 0);
  prev_target_gimbal_angles_.resize(motor_num_, 0);
  current_gimbal_angles_.resize(motor_num_, 0);

  target_wrench_acc_cog_.resize(6);
  controlled_wrench_acc_cog_.resize(6);

  target_acc_cog_.resize(6);
  target_acc_dash_.resize(6);

  full_lambda_trans_.resize(2 * motor_num_);
  full_lambda_rot_.resize(2 * motor_num_);
  full_lambda_all_.resize(2 * motor_num_);

  full_q_mat_.resize(6, 2 * motor_num_);

  rosParamInit();

  target_roll_ = 0.0;
  target_pitch_ = 0.0;
  target_thrust_z_term_.resize(2 * motor_num_);

  rolling_control_timestamp_ = -1;

  rpy_gain_pub_ = nh_.advertise<spinal::RollPitchYawTerms>("rpy/gain", 1);
  flight_cmd_pub_ = nh_.advertise<spinal::FourAxisCommand>("four_axes/command", 1);
  gimbal_control_pub_ = nh_.advertise<sensor_msgs::JointState>("gimbals_ctrl", 1);
  torque_allocation_matrix_inv_pub_ = nh_.advertise<spinal::TorqueAllocationMatrixInv>("torque_allocation_matrix_inv", 1);
  gimbal_dof_pub_ = nh_.advertise<std_msgs::UInt8>("gimbal_dof", 1);
  gimbal_indices_pub_ = nh_.advertise<std_msgs::UInt8MultiArray>("gimbal_indices", 1);

  desire_coordinate_pub_ = nh_.advertise<spinal::DesireCoord>("desire_coordinate", 1);
  target_vectoring_force_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("debug/target_vectoring_force", 1);
  target_wrench_acc_cog_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("debug/target_wrench_acc_cog", 1);
  wrench_allocation_matrix_pub_ = nh_.advertise<aerial_robot_msgs::WrenchAllocationMatrix>("debug/wrench_allocation_matrix", 1);
  full_q_mat_pub_ = nh_.advertise<aerial_robot_msgs::WrenchAllocationMatrix>("debug/full_q_mat", 1);
  operability_pub_ = nh_.advertise<std_msgs::Float32>("debug/operability", 1);
  target_acc_cog_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("debug/target_acc_cog", 1);
  target_acc_dash_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("debug/target_acc_dash", 1);
  exerted_wrench_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("debug/exerted_wrench_cog", 1);

  joint_state_sub_ = nh_.subscribe("joint_states", 1, &RollingController::jointStateCallback, this);
  calc_gimbal_in_fc_sub_ = nh_.subscribe("calc_gimbal_in_fc", 1, &RollingController::calcGimbalInFcCallback, this);

  ground_navigation_mode_ = aerial_robot_navigation::FLYING_STATE;

  control_dof_ = std::accumulate(controlled_axis_.begin(), controlled_axis_.end(), 0);

  controlled_q_mat_.resize(control_dof_, 2 * motor_num_);
  controlled_q_mat_inv_.resize(2 * motor_num_, control_dof_);
  q_mat_.resize(control_dof_, motor_num_);
  q_mat_inv_.resize(motor_num_, control_dof_);

}

void RollingController::reset()
{
  PoseLinearController::reset();

  setControllerParams("controller");
  setControlAxisWithNameSpace("controller");
  setAttitudeGains();

  // standing_target_phi_ = 0.0;
  standing_baselink_ref_pitch_last_update_time_ = -1;
  // standing_target_baselink_pitch_ = 0.0;
  rolling_control_timestamp_ = -1;

  calc_gimbal_in_fc_ = false;

  std_msgs::UInt8 gimbal_dof_msg;
  gimbal_dof_msg.data = 0;
  gimbal_dof_pub_.publish(gimbal_dof_msg);

  std_msgs::UInt8MultiArray gimbal_indices_msg;
  gimbal_indices_msg.data.resize(motor_num_);
  gimbal_indices_msg.data.at(0) = 0;
  gimbal_indices_msg.data.at(1) = 2;
  gimbal_indices_msg.data.at(2) = 4;
  gimbal_indices_pub_.publish(gimbal_indices_msg);

  setControllerParams("standing_controller");

  ROS_INFO_STREAM("[control] reset controller\n");
}

void RollingController::rosParamInit()
{
  ros::NodeHandle control_nh(nh_, "controller");

  getParam<double>(control_nh, "torque_allocation_matrix_inv_pub_interval", torque_allocation_matrix_inv_pub_interval_, 0.05);
  getParam<bool>(control_nh, "use_sr_inv", use_sr_inv_, false);
  getParam<double>(control_nh, "sr_inv_weight", sr_inv_weight_, 0.0);
  getParam<bool>(control_nh, "hovering_approximate", hovering_approximate_, false);
  getParam<bool>(control_nh, "calc_gimbal_in_fc", calc_gimbal_in_fc_, false);
  getParam<double>(control_nh, "gimbal_lpf_factor",gimbal_lpf_factor_, 1.0);
  getParam<double>(nh_, "circle_radius", circle_radius_, 0.5);
  getParam<bool>(control_nh, "realtime_gimbal_allocation", realtime_gimbal_allocation_, false);
  getParam<double>(control_nh, "gravity_compensate_ratio", gravity_compensate_ratio_, 1.0);

  rolling_robot_model_->setCircleRadius(circle_radius_);
  rolling_robot_model_for_opt_->setCircleRadius(circle_radius_);

  getParam<bool>(control_nh, "standing_baselink_pitch_update", standing_baselink_pitch_update_, false);
  getParam<double>(control_nh, "standing_baselink_ref_pitch_update_thresh", standing_baselink_ref_pitch_update_thresh_, 1.0);
  getParam<double>(control_nh, "steering_mu", steering_mu_, 0.0);

  getParam<string>(nhp_, "tf_prefix", tf_prefix_, std::string(""));

  double rotor_tilt1;
  double rotor_tilt2;
  double rotor_tilt3;
  getParam<double>(control_nh, "rotor_tilt1", rotor_tilt1, 0.0);
  rotor_tilt_.at(0) = rotor_tilt1;
  getParam<double>(control_nh, "rotor_tilt2", rotor_tilt2, 0.0);
  rotor_tilt_.at(1) = rotor_tilt2;
  getParam<double>(control_nh, "rotor_tilt3", rotor_tilt3, 0.0);
  rotor_tilt_.at(2) = rotor_tilt3;

  rosoutControlParams("controller");
  rosoutControlParams("standing_controller");

  controlled_axis_.resize(6, 1);
  rosoutControlAxis("controller");
  rosoutControlAxis("standing_controller");
}

void RollingController::controlCore()
{
  PoseLinearController::controlCore();

  ground_navigation_mode_ = rolling_navigator_->getCurrentGroundNavigationMode();

  setAttitudeGains();

  control_dof_ = std::accumulate(controlled_axis_.begin(), controlled_axis_.end(), 0);

  /* for stand */
  rolling_robot_model_->setTargetFrameName("cp");
  standingPlanning();
  calcStandingFullLambda();
  /* for stand */


  /* for flight */
  // rolling_robot_model_->setTargetFrameName("cog");
  // setControllerParams("controller");
  // calcAccFromCog();
  // calcWrenchAllocationMatrix();
  // calcFullLambda();
  /* for flight */

  /* common part */
  wrenchAllocation();
  calcYawTerm();
  /* common part */

  rolling_navigator_->setPrevGroundNavigationMode(ground_navigation_mode_);
}

void RollingController::calcYawTerm()
{
  if(controlled_axis_.at(YAW) == 0)
    {
      candidate_yaw_term_ = 0;
      ROS_ERROR("[control] control axis around YAW is false");
      return;
    }

  // special process for yaw since the bandwidth between PC and spinal
  double max_yaw_scale = 0; // for reconstruct yaw control term in spinal
  Eigen::MatrixXd full_q_mat_inv = aerial_robot_model::pseudoinverse(full_q_mat_);
  int torque_allocation_matrix_inv_rows;
  if(calc_gimbal_in_fc_) torque_allocation_matrix_inv_rows = 2 * motor_num_;
  else torque_allocation_matrix_inv_rows = motor_num_;

  for (unsigned int i = 0; i < torque_allocation_matrix_inv_rows; i++)
    {
      if(calc_gimbal_in_fc_)
        {
          if(full_q_mat_inv(i, YAW) > max_yaw_scale) max_yaw_scale = full_q_mat_inv(i, YAW);
        }
      else
        {
          if(q_mat_inv_(i, control_dof_ - 1) > max_yaw_scale) max_yaw_scale = q_mat_inv_(i, control_dof_ - 1);
        }
    }
      candidate_yaw_term_ = pid_controllers_.at(YAW).result() * max_yaw_scale;
}

void RollingController::sendCmd()
{
  PoseLinearController::sendCmd();

  std_msgs::Float32MultiArray target_vectoring_force_msg;
  for(int i = 0; i < full_lambda_all_.size(); i++)
    {
      target_vectoring_force_msg.data.push_back(full_lambda_all_(i));
    }
  target_vectoring_force_pub_.publish(target_vectoring_force_msg);

  std_msgs::Float32MultiArray target_wrench_acc_cog_msg;
  for(int i = 0; i < target_wrench_acc_cog_.size(); i++)
    {
      target_wrench_acc_cog_msg.data.push_back(target_wrench_acc_cog_(i));
    }
  target_wrench_acc_cog_pub_.publish(target_wrench_acc_cog_msg);

  aerial_robot_msgs::WrenchAllocationMatrix wrench_allocation_matrix_msg;
  for(int i = 0; i < q_mat_.cols(); i++)
    {
      wrench_allocation_matrix_msg.f_x.push_back(q_mat_(0, i));
      wrench_allocation_matrix_msg.f_y.push_back(q_mat_(1, i));
      wrench_allocation_matrix_msg.f_z.push_back(q_mat_(2, i));
      wrench_allocation_matrix_msg.t_x.push_back(q_mat_(3, i));
      wrench_allocation_matrix_msg.t_y.push_back(q_mat_(4, i));
      wrench_allocation_matrix_msg.t_z.push_back(q_mat_(5, i));
    }
  wrench_allocation_matrix_pub_.publish(wrench_allocation_matrix_msg);

  std_msgs::Float32MultiArray target_acc_cog_msg;
  for(int i = 0; i < target_acc_cog_.size(); i++)
    {
      target_acc_cog_msg.data.push_back(target_acc_cog_.at(i));
    }
  target_acc_cog_pub_.publish(target_acc_cog_msg);

  std_msgs::Float32MultiArray target_acc_dash_msg;
  for(int i = 0; i < target_acc_dash_.size(); i++)
    {
      target_acc_dash_msg.data.push_back(target_acc_dash_.at(i));
    }
  target_acc_dash_pub_.publish(target_acc_dash_msg);

  aerial_robot_msgs::WrenchAllocationMatrix full_q_mat_msg;
  for(int i = 0; i < 2 * motor_num_; i++)
    {
      full_q_mat_msg.f_x.push_back(full_q_mat_(0, i));
      full_q_mat_msg.f_y.push_back(full_q_mat_(1, i));
      full_q_mat_msg.f_z.push_back(full_q_mat_(2, i));
      full_q_mat_msg.t_x.push_back(full_q_mat_(3, i));
      full_q_mat_msg.t_y.push_back(full_q_mat_(4, i));
      full_q_mat_msg.t_z.push_back(full_q_mat_(5, i));
    }
  full_q_mat_pub_.publish(full_q_mat_msg);

  std_msgs::Float32MultiArray exerted_wrench_msg;
  Eigen::VectorXd exerted_wrench = full_q_mat_ * full_lambda_all_;
  for(int i = 0; i < exerted_wrench.size(); i++)
    {
      exerted_wrench_msg.data.push_back(exerted_wrench(i));
    }
  exerted_wrench_pub_.publish(exerted_wrench_msg);

  std_msgs::Float32 operability_msg;
  Eigen::MatrixXd q_qt;
  q_qt = controlled_q_mat_ * controlled_q_mat_.transpose();
  float det = q_qt.determinant();
  operability_msg.data =sqrt(det);
  operability_pub_.publish(operability_msg);

  if(!calc_gimbal_in_fc_)
    sendGimbalAngles();

  sendFourAxisCommand();

  sendTorqueAllocationMatrixInv();
}

void RollingController::sendGimbalAngles()
{
  sensor_msgs::JointState gimbal_control_msg;
  gimbal_control_msg.header.stamp = ros::Time::now();
  for(int i = 0; i < motor_num_; i++){
    gimbal_control_msg.position.push_back(target_gimbal_angles_.at(i));
    gimbal_control_msg.name.push_back(std::string("gimbal") + std::to_string(i + 1));
  }
  gimbal_control_pub_.publish(gimbal_control_msg);
}

void RollingController::sendFourAxisCommand()
{
  spinal::FourAxisCommand flight_command_data;
  flight_command_data.angles[2] = candidate_yaw_term_;
  if(calc_gimbal_in_fc_)
    {
      flight_command_data.base_thrust.resize(2 * motor_num_);
      for(int i = 0; i < 2 * motor_num_; i++)
        {
          flight_command_data.base_thrust.at(i) = full_lambda_trans_(i);
        }
    }
  else
    flight_command_data.base_thrust = target_base_thrust_;

  if(!controlled_axis_[0])
    {
      flight_command_data.angles[1] = target_pitch_;
    }
  if(!controlled_axis_[1])
    {
      flight_command_data.angles[0] = target_roll_;
    }
  flight_cmd_pub_.publish(flight_command_data);
}

void RollingController::sendTorqueAllocationMatrixInv()
{
  if (ros::Time::now().toSec() - torque_allocation_matrix_inv_pub_stamp_ > torque_allocation_matrix_inv_pub_interval_)
    {
      torque_allocation_matrix_inv_pub_stamp_ = ros::Time::now().toSec();

      spinal::TorqueAllocationMatrixInv torque_allocation_matrix_inv_msg;
      Eigen::MatrixXd torque_allocation_matrix_inv;
      int torque_allocation_matrix_inv_msg_rows;
      if(calc_gimbal_in_fc_)
        {
          torque_allocation_matrix_inv_msg.rows.resize(2 * motor_num_);
          torque_allocation_matrix_inv = aerial_robot_model::pseudoinverse(full_q_mat_).rightCols(3);
          torque_allocation_matrix_inv_msg_rows = 2 * motor_num_;
        }
      else
        {
          torque_allocation_matrix_inv_msg.rows.resize(motor_num_);
          torque_allocation_matrix_inv = q_mat_inv_.rightCols(3);
          torque_allocation_matrix_inv_msg_rows = motor_num_;
        }

      if (torque_allocation_matrix_inv.cwiseAbs().maxCoeff() > INT16_MAX * 0.001f)
        ROS_ERROR("Torque Allocation Matrix overflow");
      for (unsigned int i = 0; i < torque_allocation_matrix_inv_msg_rows; i++)
        {
          torque_allocation_matrix_inv_msg.rows.at(i).x = torque_allocation_matrix_inv(i,0) * 1000;
          torque_allocation_matrix_inv_msg.rows.at(i).y = torque_allocation_matrix_inv(i,1) * 1000;
          torque_allocation_matrix_inv_msg.rows.at(i).z = torque_allocation_matrix_inv(i,2) * 1000;
        }
      torque_allocation_matrix_inv_pub_.publish(torque_allocation_matrix_inv_msg);
    }
}

void RollingController::setAttitudeGains()
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

void RollingController::jointStateCallback(const sensor_msgs::JointStateConstPtr & state)
{
  sensor_msgs::JointState joint_state = *state;

  /* get current gimbal angles */
  for(int i = 0; i < joint_state.name.size(); i++)
    {
      if(joint_state.name.at(i).find("gimbal") != string::npos)
        {
          for(int j = 0; j < motor_num_; j++)
            {
              if(joint_state.name.at(i) == std::string("gimbal") + std::to_string(j + 1))
                {
                  std::lock_guard<std::mutex> lock(current_gimbal_angles_mutex_);
                  current_gimbal_angles_.at(j) = joint_state.position.at(i);
                }
            }
        }
    }

  /* tf of contact point */
  geometry_msgs::TransformStamped contact_point_tf = rolling_robot_model_->getContactPoint<geometry_msgs::TransformStamped>();
  contact_point_tf.header = state->header;
  contact_point_tf.header.frame_id = tf::resolve(tf_prefix_, std::string("root"));
  contact_point_tf.child_frame_id = tf::resolve(tf_prefix_, std::string("contact_point"));
  br_.sendTransform(contact_point_tf);

  /* tf of center point */
  geometry_msgs::TransformStamped center_point_tf = rolling_robot_model_->getCenterPoint<geometry_msgs::TransformStamped>();
  center_point_tf.header = state->header;
  center_point_tf.header.frame_id = tf::resolve(tf_prefix_, std::string("root"));
  center_point_tf.child_frame_id = tf::resolve(tf_prefix_, std::string("center_point"));
  br_.sendTransform(center_point_tf);

  /* tf of contact point alined to ground plane */
  KDL::Frame cog = robot_model_->getCog<KDL::Frame>();
  KDL::Frame contact_point = rolling_robot_model_->getContactPoint<KDL::Frame>();
  double baselink_roll = estimator_->getEuler(Frame::COG, estimate_mode_).x();
  double baselink_pitch = estimator_->getEuler(Frame::COG, estimate_mode_).y();

  if(true)
    {
      std::lock_guard<std::mutex> lock(contact_point_alined_mutex_);
      contact_point_alined_.p = contact_point.p;
      contact_point_alined_.M = cog.M;
      contact_point_alined_.M.DoRotX(-baselink_roll);
      contact_point_alined_.M.DoRotY(-baselink_pitch);
    }
  geometry_msgs::TransformStamped contact_point_alined_tf = kdlToMsg(contact_point_alined_);
  contact_point_alined_tf.header = state->header;
  contact_point_alined_tf.header.frame_id = tf::resolve(tf_prefix_, std::string("root"));
  contact_point_alined_tf.child_frame_id = tf::resolve(tf_prefix_, std::string("contact_point_alined"));
  br_.sendTransform(contact_point_alined_tf);

}

void RollingController::calcGimbalInFcCallback(const std_msgs::BoolPtr & msg)
{
  if(calc_gimbal_in_fc_ != msg->data)
    {
      std_msgs::UInt8 gimbal_dof_msg;
      if(msg->data)
        {
          gimbal_dof_msg.data = 1;
          ROS_WARN("[control] calc gimbal in fc.");
        }
      else
        {
          gimbal_dof_msg.data = 0;
          ROS_WARN("[control] not calc gimbal in fc.");
        }
      gimbal_dof_pub_.publish(gimbal_dof_msg);
      calc_gimbal_in_fc_ = msg->data ? true : false;
    }
  else
    {
      return;
    }
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::RollingController, aerial_robot_control::ControlBase);
