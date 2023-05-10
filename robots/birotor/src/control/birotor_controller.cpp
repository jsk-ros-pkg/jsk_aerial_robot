#include <birotor/control/birotor_controller.h>

using namespace aerial_robot_control;

BirotorController::BirotorController():
  PoseLinearController()
{
}

void BirotorController::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                   boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                   boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                   boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                   double ctrl_loop_rate
                                   )
{
  PoseLinearController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);

  birotor_robot_model_ = boost::dynamic_pointer_cast<BirotorRobotModel>(robot_model);

  rosParamInit();

  target_base_thrust_.resize(motor_num_);
  target_gimbal_angles_.resize(motor_num_, 0);

  flight_cmd_pub_ = nh_.advertise<spinal::FourAxisCommand>("four_axes/command", 1);
  gimbal_control_pub_ = nh_.advertise<sensor_msgs::JointState>("gimbals_ctrl", 1);
  target_vectoring_force_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("debug/target_vectoring_force", 1);
}

void BirotorController::rosParamInit()
{
  ros::NodeHandle control_nh(nh_, "controller");
  getParam<bool>(control_nh, "hovering_approximate", hovering_approximate_, false);
}

void BirotorController::controlCore()
{
  PoseLinearController::controlCore();

  // wrench allocation matrix
  const std::vector<Eigen::Vector3d> rotors_origin_from_cog = robot_model_->getRotorsOriginFromCog<Eigen::Vector3d>();
  double uav_mass_inv = 1.0 / robot_model_->getMass();
  Eigen::Matrix3d inertia_inv = robot_model_->getInertia<Eigen::Matrix3d>().inverse();

  Eigen::MatrixXd q_mat_four_axis = Eigen::MatrixXd::Zero(4, 3 * motor_num_);
  Eigen::MatrixXd wrench_map_four_axis =  Eigen::MatrixXd::Zero(4, 3);
  wrench_map_four_axis(0, 2) = 1 * uav_mass_inv;
  int last_col = 0;
  for(int i = 0; i < motor_num_; i++)
    {
      wrench_map_four_axis.block(1, 0, 3, 3) = aerial_robot_model::skew(rotors_origin_from_cog.at(i));
      q_mat_four_axis.middleCols(last_col, 3) = wrench_map_four_axis;
      last_col += 3;
    }
  q_mat_four_axis.bottomRows(3) = inertia_inv * q_mat_four_axis.bottomRows(3);

  /* calculate masked rotation matrix */
  std::vector<KDL::Rotation> rotors_coord_rot = birotor_robot_model_->getRotorsCoordRot<KDL::Rotation>();
  std::vector<Eigen::MatrixXd> masked_rot;
  for(int i = 0; i < motor_num_; i++)
    {
      tf::Quaternion r;  tf::quaternionKDLToTF(rotors_coord_rot.at(i), r);
      Eigen::Matrix3d conv_cog_from_thrust; tf::matrixTFToEigen(tf::Matrix3x3(r), conv_cog_from_thrust);
      Eigen::MatrixXd mask(3, 2);
      mask << 0, 0, 1, 0, 0, 1;
      masked_rot.push_back(conv_cog_from_thrust * mask);
    }

  /* calculate integrated allocation */
  Eigen::MatrixXd integrated_rot = Eigen::MatrixXd::Zero(3 * motor_num_, 2 * motor_num_);
  for(int i = 0; i < motor_num_; i++)
    {
      integrated_rot.block(3 * i, 2 * i, 3, 2) = masked_rot[i];
    }

  Eigen::MatrixXd q_mat = q_mat_four_axis * integrated_rot;
  Eigen::MatrixXd q_mat_inv = aerial_robot_model::pseudoinverse(q_mat);

  tf::Vector3 target_acc_w(pid_controllers_.at(X).result(),
                           pid_controllers_.at(Y).result(),
                           pid_controllers_.at(Z).result());

  tf::Vector3 target_acc_dash = (tf::Matrix3x3(tf::createQuaternionFromYaw(rpy_.z()))).inverse() * target_acc_w;

  double target_ang_acc_x = pid_controllers_.at(ROLL).result();
  double target_ang_acc_y = pid_controllers_.at(PITCH).result();
  double target_ang_acc_z = pid_controllers_.at(YAW).result();

  double target_roll, target_pitch;
  if(hovering_approximate_)
    {
      target_pitch = target_acc_dash.x() / aerial_robot_estimation::G;
      target_roll = -target_acc_dash.y() / aerial_robot_estimation::G;
      navigator_->setTargetRoll(target_roll);
      navigator_->setTargetPitch(target_pitch);
    }
  else
    {
      target_pitch = atan2(target_acc_dash.x(), target_acc_dash.z());
      target_roll = atan2(-target_acc_dash.y(), sqrt(target_acc_dash.x() * target_acc_dash.x() + target_acc_dash.z() * target_acc_dash.z()));
      navigator_->setTargetRoll(target_roll);
      navigator_->setTargetPitch(target_pitch);
    }

  Eigen::VectorXd target_wrench_acc_cog = Eigen::VectorXd::Zero(4);

  target_wrench_acc_cog(0) = target_acc_dash.z();
  target_wrench_acc_cog.tail(3) = Eigen::Vector3d(pid_controllers_.at(ROLL).result(), pid_controllers_.at(PITCH).result(), pid_controllers_.at(YAW).result());

  target_vectoring_f_ = q_mat_inv * target_wrench_acc_cog;

  last_col = 0;
  for(int i = 0; i < motor_num_; i++){
    Eigen::VectorXd f_i = target_vectoring_f_.segment(last_col, 2);
    target_base_thrust_.at(i) = f_i.norm();

    target_gimbal_angles_.at(i) = atan2(-f_i(0), f_i(1));

    last_col += 2;
  }
}

void BirotorController::sendCmd()
{
  PoseLinearController::sendCmd();

  spinal::FourAxisCommand flight_command_data;
  flight_command_data.base_thrust = target_base_thrust_;
  flight_cmd_pub_.publish(flight_command_data);

  sensor_msgs::JointState gimbal_control_msg;
  gimbal_control_msg.header.stamp = ros::Time::now();
  for(int i = 0; i < motor_num_; i++){
    gimbal_control_msg.position.push_back(target_gimbal_angles_.at(i));
  }
  gimbal_control_pub_.publish(gimbal_control_msg);

  std_msgs::Float32MultiArray target_vectoring_force_msg;
  for(int i = 0; i < target_vectoring_f_.size(); i++){
    target_vectoring_force_msg.data.push_back(target_vectoring_f_(i));
  }
  target_vectoring_force_pub_.publish(target_vectoring_force_msg);

}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::BirotorController, aerial_robot_control::ControlBase);
