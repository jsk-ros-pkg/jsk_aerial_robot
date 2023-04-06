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

  tf::Vector3 target_acc_w(pid_controllers_.at(X).result(),
                           pid_controllers_.at(Y).result(),
                           pid_controllers_.at(Z).result());

  double target_ang_acc_x = pid_controllers_.at(ROLL).result();
  double target_ang_acc_y = pid_controllers_.at(PITCH).result();
  double target_ang_acc_z = pid_controllers_.at(YAW).result();

  tf::Vector3 target_acc_dash = (tf::Matrix3x3(tf::createQuaternionFromYaw(rpy_.z()))).inverse() * target_acc_w;

  float target_roll;
  float target_pitch;
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

  pid_msg_.roll.total.at(0) = target_ang_acc_x;
  pid_msg_.roll.p_term.at(0) = pid_controllers_.at(ROLL).getPTerm();
  pid_msg_.roll.i_term.at(0) = pid_controllers_.at(ROLL).getITerm();
  pid_msg_.roll.d_term.at(0) = pid_controllers_.at(ROLL).getDTerm();
  pid_msg_.roll.target_p = target_rpy_.x();
  pid_msg_.roll.err_p = pid_controllers_.at(ROLL).getErrP();
  pid_msg_.roll.target_d = target_omega_.x();
  pid_msg_.roll.err_d = pid_controllers_.at(ROLL).getErrD();
  pid_msg_.pitch.total.at(0) = target_ang_acc_y;
  pid_msg_.pitch.p_term.at(0) = pid_controllers_.at(PITCH).getPTerm();
  pid_msg_.pitch.i_term.at(0) = pid_controllers_.at(PITCH).getITerm();
  pid_msg_.pitch.d_term.at(0) = pid_controllers_.at(PITCH).getDTerm();
  pid_msg_.pitch.target_p = target_rpy_.y();
  pid_msg_.pitch.err_p = pid_controllers_.at(PITCH).getErrP();
  pid_msg_.pitch.target_d = target_omega_.y();
  pid_msg_.pitch.err_d = pid_controllers_.at(PITCH).getErrD();

  Eigen::MatrixXd full_q_mat = Eigen::MatrixXd::Zero(6, 2 * motor_num_);

  Eigen::Matrix3d inertia_inv = robot_model_->getInertia<Eigen::Matrix3d>().inverse();
  double mass_inv = 1 / robot_model_->getMass();

  double t = ros::Time::now().toSec();

  std::vector<Eigen::Vector3d> rotors_origin_from_cog = robot_model_->getRotorsOriginFromCog<Eigen::Vector3d>();
  Eigen::MatrixXd wrench_map = Eigen::MatrixXd::Zero(6, 3);
  wrench_map.block(0, 0, 3, 3) =  Eigen::MatrixXd::Identity(3, 3);
  Eigen::MatrixXd mask(3, 2);
  mask << 1, 0, 0, 0, 0, 1;
  int last_col = 0;
  for(int i = 0; i < motor_num_; i++){
    wrench_map.block(3, 0, 3, 3) = aerial_robot_model::skew(rotors_origin_from_cog.at(i));
    full_q_mat.middleCols(last_col, 2) = wrench_map * mask;
    last_col += 2;
  }

  Eigen::MatrixXd allocation_mat = Eigen::MatrixXd::Zero(4, 4);
  full_q_mat.topRows(3) = mass_inv * full_q_mat.topRows(3);
  full_q_mat.bottomRows(3) = inertia_inv * full_q_mat.bottomRows(3);

  allocation_mat.topRows(1) = full_q_mat.row(2);
  allocation_mat.bottomRows(3) = full_q_mat.bottomRows(3);

  Eigen::MatrixXd allocation_mat_inv = allocation_mat.inverse();
  target_vectoring_f_ = allocation_mat_inv * target_wrench_acc_cog;

  last_col = 0;
  for(int i = 0; i < motor_num_; i++){
    Eigen::VectorXd f_i = target_vectoring_f_.segment(last_col, 2);
    target_base_thrust_.at(i) = f_i.norm();

    target_gimbal_angles_.at(i) = atan2(f_i(0), f_i(1));

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
