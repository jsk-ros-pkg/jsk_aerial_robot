#include <trirotor/control/trirotor_controller.h>

using namespace aerial_robot_control;

TrirotorController::TrirotorController():
  PoseLinearController()
{
}

void TrirotorController::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                   boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                   boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                   boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                   double ctrl_loop_rate
                                   )
{
  PoseLinearController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);
  trirotor_robot_model_ = boost::dynamic_pointer_cast<TrirotorRobotModel>(robot_model);

  rosParamInit();

  target_base_thrust_.resize(motor_num_);
  target_gimbal_angles_.resize(motor_num_, 0);

  flight_cmd_pub_ = nh_.advertise<spinal::FourAxisCommand>("four_axes/command", 1);
  gimbal_control_pub_ = nh_.advertise<sensor_msgs::JointState>("gimbals_ctrl", 1);
  target_vectoring_force_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("debug/target_vectoring_force", 1);

}

void TrirotorController::rosParamInit()
{
  ros::NodeHandle control_nh(nh_, "controller");
  getParam<bool>(control_nh, "hovering_approximate", hovering_approximate_, false);
}

void TrirotorController::controlCore()
{
  std::vector<Eigen::MatrixXd> mask = trirotor_robot_model_->getRotorMasks();
  PoseLinearController::controlCore();

  tf::Matrix3x3 uav_rot = estimator_->getOrientation(Frame::COG, estimate_mode_);
  tf::Vector3 target_acc_w(pid_controllers_.at(X).result(),
                           pid_controllers_.at(Y).result(),
                           pid_controllers_.at(Z).result());
  tf::Vector3 target_acc_cog = uav_rot.inverse() * target_acc_w;
  Eigen::VectorXd target_wrench_acc_cog = Eigen::VectorXd::Zero(6);
  target_wrench_acc_cog.head(3) = Eigen::Vector3d(target_acc_cog.x(), target_acc_cog.y(), target_acc_cog.z());

  double target_ang_acc_x = pid_controllers_.at(ROLL).result();
  double target_ang_acc_y = pid_controllers_.at(PITCH).result();
  double target_ang_acc_z = pid_controllers_.at(YAW).result();
  target_wrench_acc_cog.tail(3) = Eigen::Vector3d(target_ang_acc_x, target_ang_acc_y, target_ang_acc_z);

  // coversion of target wrench from cog frame to baselink frame
  std::vector<KDL::Rotation> links_frame_from_cog = trirotor_robot_model_->getLinksRotationFromCog<KDL::Rotation>();
  tf::Quaternion q;  tf::quaternionKDLToTF(links_frame_from_cog.at(0), q);
  Eigen::Matrix3d base_rot_ag_cog; tf::matrixTFToEigen(tf::Matrix3x3(q),base_rot_ag_cog);

  Eigen::VectorXd target_wrench_acc_base = Eigen::VectorXd::Zero(6);
  target_wrench_acc_base.head(3) = base_rot_ag_cog.transpose() * target_wrench_acc_cog.head(3);
  target_wrench_acc_base.tail(3) = base_rot_ag_cog.transpose() * target_wrench_acc_cog.tail(3);

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

  Eigen::Matrix3d inertia_inv = (base_rot_ag_cog.transpose() * trirotor_robot_model_->getInertia<Eigen::Matrix3d>()).inverse();
  double mass_inv = 1 / trirotor_robot_model_->getMass();

  double t = ros::Time::now().toSec();

  std::vector<Eigen::Vector3d> rotors_origin_from_cog = trirotor_robot_model_->getRotorsOriginFromCog<Eigen::Vector3d>();
  Eigen::MatrixXd wrench_map = Eigen::MatrixXd::Zero(6, 3);
  wrench_map.block(0, 0, 3, 3) =  Eigen::MatrixXd::Identity(3, 3);
  int last_col = 0;
  for(int i = 0; i < motor_num_; i++){
    wrench_map.block(3, 0, 3, 3) = aerial_robot_model::skew(base_rot_ag_cog.transpose() *  rotors_origin_from_cog.at(i));
    full_q_mat.middleCols(last_col, 2) = wrench_map * mask[i];
    last_col += 2;
  }

  full_q_mat.topRows(3) = mass_inv * full_q_mat.topRows(3);
  full_q_mat.bottomRows(3) = inertia_inv * full_q_mat.bottomRows(3);
  Eigen::MatrixXd full_q_mat_inv = aerial_robot_model::pseudoinverse(full_q_mat);
  target_vectoring_f_ = full_q_mat_inv * target_wrench_acc_base;
  last_col = 0;
  for(int i = 0; i < motor_num_; i++){
    Eigen::VectorXd f_i = target_vectoring_f_.segment(last_col, 2);

    target_base_thrust_.at(i) = f_i.norm();
    target_gimbal_angles_.at(i) = atan2(f_i[0], f_i[1]);
    target_gimbal_angles_.at(i) = atan2(f_i[0], f_i[1]);

    last_col += 2;
  }
}

void TrirotorController::sendCmd()
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
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::TrirotorController, aerial_robot_control::ControlBase);
