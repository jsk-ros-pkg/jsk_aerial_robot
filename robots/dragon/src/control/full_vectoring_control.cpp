#include <dragon/control/full_vectoring_control.h>

using namespace aerial_robot_model;
using namespace aerial_robot_control;

DragonFullVectoringController::DragonFullVectoringController():
  PoseLinearController()
{
}

void DragonFullVectoringController::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                     boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                     boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                     boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                     double ctrl_loop_rate)
{
  PoseLinearController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);
  rosParamInit();

  dragon_robot_model_ = boost::dynamic_pointer_cast<Dragon::FullVectoringRobotModel>(robot_model);

  /* initialize the gimbal target angles */
  target_base_thrust_.resize(motor_num_);
  target_gimbal_angles_.resize(motor_num_ * 2, 0);

  gimbal_control_pub_ = nh_.advertise<sensor_msgs::JointState>("gimbals_ctrl", 1);
  flight_cmd_pub_ = nh_.advertise<spinal::FourAxisCommand>("four_axes/command", 1);
  target_vectoring_force_pub_ = nh_.advertise<sensor_msgs::JointState>("debug/target_vectoring_force", 1);
}

void DragonFullVectoringController::controlCore()
{
  /* TODO: saturation of z control */
  PoseLinearController::controlCore();

  tf::Matrix3x3 uav_rot = estimator_->getOrientation(Frame::COG, estimate_mode_);
  tf::Vector3 target_acc_w(pid_controllers_.at(X).result(),
                           pid_controllers_.at(Y).result(),
                           pid_controllers_.at(Z).result());
  tf::Vector3 target_acc_cog = uav_rot.inverse() * target_acc_w;
  Eigen::VectorXd target_wrench_cog = Eigen::VectorXd::Zero(6);
  target_wrench_cog.head(3) = Eigen::Vector3d(target_acc_cog.x(), target_acc_cog.y(), target_acc_cog.z());

  double target_ang_acc_x = pid_controllers_.at(ROLL).result();
  double target_ang_acc_y = pid_controllers_.at(PITCH).result();
  double target_ang_acc_z = pid_controllers_.at(YAW).result();
  target_wrench_cog.tail(3) = Eigen::Vector3d(target_ang_acc_x, target_ang_acc_y, target_ang_acc_z);

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


  //wrench allocation matrix
  Eigen::Matrix3d inertia_inv = robot_model_->getInertia<Eigen::Matrix3d>().inverse();
  double mass_inv =  1 / robot_model_->getMass();
  Eigen::MatrixXd full_q_mat = dragon_robot_model_->getVectoringForceWrenchMatrix();
  const std::vector<int>& roll_locked_gimbal = dragon_robot_model_->getRollLockedGimbal();
  assert(full_q_mat.cols() == roll_locked_gimbal.sum);

  full_q_mat.topRows(3) =  mass_inv * full_q_mat.topRows(3) ;
  full_q_mat.bottomRows(3) =  inertia_inv * full_q_mat.bottomRows(3);
  Eigen::MatrixXd full_q_mat_inv = aerial_robot_model::pseudoinverse(full_q_mat);
  Eigen::VectorXd target_vectoring_f = full_q_mat_inv * target_wrench_cog;
  ROS_DEBUG_STREAM("target vectoring f: \n" << target_vectoring_f.transpose());

  /* pwm calculation */
  int last_col = 0;
  for(int i = 0; i < motor_num_; i++)
    {
      if(roll_locked_gimbal.at(i) == 0)
        {
          /* 2D vectoring -> 3DoF force  */
        Eigen:Vector3d f_i = target_vectoring_f.segment(last_col, 3);
          target_base_thrust_.at(i) = f_i.norm();

          /* before takeoff, the form is level -> joint_pitch = 0*/
          if(!start_rp_integration_)
            f_i.z() = dragon_robot_model_->getHoverVectoringF()[last_col + 2]; // approximation: stable state

          /* [S_theta, -S_phy * C_theta, C_phy * C_phy]^T = R.transpose * f_i_normalized */
          /* f -> gimbal angle */
          double gimbal_i_roll = atan2(-f_i.y(), f_i.z());
          double gimbal_i_pitch = atan2(f_i.x(), -f_i.y() * sin(gimbal_i_roll) + f_i.z() * cos(gimbal_i_roll));

          target_gimbal_angles_.at(2 * i) = gimbal_i_roll;
          target_gimbal_angles_.at(2 * i + 1) = gimbal_i_pitch;

          if(control_verbose_)
            std::cout << "gimbal" << i + 1 <<", r & p: " << gimbal_i_roll << ", "<< gimbal_i_pitch  << std::endl;
          last_col += 3;
        }
      else
        {
          /* 1D vectoring -> 2DoF force  */
          Eigen::VectorXd f_i = target_vectoring_f.segment(last_col, 2);
          target_base_thrust_.at(i) = f_i.norm();

          /* before takeoff, the form is level -> joint_pitch = 0*/
          if(!start_rp_integration_)
            f_i(1) = dragon_robot_model_->getHoverVectoringF()[last_col + 1]; // approximation: stable state

          double gimbal_i_pitch = atan2(f_i(0), f_i(1));

          target_gimbal_angles_.at(2 * i) = dragon_robot_model_->getGimbalNominalAngles().at(2 * i); // lock the gimbal roll
          target_gimbal_angles_.at(2 * i + 1) = gimbal_i_pitch;

          if(control_verbose_)
            std::cout << "gimbal" << i + 1 <<", lock roll, p: " << gimbal_i_pitch  << std::endl;

          last_col += 2;
        }
    }
}

void DragonFullVectoringController::sendCmd()
{
  PoseLinearController::sendCmd();

  /* send base throttle command */
  spinal::FourAxisCommand flight_command_data;
  flight_command_data.base_thrust = target_base_thrust_;
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

void DragonFullVectoringController::rosParamInit()
{
  ros::NodeHandle control_nh(nh_, "controller");
  getParam<bool>(control_nh, "decoupling", decoupling_, false);
  getParam<bool>(control_nh, "gimbal_vectoring_check_flag", gimbal_vectoring_check_flag_, false);
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::DragonFullVectoringController, aerial_robot_control::ControlBase);
