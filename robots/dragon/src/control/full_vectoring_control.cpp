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
  robot_model_for_control_ = boost::make_shared<aerial_robot_model::RobotModel>();

  /* initialize the gimbal target angles */
  target_base_thrust_.resize(motor_num_);
  target_gimbal_angles_.resize(motor_num_ * 2, 0);

  gimbal_control_pub_ = nh_.advertise<sensor_msgs::JointState>("gimbals_ctrl", 1);
  flight_cmd_pub_ = nh_.advertise<spinal::FourAxisCommand>("four_axes/command", 1);
  target_vectoring_force_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("debug/target_vectoring_force", 1);
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


  if(navigator_->getForceLandingFlag() && target_acc_w.z() < 5.0) // heuristic measures to avoid to large gimbal angles after force land
    start_rp_integration_ = false;

#if 1 // iteratively find the target force and target gimbal angles
  KDL::Rotation cog_desire_orientation = robot_model_->getCogDesireOrientation<KDL::Rotation>();
  robot_model_for_control_->setCogDesireOrientation(cog_desire_orientation); // update the cog orientation
  KDL::JntArray gimbal_processed_joint = dragon_robot_model_->getJointPositions();
  robot_model_for_control_->updateRobotModel(gimbal_processed_joint);

  const auto& roll_locked_gimbal = dragon_robot_model_->getRollLockedGimbal();
  const auto& links_rotation_from_cog = dragon_robot_model_->getLinksRotationFromCog<Eigen::Matrix3d>();
  const auto& gimbal_nominal_angles = dragon_robot_model_->getGimbalNominalAngles();
  const auto& joint_index_map = dragon_robot_model_->getJointIndexMap();
  Eigen::MatrixXd full_q_mat = dragon_robot_model_->getVectoringForceWrenchMatrix();

  double t = ros::Time::now().toSec();
  for(int j = 0; j < allocation_refine_max_iteration_; j++)
    {
      /* 5.2.1. update the wrench allocation matrix  */
      std::vector<Eigen::Vector3d> rotors_origin_from_cog = robot_model_for_control_->getRotorsOriginFromCog<Eigen::Vector3d>();

      Eigen::MatrixXd wrench_map = Eigen::MatrixXd::Zero(6, 3);
      wrench_map.block(0, 0, 3, 3) = Eigen::MatrixXd::Identity(3, 3);
      Eigen::MatrixXd mask(3,2);
      mask << 1, 0, 0, 0, 0, 1;
      int last_col = 0;
      for(int i = 0; i < motor_num_; i++)
        {
          wrench_map.block(3, 0, 3, 3) = aerial_robot_model::skew(rotors_origin_from_cog.at(i));

          if(roll_locked_gimbal.at(i) == 0)
            {
              /* 3DoF */
              full_q_mat.middleCols(last_col, 3) = wrench_map * links_rotation_from_cog.at(i);
              last_col += 3;
            }
          else
            {
              /* gimbal lock: 2Dof */
              full_q_mat.middleCols(last_col, 2) = wrench_map * links_rotation_from_cog.at(i) * aerial_robot_model::kdlToEigen(KDL::Rotation::RPY(gimbal_nominal_angles.at(i * 2), 0, 0)) * mask;
              last_col += 2;
            }
        }

      Eigen::Matrix3d inertia_inv = robot_model_->getInertia<Eigen::Matrix3d>().inverse();
      double mass_inv =  1 / robot_model_->getMass();
      full_q_mat.topRows(3) =  mass_inv * full_q_mat.topRows(3) ;
      full_q_mat.bottomRows(3) =  inertia_inv * full_q_mat.bottomRows(3);
      Eigen::MatrixXd full_q_mat_inv = aerial_robot_model::pseudoinverse(full_q_mat);
      target_vectoring_f_ = full_q_mat_inv * target_wrench_cog;

      if(control_verbose_) ROS_DEBUG_STREAM("vectoring force for control in iteration "<< j+1 << ": " << target_vectoring_f_.transpose());
      last_col = 0;
      for(int i = 0; i < motor_num_; i++)
        {
          if(roll_locked_gimbal.at(i) == 0)
            {
              Eigen::Vector3d f_i = target_vectoring_f_.segment(last_col, 3);
              target_base_thrust_.at(i) = f_i.norm();

              /* before takeoff, the form is level -> joint_pitch = 0*/
              if(!start_rp_integration_)
                f_i.z() = dragon_robot_model_->getHoverVectoringF()[last_col + 2]; // approximation: stable state

              double gimbal_i_roll = atan2(-f_i.y(), f_i.z());
              double gimbal_i_pitch = atan2(f_i.x(), -f_i.y() * sin(gimbal_i_roll) + f_i.z() * cos(gimbal_i_roll));

              target_gimbal_angles_.at(2 * i) = gimbal_i_roll;
              target_gimbal_angles_.at(2 * i + 1) = gimbal_i_pitch;

              last_col += 3;
            }
          else
            {
              Eigen::VectorXd f_i = target_vectoring_f_.segment(last_col, 2);
              target_base_thrust_.at(i) = f_i.norm();

              /* before takeoff, the form is level -> joint_pitch = 0*/
              if(!start_rp_integration_)
                f_i(1) = dragon_robot_model_->getHoverVectoringF()[last_col + 1]; // approximation: stable state

              target_gimbal_angles_.at(2 * i) = gimbal_nominal_angles.at(2 * i); // lock the gimbal roll
              target_gimbal_angles_.at(2 * i + 1) = atan2(f_i(0), f_i(1));

              last_col += 2;
            }
        }

      std::vector<Eigen::Vector3d> prev_rotors_origin_from_cog = rotors_origin_from_cog;
      for(int i = 0; i < motor_num_; ++i)
        {
          std::string s = std::to_string(i + 1);
          gimbal_processed_joint(joint_index_map.find(std::string("gimbal") + s + std::string("_roll"))->second) = target_gimbal_angles_.at(i * 2);
          gimbal_processed_joint(joint_index_map.find(std::string("gimbal") + s + std::string("_pitch"))->second) = target_gimbal_angles_.at(i * 2 + 1);
        }
      robot_model_for_control_->updateRobotModel(gimbal_processed_joint);
      rotors_origin_from_cog = robot_model_for_control_->getRotorsOriginFromCog<Eigen::Vector3d>();

      double max_diff = 1e-6;
      for(int i = 0; i < motor_num_; i++)
        {
          double diff = (rotors_origin_from_cog.at(i) - prev_rotors_origin_from_cog.at(i)).norm();
          if(diff > max_diff) max_diff = diff;
        }

      if(control_verbose_) ROS_DEBUG_STREAM("refine rotor origin in control: iteration "<< j+1 << ", max_diff: " << max_diff);

      if(max_diff < allocation_refine_threshold_)
        {
          if(control_verbose_) ROS_INFO_STREAM("refine rotor origin in control: converge in iteration " << j+1 << " max_diff " << max_diff << ", use " << ros::Time::now().toSec() - t << "sec");
          break;
        }

      if(j == allocation_refine_max_iteration_ - 1)
        {
          ROS_WARN_STREAM("refine rotor origin in control: can not converge in iteration " << j+1 << " max_diff " << max_diff);
        }
    }

#else// approximate the rotor origin with the hovering state
  //wrench allocation matrix
  Eigen::Matrix3d inertia_inv = robot_model_->getInertia<Eigen::Matrix3d>().inverse();
  double mass_inv =  1 / robot_model_->getMass();
  Eigen::MatrixXd full_q_mat = dragon_robot_model_->getVectoringForceWrenchMatrix();
  const std::vector<int>& roll_locked_gimbal = dragon_robot_model_->getRollLockedGimbal();
  assert(full_q_mat.cols() == roll_locked_gimbal.sum);

  full_q_mat.topRows(3) =  mass_inv * full_q_mat.topRows(3) ;
  full_q_mat.bottomRows(3) =  inertia_inv * full_q_mat.bottomRows(3);
  Eigen::MatrixXd full_q_mat_inv = aerial_robot_model::pseudoinverse(full_q_mat);
  target_vectoring_f_ = full_q_mat_inv * target_wrench_cog;
  ROS_DEBUG_STREAM("target vectoring f: \n" << target_vectoring_f_.transpose());

  /* conversion */
  int last_col = 0;
  for(int i = 0; i < motor_num_; i++)
    {
      if(roll_locked_gimbal.at(i) == 0)
        {
          /* 2D vectoring -> 3DoF force  */
          Eigen::Vector3d f_i = target_vectoring_f_.segment(last_col, 3);
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
          Eigen::VectorXd f_i = target_vectoring_f_.segment(last_col, 2);
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
#endif

#if 0
  // recalculate p matrix no gimbal roll lock case
  const auto& joint_index_map = dragon_robot_model_->getJointIndexMap();
  const auto& links_rotation_from_cog = dragon_robot_model_->getLinksRotationFromCog<Eigen::Matrix3d>();
  KDL::JntArray gimbal_processed_joint = dragon_robot_model_->getJointPositions();
  for(int i = 0; i < motor_num_; ++i)
    {
      std::string s = std::to_string(i + 1);
      gimbal_processed_joint(joint_index_map.find(std::string("gimbal") + s + std::string("_roll"))->second) = target_gimbal_angles_.at(i * 2);
      gimbal_processed_joint(joint_index_map.find(std::string("gimbal") + s + std::string("_pitch"))->second) = target_gimbal_angles_.at(i * 2 + 1);
    }
  robot_model_for_control_->setCogDesireOrientation(robot_model_->getCogDesireOrientation<KDL::Rotation>());
  robot_model_for_control_->updateRobotModel(gimbal_processed_joint);
  std::vector<Eigen::Vector3d> rotors_origin_from_cog_origin = robot_model_->getRotorsOriginFromCog<Eigen::Vector3d>();
  std::vector<Eigen::Vector3d> rotors_origin_from_cog = robot_model_for_control_->getRotorsOriginFromCog<Eigen::Vector3d>();
  ROS_WARN_STREAM_THROTTLE(1.0, "rotor origin diff: " <<
                           (rotors_origin_from_cog_origin.at(0) - rotors_origin_from_cog.at(0)).transpose() << "; " <<
                           (rotors_origin_from_cog_origin.at(1) - rotors_origin_from_cog.at(1)).transpose() << "; " <<
                           (rotors_origin_from_cog_origin.at(2) - rotors_origin_from_cog.at(2)).transpose() << "; " <<
                           (rotors_origin_from_cog_origin.at(3) - rotors_origin_from_cog.at(3)).transpose());
  Eigen::MatrixXd full_q_mat_dash = Eigen::MatrixXd::Zero(6, 3 * motor_num_);
  Eigen::MatrixXd wrench_map = Eigen::MatrixXd::Zero(6, 3);
  wrench_map.topRows(3) = Eigen::MatrixXd::Identity(3, 3);
  for(int i = 0; i < motor_num_; i++)
    {
      wrench_map.bottomRows(3) = aerial_robot_model::skew(rotors_origin_from_cog_origin.at(i));
      full_q_mat_dash.middleCols(3 * i, 3) = wrench_map * links_rotation_from_cog.at(i);
    }

  ROS_INFO_STREAM_THROTTLE(1.0, "full q mat diff: \n" << full_q_mat_dash - dragon_robot_model_->getVectoringForceWrenchMatrix());

  //inertia_inv = robot_model_for_control_->getInertia<Eigen::Matrix3d>().inverse();
  //mass_inv =  1 / robot_model_for_control_->getMass();
  full_q_mat_dash.topRows(3) =  mass_inv * full_q_mat_dash.topRows(3) ;
  full_q_mat_dash.bottomRows(3) =  inertia_inv * full_q_mat_dash.bottomRows(3);

  ROS_INFO_STREAM_THROTTLE(1.0, "full q mat diff2: \n" << full_q_mat_dash - full_q_mat);

  Eigen::VectorXd target_wrench_cog_dash = full_q_mat_dash * target_vectoring_f_;
  //ROS_INFO_STREAM("full p mat dash: \n" << full_q_mat_dash);
  //ROS_INFO_STREAM_THROTTLE(1.0, "target wrench cog with new p: " << target_wrench_cog_dash.transpose());
  ROS_INFO_STREAM_THROTTLE(1.0, "diff: " << (target_wrench_cog_dash - target_wrench_cog).transpose());

  // Eigen::VectorXd target_wrench_cog_tmp = Eigen::VectorXd::Zero(6);
  // ROS_INFO_STREAM("target x pertubate: " <<  * target_wrench_cog_tmp);
  ROS_INFO_STREAM_THROTTLE(1.0, "full p mat perturbate: \n " <<  full_q_mat_dash * full_q_mat_inv);

#endif
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


  std_msgs::Float32MultiArray target_vectoring_force_msg;
  for(int i = 0; i < target_vectoring_f_.size(); i++)
    target_vectoring_force_msg.data.push_back(target_vectoring_f_(i));
  target_vectoring_force_pub_.publish(target_vectoring_force_msg);
}

void DragonFullVectoringController::rosParamInit()
{
  ros::NodeHandle control_nh(nh_, "controller");
  getParam<bool>(control_nh, "decoupling", decoupling_, false);
  getParam<bool>(control_nh, "gimbal_vectoring_check_flag", gimbal_vectoring_check_flag_, false);
  getParam<double>(control_nh, "allocation_refine_threshold", allocation_refine_threshold_, 0.01);
  getParam<int>(control_nh, "allocation_refine_max_iteration", allocation_refine_max_iteration_, 1);
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::DragonFullVectoringController, aerial_robot_control::ControlBase);
