#include <gimbalrotor/control/gimbalrotor_controller.h>

using namespace std;

namespace aerial_robot_control
{
  GimbalrotorController::GimbalrotorController():
    PoseLinearController()
  {
  }

  void GimbalrotorController::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                         boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                         boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                         boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                         double ctrl_loop_rate
                                         )
  {
    PoseLinearController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);
    gimbalrotor_robot_model_ = boost::dynamic_pointer_cast<GimbalrotorRobotModel>(robot_model);

    GimbalrotorController::rosParamInit();

    rotor_coef_ = gimbal_dof_ + 1; //number of virtual rotors in each rotor arm

    target_base_thrust_.resize(motor_num_ * rotor_coef_);
    target_full_thrust_.resize(motor_num_);
    target_gimbal_angles_.resize(motor_num_ * gimbal_dof_, 0);

    flight_cmd_pub_ = nh_.advertise<spinal::FourAxisCommand>("four_axes/command", 1);
    gimbal_control_pub_ = nh_.advertise<sensor_msgs::JointState>("gimbals_ctrl", 1);
    gimbal_state_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);
    target_vectoring_force_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("debug/target_vectoring_force", 1);
    rpy_gain_pub_ = nh_.advertise<spinal::RollPitchYawTerms>("rpy/gain", 1);
    torque_allocation_matrix_inv_pub_ = nh_.advertise<spinal::TorqueAllocationMatrixInv>("torque_allocation_matrix_inv", 1);
    gimbal_dof_pub_ = nh_.advertise<std_msgs::UInt8>("gimbal_dof", 1);
  }

  void GimbalrotorController::reset()
  {
    PoseLinearController::reset();

    setAttitudeGains();
  }

  void GimbalrotorController::rosParamInit()
  {
    ros::NodeHandle control_nh(nh_, "controller");
    getParam<int>(control_nh, "gimbal_dof", gimbal_dof_, 1);
    getParam<bool>(control_nh, "gimbal_calc_in_fc", gimbal_calc_in_fc_, true);
  }

  bool GimbalrotorController::update()
  {
    sendGimbalCommand();
    if(gimbal_calc_in_fc_){
      std_msgs::UInt8 msg;
      msg.data = gimbal_dof_;
      gimbal_dof_pub_.publish(msg);
    }

    return PoseLinearController::update();
  }

  void GimbalrotorController::controlCore()
  {
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
    Eigen::Matrix3d inertia = gimbalrotor_robot_model_->getInertia<Eigen::Matrix3d>();
    Eigen::Vector3d omega;
    tf::vectorTFToEigen(omega_, omega);
    Eigen::Vector3d gyro = omega.cross(inertia * omega);

    if(gimbal_calc_in_fc_)
      target_wrench_acc_cog.tail(3) = Eigen::Vector3d(target_ang_acc_x, target_ang_acc_y, target_ang_acc_z);
    else
      target_wrench_acc_cog.tail(3) = Eigen::Vector3d(target_ang_acc_x, target_ang_acc_y, target_ang_acc_z) + gyro;
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

    Eigen::MatrixXd full_q_mat = Eigen::MatrixXd::Zero(6, 3 * motor_num_);

    double mass_inv = 1 / gimbalrotor_robot_model_->getMass();

    Eigen::Matrix3d inertia_inv = inertia.inverse();

    std::vector<Eigen::Vector3d> rotors_origin_from_cog = gimbalrotor_robot_model_->getRotorsOriginFromCog<Eigen::Vector3d>();
    const auto& rotor_direction = gimbalrotor_robot_model_->getRotorDirection();
    const double m_f_rate = gimbalrotor_robot_model_->getMFRate();

    Eigen::MatrixXd wrench_map = Eigen::MatrixXd::Zero(6, 3);
    wrench_map.block(0, 0, 3, 3) =  Eigen::MatrixXd::Identity(3, 3);
    int last_col = 0;

    /* calculate normal allocation */
    for(int i = 0; i < motor_num_; i++){
      wrench_map.block(3, 0, 3, 3) = aerial_robot_model::skew(rotors_origin_from_cog.at(i))  + m_f_rate * rotor_direction.at(i + 1) * Eigen::Matrix3d::Identity();
      full_q_mat.middleCols(last_col, 3) = wrench_map;
      last_col += 3;
    }

    full_q_mat.topRows(3) = mass_inv * full_q_mat.topRows(3);
    full_q_mat.bottomRows(3) = inertia_inv * full_q_mat.bottomRows(3);

    /* calculate masked rotation matrix */
    std::vector<KDL::Rotation> thrust_coords_rot = gimbalrotor_robot_model_->getThrustCoordRot<KDL::Rotation>();
    std::vector<Eigen::MatrixXd> masked_rot;
    for(int i = 0; i < motor_num_; i++){
      tf::Quaternion r;  tf::quaternionKDLToTF(thrust_coords_rot.at(i), r);
      Eigen::Matrix3d conv_cog_from_thrust; tf::matrixTFToEigen(tf::Matrix3x3(r),conv_cog_from_thrust);
      if(gimbal_dof_ == 1)
        {
          Eigen::MatrixXd mask(3, 2);
          mask << 0, 0, 1, 0, 0, 1;
          masked_rot.push_back(conv_cog_from_thrust * mask);
        }
      else if(gimbal_dof_ == 2)
        {
          Eigen::MatrixXd mask = Eigen::Matrix3d::Identity();
          masked_rot.push_back(conv_cog_from_thrust * mask);
        }
    }

    /* mask integrated allocation */
    Eigen::MatrixXd integrated_rot = Eigen::MatrixXd::Zero(3 * motor_num_, rotor_coef_ * motor_num_);
    Eigen::MatrixXd integrated_map = Eigen::MatrixXd::Zero(6, (gimbal_dof_ + 1) * motor_num_);
    for(int i = 0; i< motor_num_; i++){
      integrated_rot.block(3*i, rotor_coef_*i, 3, rotor_coef_) = masked_rot[i];
    }
    integrated_map = full_q_mat * integrated_rot;

    Eigen::MatrixXd integrated_map_inv = aerial_robot_model::pseudoinverse(integrated_map);
    integrated_map_inv_trans_ = integrated_map_inv.leftCols(3);
    integrated_map_inv_rot_ = integrated_map_inv.rightCols(3);
    target_vectoring_f_trans_ = integrated_map_inv_trans_ * target_wrench_acc_cog.topRows(3);
    target_vectoring_f_rot_ = integrated_map_inv_rot_ * target_wrench_acc_cog.bottomRows(3); //debug
    last_col = 0;

    /*  calculate target base thrust (considering only translational components)*/
    double max_yaw_scale = 0; // for reconstruct yaw control term in spinal
    for(int i = 0; i < motor_num_; i++){
      Eigen::VectorXd f_i = target_vectoring_f_trans_.segment(last_col, rotor_coef_);
      if(gimbal_dof_ == 1)
        {
          target_base_thrust_.at(rotor_coef_ * i) = f_i[0];
          target_base_thrust_.at(rotor_coef_ * i+1) = f_i[1];
        }else if(gimbal_dof_ == 2){
          target_base_thrust_.at(rotor_coef_ * i) = f_i[0];
          target_base_thrust_.at(rotor_coef_ * i+1) = f_i[1];
          target_base_thrust_.at(rotor_coef_ * i+2) = f_i[2];
        }
      if(integrated_map_inv(i, YAW) > max_yaw_scale) max_yaw_scale = integrated_map_inv(i, YAW);
      last_col += rotor_coef_;
    }
    candidate_yaw_term_ = pid_controllers_.at(YAW).result() * max_yaw_scale;

    /* calculate target full thrusts and gimbal angles (considering full components)*/
    last_col = 0;
    for(int i = 0; i < motor_num_; i++){
      Eigen::VectorXd f_i_integrated = target_vectoring_f_rot_.segment(last_col, rotor_coef_) + target_vectoring_f_trans_.segment(last_col, rotor_coef_);
      target_full_thrust_.at(i) = f_i_integrated.norm();
      if(gimbal_dof_ == 1)
        {
          target_gimbal_angles_.at(i) = atan2(-f_i_integrated[0], f_i_integrated[1]);
        }
      else if(gimbal_dof_ == 2)
        {
          if(f_i_integrated[0] == 0 || f_i_integrated[2] == 0) continue;

          double gimbal_roll = atan2(-f_i_integrated[1], f_i_integrated[2]);
          double gimbal_pitch = atan2(f_i_integrated[0], -f_i_integrated[1] * sin(gimbal_roll) + f_i_integrated[2]* cos(gimbal_roll));
          target_gimbal_angles_.at(2 * i) = gimbal_roll;
          target_gimbal_angles_.at(2 * i + 1) = gimbal_pitch;
        }
      last_col += rotor_coef_;
    }
  }

  void GimbalrotorController::sendCmd()
  {
    PoseLinearController::sendCmd();

    sendFourAxisCommand();

    if(gimbal_calc_in_fc_){
      sendTorqueAllocationMatrixInv();
    }
    else
      {
        sensor_msgs::JointState gimbal_control_msg;
        gimbal_control_msg.header.stamp = ros::Time::now();
        for(int i = 0; i < motor_num_; i++){
          if(gimbal_dof_ == 1)
            {
              gimbal_control_msg.position.push_back(target_gimbal_angles_.at(i));
            }
          else if(gimbal_dof_ == 2)
            {
              gimbal_control_msg.position.push_back(target_gimbal_angles_.at(2*i));
              gimbal_control_msg.position.push_back(target_gimbal_angles_.at(2*i + 1));
            }
        }
        gimbal_control_pub_.publish(gimbal_control_msg);
        
        std_msgs::Float32MultiArray target_vectoring_force_msg;
        target_vectoring_f_ = target_vectoring_f_trans_ + target_vectoring_f_rot_;
        for(int i = 0; i < target_vectoring_f_.size(); i++){
          target_vectoring_force_msg.data.push_back(target_vectoring_f_(i));
        }
        target_vectoring_force_pub_.publish(target_vectoring_force_msg);
        
      }
  }

  void GimbalrotorController::sendFourAxisCommand()
  {
    spinal::FourAxisCommand flight_command_data;

    if(gimbal_calc_in_fc_){
      flight_command_data.base_thrust = target_base_thrust_;
      flight_command_data.angles[2] = candidate_yaw_term_;
    }
    else
      {
        flight_command_data.base_thrust = target_full_thrust_;
      }

    flight_cmd_pub_.publish(flight_command_data);
  }

  void GimbalrotorController::sendGimbalCommand()
  {
    sensor_msgs::JointState gimbal_state_msg;
    gimbal_state_msg.header.stamp = ros::Time::now();
    for(int i = 0; i < motor_num_; i++){
      if(gimbal_dof_ == 1)
        {
          gimbal_state_msg.position.push_back(target_gimbal_angles_.at(i));
          std::string gimbal_name = "gimbal" + std::to_string(i+1);
          gimbal_state_msg.name.push_back(gimbal_name);
        }
      else if(gimbal_dof_ == 2)
      {
        gimbal_state_msg.position.push_back(target_gimbal_angles_.at(2*i));
        gimbal_state_msg.position.push_back(target_gimbal_angles_.at(2*i+1));
        std::string gimbal_roll_name = "gimbal" + std::to_string(i+1) + "_roll";
        std::string gimbal_pitch_name = "gimbal" + std::to_string(i+1) + "_pitch";
        gimbal_state_msg.name.push_back(gimbal_roll_name);
        gimbal_state_msg.name.push_back(gimbal_pitch_name);
      }
    }
    // gimbal_state_pub_.publish(gimbal_state_msg);

  }

  void GimbalrotorController::sendTorqueAllocationMatrixInv()
  {
    spinal::TorqueAllocationMatrixInv torque_allocation_matrix_inv_msg;
    torque_allocation_matrix_inv_msg.rows.resize(motor_num_ * rotor_coef_);
    Eigen::MatrixXd torque_allocation_matrix_inv = integrated_map_inv_rot_;
    if (torque_allocation_matrix_inv.cwiseAbs().maxCoeff() > INT16_MAX * 0.001f)
      ROS_ERROR("Torque Allocation Matrix overflow");
    for (unsigned int i = 0; i < motor_num_* rotor_coef_; i++)
      {
        torque_allocation_matrix_inv_msg.rows.at(i).x = torque_allocation_matrix_inv(i,0) * 1000;
        torque_allocation_matrix_inv_msg.rows.at(i).y = torque_allocation_matrix_inv(i,1) * 1000;
        torque_allocation_matrix_inv_msg.rows.at(i).z = torque_allocation_matrix_inv(i,2) * 1000;
      }
    torque_allocation_matrix_inv_pub_.publish(torque_allocation_matrix_inv_msg);
  }

  void GimbalrotorController::setAttitudeGains()
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
} //namespace aerial_robot_controller



/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::GimbalrotorController, aerial_robot_control::ControlBase);
