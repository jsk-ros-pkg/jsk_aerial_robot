#include <hydrus/hydrus_tilted_lqi_torsion_shift_controller.h>

using namespace aerial_robot_control;

void HydrusTiltedLQITorsionShiftController::initialize(ros::NodeHandle nh,
                                           ros::NodeHandle nhp,
                                           boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                           boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                           boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                           double ctrl_loop_rate)
{
  HydrusTiltedLQIController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);

  gain_shift_matrix_pub_stamp_ = ros::Time::now().toSec();
  K_gain_for_shift_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("K_gain_for_shift", 1);
  B_eom_kernel_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("B_eom_kernel", 1);

  kernel_mix_ratio_sub_ = nh_.subscribe<std_msgs::Float32MultiArray>("kernel_mix_ratio", 1, &HydrusTiltedLQITorsionShiftController::kernelMixRatioCallback, this);

  K_gain_for_shift_.resize(motor_num_, 3);
}

void HydrusTiltedLQITorsionShiftController::kernelMixRatioCallback(const std_msgs::Float32MultiArrayConstPtr& msg)
{
  kernel_mix_ratio_ = msg_utils::Float32MultiArray2EigenMatrix(msg);
}

bool HydrusTiltedLQITorsionShiftController::optimalGain()
{
  /* calculate the P_orig pseudo inverse */
  Eigen::MatrixXd P = robot_model_->calcWrenchMatrixOnCoG();
  Eigen::MatrixXd inertia = robot_model_->getInertia<Eigen::Matrix3d>();
  Eigen::MatrixXd P_dash  = inertia.inverse() * P.bottomRows(3); // roll, pitch, yaw

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(9, 9);
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(9, motor_num_);
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(3, 9);
  for(int i = 0; i < 3; i++)
    {
      A(2 * i, 2 * i + 1) = 1;
      B.row(2 * i + 1) = P_dash.row(i);
      C(i, 2 * i) = 1;
    }
  A.block(6, 0, 3, 9) = -C;

  ROS_DEBUG_STREAM_NAMED("LQI gain generator", "LQI gain generator: B: \n"  <<  B );

  Eigen::VectorXd q_diagonals(9);
  q_diagonals << lqi_roll_pitch_weight_(0), lqi_roll_pitch_weight_(2), lqi_roll_pitch_weight_(0), lqi_roll_pitch_weight_(2), lqi_yaw_weight_(0), lqi_yaw_weight_(2), lqi_roll_pitch_weight_(1), lqi_roll_pitch_weight_(1), lqi_yaw_weight_(1);
  Eigen::MatrixXd Q = q_diagonals.asDiagonal();

  Eigen::MatrixXd P_trans = P.topRows(3) / robot_model_->getMass() ;
  Eigen::MatrixXd R_trans = P_trans.transpose() * P_trans;
  Eigen::MatrixXd R_input = Eigen::MatrixXd::Identity(motor_num_, motor_num_);
  Eigen::MatrixXd R = R_trans * trans_constraint_weight_ + R_input * att_control_weight_;

  double t = ros::Time::now().toSec();
  bool use_kleinman_method = true;
  if(K_.cols() == 0 || K_.rows() == 0) use_kleinman_method = false;
  if(!control_utils::care(A, B, R, Q, K_, use_kleinman_method))
    {
      ROS_ERROR_STREAM("error in solver of continuous-time algebraic riccati equation");
      return false;
    }

  ROS_DEBUG_STREAM_NAMED("LQI gain generator",  "LQI gain generator: CARE: %f sec" << ros::Time::now().toSec() - t);
  ROS_DEBUG_STREAM_NAMED("LQI gain generator",  "LQI gain generator:  K \n" <<  K_);

  // Null Space Shift
  for (int i = 0; i < 3; ++i) {
    K_gain_for_shift_.col(i) = K_.col(2*i);
  }

  Eigen::FullPivLU<Eigen::MatrixXd> B_eom_lu_decomp(B);
  B_eom_kernel_ = B_eom_lu_decomp.kernel();

  Eigen::MatrixXd K_shifted = K_;

  if (kernel_mix_ratio_.rows() == 3 && kernel_mix_ratio_.cols() == B_eom_kernel_.cols()) {
    for (int i = 0; i < 3; ++i) {
      double norm_p = K_shifted.col(2*i).norm();
      double norm_i = K_shifted.col(6+i).norm();
      double norm_d = K_shifted.col(2*i+1).norm();
      for (int j = 0; j < B_eom_kernel_.cols(); ++j) {
        K_shifted.col(2*i) += B_eom_kernel_.col(j) * kernel_mix_ratio_(i,j);
        K_shifted.col(6+i) -= B_eom_kernel_.col(j) * norm_i/norm_p * kernel_mix_ratio_(i,j);
        K_shifted.col(2*i+1) += B_eom_kernel_.col(j) * norm_d/norm_p * kernel_mix_ratio_(i,j);
      }
    }
  }

  for(int i = 0; i < motor_num_; ++i)
    {
      roll_gains_.at(i) = Eigen::Vector3d(-K_shifted(i,0), K_shifted(i,6), -K_shifted(i,1));
      pitch_gains_.at(i) = Eigen::Vector3d(-K_shifted(i,2),  K_shifted(i,7), -K_shifted(i,3));
      yaw_gains_.at(i) = Eigen::Vector3d(-K_shifted(i,4), K_shifted(i,8), -K_shifted(i,5));
    }

  // compensation for gyro moment
  p_mat_pseudo_inv_ = aerial_robot_model::pseudoinverse(P.middleRows(2, 4));
  return true;
}

void HydrusTiltedLQITorsionShiftController::publishGain()
{
  HydrusTiltedLQIController::publishGain();

  if (ros::Time::now().toSec() - gain_shift_matrix_pub_stamp_ > gain_shift_matrix_pub_interval_) {
    gain_shift_matrix_pub_stamp_ = ros::Time::now().toSec();
    K_gain_for_shift_pub_.publish(msg_utils::EigenMatrix2Float32MultiArray(K_gain_for_shift_));
    B_eom_kernel_pub_.publish(msg_utils::EigenMatrix2Float32MultiArray(B_eom_kernel_));
  }
}

void HydrusTiltedLQITorsionShiftController::rosParamInit()
{
  HydrusTiltedLQIController::rosParamInit();

  ros::NodeHandle control_nh(nh_, "controller");
  ros::NodeHandle lqi_nh(control_nh, "lqi");
  control_nh.param("gain_shift_matrix_pub_interval", gain_shift_matrix_pub_interval_, 0.1);
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::HydrusTiltedLQITorsionShiftController, aerial_robot_control::ControlBase);
