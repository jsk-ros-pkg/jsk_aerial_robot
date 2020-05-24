#include <hydrus/hydrus_tilted_lqi_controller.h>

using namespace control_plugin;

void HydrusTiltedLQIController::initialize(ros::NodeHandle nh,
                                           ros::NodeHandle nhp,
                                           boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                           boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                           boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                           double ctrl_loop_rate)
{
  HydrusLQIController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);

  desired_orientation_pub_ = nh_.advertise<spinal::DesireCoord>("desire_coordinate", 1);
}

void HydrusTiltedLQIController::pidUpdate()
{
  AgileFlatnessPid::pidUpdate();
}

bool HydrusTiltedLQIController::optimalGain()
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
  q_diagonals << q_roll_, q_roll_d_, q_pitch_, q_pitch_d_, q_yaw_, q_yaw_d_, q_roll_i_, q_pitch_i_, q_yaw_i_;
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
      ROS_ERROR_STREAM("error in solver ofcontinuous-time algebraic riccati equation");
      return false;
    }

  ROS_DEBUG_STREAM_NAMED("LQI gain generator",  "LQI gain generator: CARE: %f sec" << ros::Time::now().toSec() - t);
  ROS_DEBUG_STREAM_NAMED("LQI gain generator",  "LQI gain generator:  K \n" <<  K_);

  // convert to gains
  Eigen::VectorXd f = robot_model_->getStaticThrust();
  double f_sum = f.sum();

  for(int i = 0; i < motor_num_; ++i)
    {
      roll_gains_.at(i).p = K_(i,0);
      roll_gains_.at(i).i = K_(i,6);
      roll_gains_.at(i).d = K_(i,1);
      pitch_gains_.at(i).p = K_(i,2);
      pitch_gains_.at(i).i = K_(i,7);
      pitch_gains_.at(i).d = K_(i,3);
      yaw_gains_.at(i).setValue(K_(i,4), K_(i,8), K_(i,5));

      /* special process to calculate z gain */
      z_gains_.at(i).setValue(q_z_ * f(i) / (f_sum / 4),
                              q_z_i_ * f(i) / (f_sum / 4),
                              q_z_d_ * f(i) / (f_sum / 4));
    }

  // compensation for gyro moment
  p_mat_pseudo_inv_ = aerial_robot_model::pseudoinverse(P.middleRows(2, 4));
  return true;
}

void HydrusTiltedLQIController::publishGain()
{
  HydrusLQIController::publishGain();

  double roll,pitch, yaw;
  robot_model_->getCogDesireOrientation<KDL::Rotation>().GetRPY(roll, pitch, yaw);

  spinal::DesireCoord coord_msg;
  coord_msg.roll = roll;
  coord_msg.pitch = pitch;
  desired_orientation_pub_.publish(coord_msg);
}

void HydrusTiltedLQIController::rosParamInit()
{
  HydrusLQIController::rosParamInit();

  ros::NodeHandle control_nh(nh_, "controller");
  ros::NodeHandle lqi_nh(control_nh, "lqi");

  getParam<double>(lqi_nh, "trans_constraint_weight", trans_constraint_weight_, 1.0);
  getParam<double>(lqi_nh, "att_control_weight", att_control_weight_, 1.0);
}


/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(control_plugin::HydrusTiltedLQIController, control_plugin::ControlBase);
