#include <hydrus/tilted_transform_control.h>

TiltedTransformController::TiltedTransformController(ros::NodeHandle nh, ros::NodeHandle nhp, std::unique_ptr<HydrusTiltedRobotModel> robot_model):
    TransformController(nh, nhp, std::move(robot_model))
{
  /* additional ros parameter */
  nhp_.param ("trans_constraint_weight", trans_constraint_weight_, 1.0);
  if(verbose_) std::cout << "trans_constraint_weight: " << std::setprecision(3) << 100 << std::endl;
  nhp_.param ("att_control_weight", att_control_weight_, 1.0);
  if(verbose_) std::cout << "att_control_weight: " << std::setprecision(3) << 100 << std::endl;

  desired_orientation_pub_ = nh_.advertise<spinal::DesireCoord>("desire_coordinate", 1);
}

bool TiltedTransformController::optimalGain()
{
  const int rotor_num = getRobotModel().getRotorNum();

  /* calculate the P_orig pseudo inverse */
  Eigen::MatrixXd P = getRobotModel().calcWrenchMatrixOnCoG();
  Eigen::MatrixXd P_dash  = getRobotModel().getInertia<Eigen::Matrix3d>().inverse() * P.bottomRows(3); // roll, pitch, yaw

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(9, 9);
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(9, rotor_num);
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

  Eigen::MatrixXd P_trans = P.topRows(3) / getRobotModel().getMass() ;
  Eigen::MatrixXd R_trans = P_trans.transpose() * P_trans;
  Eigen::MatrixXd R_input = Eigen::MatrixXd::Identity(rotor_num, rotor_num);
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
  const auto f = getRobotModel().getStaticThrust();
  double f_sum = f.sum();

  for(int i = 0; i < rotor_num; ++i)
    {
      roll_gains_.at(i).p = K_(i,0);
      roll_gains_.at(i).i = K_(i,6);
      roll_gains_.at(i).d = K_(i,1);
      pitch_gains_.at(i).p = K_(i,2);
      pitch_gains_.at(i).i = K_(i,7);
      pitch_gains_.at(i).d = K_(i,3);
      yaw_gains_.at(i).p = K_(i,4);
      yaw_gains_.at(i).i = K_(i,8);
      yaw_gains_.at(i).d = K_(i,5);

      /* special process to calculate z gain */
      z_gains_.at(i).p = q_z_ * f(i) / (f_sum / 4);
      z_gains_.at(i).i = q_z_i_ * f(i) / (f_sum / 4);
      z_gains_.at(i).d = q_z_d_ * f(i) / (f_sum / 4);
    }

  // compensation for gyro moment
  p_mat_pseudo_inv_ = aerial_robot_model::pseudoinverse(P.middleRows(2, 4));
  return true;
}

void TiltedTransformController::param2controller()
{
  TransformController::param2controller();

  double roll,pitch, yaw;
  getRobotModel().getCogDesireOrientation<KDL::Rotation>().GetRPY(roll, pitch, yaw);

  spinal::DesireCoord coord_msg;
  coord_msg.roll = roll;
  coord_msg.pitch = pitch;
  desired_orientation_pub_.publish(coord_msg);
}
