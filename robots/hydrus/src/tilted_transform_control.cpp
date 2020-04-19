#include <hydrus/tilted_transform_control.h>

TiltedTransformController::TiltedTransformController(ros::NodeHandle nh, ros::NodeHandle nh_private, std::unique_ptr<HydrusTiltedRobotModel> robot_model):
    TransformController(nh, nh_private, std::move(robot_model))
{
  /* additional ros parameter */
  nh_private_.param ("trans_constraint_weight", trans_constraint_weight_, 1.0);
  if(verbose_) std::cout << "trans_constraint_weight: " << std::setprecision(3) << 100 << std::endl;
  nh_private_.param ("att_control_weight", att_control_weight_, 1.0);
  if(verbose_) std::cout << "att_control_weight: " << std::setprecision(3) << 100 << std::endl;

  desired_orientation_pub_ = nh_private_.advertise<spinal::DesireCoord>("/desire_coordinate", 1);

  /* only attitude control */
  q_diagonal_ = Eigen::VectorXd::Zero(HydrusRobotModel::LQI_THREE_AXIS_MODE * 3);
  q_diagonal_ << q_roll_,q_roll_d_,q_pitch_,q_pitch_d_,q_yaw_,q_yaw_d_, q_roll_i_,q_pitch_i_,q_yaw_i_;
}

void TiltedTransformController::cfgLQICallback(hydrus::LQIConfig &config, uint32_t level)
{
  if(config.lqi_gain_flag)
    {
      switch(level)
        {
        case LQI_RP_P_GAIN:
          ROS_INFO_STREAM_NAMED("LQI gain generator", "LQI gain generator: change the p gain weight of roll and pitch from " << q_roll_ <<  " to "  << config.q_roll);
          q_roll_ = config.q_roll;
          q_pitch_ = config.q_roll;
          break;
        case LQI_RP_I_GAIN:
          ROS_INFO_STREAM_NAMED("LQI gain generator", "LQI gain generator: change the i gain weight of roll and pitch from " << q_roll_i_ <<  " to "  << config.q_roll_i);
          q_roll_i_ = config.q_roll_i;
          q_pitch_i_ = config.q_roll_i;
          break;
        case LQI_RP_D_GAIN:
          ROS_INFO_STREAM_NAMED("LQI gain generator", "LQI gain generator: change the d gain weight of roll and pitch from " << q_roll_d_ <<  " to "  << config.q_roll_d);
          q_roll_d_ = config.q_roll_d;
          q_pitch_d_ = config.q_roll_d;
          break;
        case LQI_Y_P_GAIN:
          ROS_INFO_STREAM_NAMED("LQI gain generator", "LQI gain generator: change the p gain weight of yaw from " << q_yaw_ <<  " to "  << config.q_yaw);
          q_yaw_ = config.q_yaw;
          break;
        case LQI_Y_I_GAIN:
          ROS_INFO_STREAM_NAMED("LQI gain generator", "LQI gain generator: change the i gain weight of yaw from " << q_yaw_i_ <<  " to "  << config.q_yaw_i);
          q_yaw_i_ = config.q_yaw_i;
          break;
        case LQI_Y_D_GAIN:
          ROS_INFO_STREAM_NAMED("LQI gain generator", "LQI gain generator: change the d gain weight of yaw from " << q_yaw_d_ <<  " to "  << config.q_yaw_d);
          q_yaw_d_ = config.q_yaw_d;
          break;
        default :
          break;
        }
      q_diagonal_ << q_roll_,q_roll_d_,q_pitch_,q_pitch_d_,q_yaw_,q_yaw_d_, q_roll_i_,q_pitch_i_,q_yaw_i_;
    }
}

bool TiltedTransformController::optimalGain()
{
  /* for the R which is  diagonal matrix. should be changed to rotor_num */
  const int rotor_num = getRobotModel().getRotorNum();
  const int lqi_mode = getRobotModel().getLqiMode();

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(lqi_mode * 3, lqi_mode * 3);
  Eigen::MatrixXd B = Eigen::MatrixXd::Zero(lqi_mode * 3, rotor_num);
  Eigen::MatrixXd C = Eigen::MatrixXd::Zero(lqi_mode, lqi_mode * 3);

  for(int i = 0; i < lqi_mode; i++)
    {
      A(2 * i, 2 * i + 1) = 1;
      B.row(2 * i + 1) = getRobotModel().getP().row(i);
      C(i, 2 * i) = 1;
    }
  A.block(lqi_mode * 2, 0, lqi_mode, lqi_mode * 3) = -C;

  Eigen::MatrixXd Q = q_diagonal_.asDiagonal();
  ROS_DEBUG_STREAM_NAMED("LQI gain generator", "LQI gain generator: B: \n"  <<  B );

  Eigen::MatrixXd P_trans = getRobotModel().getP().block(3, 0, 3, rotor_num);
  Eigen::MatrixXd R_trans = P_trans.transpose() * P_trans;
  Eigen::MatrixXd R_input = Eigen::MatrixXd::Identity(rotor_num, rotor_num);
  Eigen::MatrixXd R = R_trans * trans_constraint_weight_ + R_input * att_control_weight_;

  double t = ros::Time::now().toSec();
  Eigen::MatrixXd P;
  if(!control_utils::care(A, B, R, Q, P, K_))
    {
      ROS_ERROR_STREAM("error in solver ofcontinuous-time algebraic riccati equation");
      return false;
    }

  ROS_DEBUG_STREAM_NAMED("LQI gain generator",  "LQI gain generator: CARE: %f sec" << ros::Time::now().toSec() - t);
  ROS_DEBUG_STREAM_NAMED("LQI gain generator",  "LQI gain generator:  K \n" <<  K_);

  // convert to gains
  const auto hovering_f = getRobotModel().getOptimalHoveringThrust();
  double hovering_f_sum = hovering_f.sum();

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
      z_gains_.at(i).p = q_z_ * hovering_f(i) / (hovering_f_sum / 4);
      z_gains_.at(i).i = q_z_i_ * hovering_f(i) / (hovering_f_sum / 4);
      z_gains_.at(i).d = q_z_d_ * hovering_f(i) / (hovering_f_sum / 4);
    }

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
