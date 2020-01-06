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
      printf("LQI Param:");
      switch(level)
        {
        case LQI_RP_P_GAIN:
          q_roll_ = config.q_roll;
          q_pitch_ = config.q_roll;
          printf("change the gain of lqi roll and pitch p gain: %f\n", q_roll_);
          break;
        case LQI_RP_I_GAIN:
          q_roll_i_ = config.q_roll_i;
          q_pitch_i_ = config.q_roll_i;
          printf("change the gain of lqi roll and pitch i gain: %f\n", q_roll_i_);
          break;
        case LQI_RP_D_GAIN:
          q_roll_d_ = config.q_roll_d;
          q_pitch_d_ = config.q_roll_d;
          printf("change the gain of lqi roll and pitch d gain:%f\n", q_roll_d_);
          break;
        case LQI_Y_P_GAIN:
          q_yaw_ = config.q_yaw;
          printf("change the gain of lqi yaw p gain:%f\n", q_yaw_);
          break;
        case LQI_Y_I_GAIN:
          q_yaw_i_ = config.q_yaw_i;
          printf("change the gain of lqi yaw i gain:%f\n", q_yaw_i_);
          break;
        case LQI_Y_D_GAIN:
          q_yaw_d_ = config.q_yaw_d;
          printf("change the gain of lqi yaw d gain:%f\n", q_yaw_d_);
          break;
        case LQI_Z_P_GAIN:
          q_z_ = config.q_z;
          printf("change the gain of lqi z p gain:%f\n", q_z_);
          break;
        case LQI_Z_I_GAIN:
          q_z_i_ = config.q_z_i;
          printf("change the gain of lqi z i gain:%f\n", q_z_i_);
          break;
        case LQI_Z_D_GAIN:
          q_z_d_ = config.q_z_d;
          printf("change the gain of lqi z d gain:%f\n", q_z_d_);
          break;
        default :
          printf("\n");
          break;
        }
      q_diagonal_ << q_roll_,q_roll_d_,q_pitch_,q_pitch_d_,q_yaw_,q_yaw_d_, q_roll_i_,q_pitch_i_,q_yaw_i_;
    }
}

bool TiltedTransformController::hamiltonMatrixSolver()
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
  if(control_verbose_) std::cout << "B:"  << std::endl << B << std::endl;

  Eigen::MatrixXd P_trans = getRobotModel().getP().block(3, 0, 3, rotor_num);
  Eigen::MatrixXd R_trans = P_trans.transpose() * P_trans;
  Eigen::MatrixXd R_input = Eigen::MatrixXd::Identity(rotor_num, rotor_num);
  Eigen::MatrixXd R_inv = (R_trans * trans_constraint_weight_ + R_input * att_control_weight_).inverse();

  Eigen::MatrixXcd H = Eigen::MatrixXcd::Zero(lqi_mode * 6, lqi_mode * 6);
  H.block(0,0, lqi_mode * 3, lqi_mode * 3) = A.cast<std::complex<double> >();
  H.block(lqi_mode * 3, 0, lqi_mode * 3, lqi_mode * 3) = -(Q.cast<std::complex<double> >());
  H.block(0, lqi_mode * 3, lqi_mode * 3, lqi_mode * 3) = - (B * R_inv * B.transpose()).cast<std::complex<double> >();
  H.block(lqi_mode * 3, lqi_mode * 3, lqi_mode * 3, lqi_mode * 3) = - (A.transpose()).cast<std::complex<double> >();


  if(debug_verbose_) ROS_INFO("  start H eigen compute");
  //eigen solving
  ros::Time start_time = ros::Time::now();
  Eigen::ComplexEigenSolver<Eigen::MatrixXcd> ces;
  ces.compute(H);
  if(debug_verbose_) ROS_INFO("  finish H eigen compute");

  if(control_verbose_)
    ROS_INFO("h eigen time is: %f\n", ros::Time::now().toSec() - start_time.toSec());

  Eigen::MatrixXcd phy = Eigen::MatrixXcd::Zero(lqi_mode * 6, lqi_mode * 3);
  int j = 0;

  for(int i = 0; i < lqi_mode * 6; i++)
    {
      if(ces.eigenvalues()[i].real() < 0)
        {
          if(j >= lqi_mode * 3)
            {
              ROS_ERROR("nagativa sigular amount is larger");
              return false;
            }

          phy.col(j) = ces.eigenvectors().col(i);
          j++;
        }
    }

  if(j != lqi_mode * 3)
    {
      ROS_ERROR("nagativa sigular value amount is not enough");
      return false;
    }

  Eigen::MatrixXcd f = phy.block(0, 0, lqi_mode * 3, lqi_mode * 3);
  Eigen::MatrixXcd g = phy.block(lqi_mode * 3, 0, lqi_mode * 3, lqi_mode * 3);

  if(debug_verbose_) ROS_INFO("  start calculate f inv");
  start_time = ros::Time::now();
  Eigen::MatrixXcd f_inv  = f.inverse();
  if(control_verbose_)
    ROS_INFO("f inverse: %f\n", ros::Time::now().toSec() - start_time.toSec());

  if(debug_verbose_) ROS_INFO("  finish calculate f inv");

  Eigen::MatrixXcd P = g * f_inv;

  //K
  K_ = -R_inv * B.transpose() * P.real();

  if(control_verbose_)
    std::cout << "K is:" << std::endl << K_ << std::endl;

  if(a_dash_eigen_calc_flag_)
    {
      if(debug_verbose_) ROS_INFO("  start A eigen compute");
      //check the eigen of new A
      Eigen::MatrixXd A_dash = Eigen::MatrixXd::Zero(lqi_mode * 3, lqi_mode * 3);
      A_dash = A + B * K_;
      Eigen::EigenSolver<Eigen::MatrixXd> esa(A_dash);
      if(debug_verbose_) ROS_INFO("  finish A eigen compute");
      if(control_verbose_)
        std::cout << "The eigenvalues of A_hash are:" << std::endl << esa.eigenvalues() << std::endl;
    }

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
