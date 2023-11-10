#include <beetle/control/beetle_controller.h>

using namespace std;

namespace aerial_robot_control
{
  BeetleController::BeetleController():
    GimbalrotorController(),
    lpf_init_flag_(false),
    pre_module_state_(SEPARATED)
  {
  }

  void BeetleController::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                         boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                         boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                         boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                         double ctrl_loop_rate
                                         )
  {
    GimbalrotorController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);
    beetle_robot_model_ = boost::dynamic_pointer_cast<BeetleRobotModel>(robot_model);
    filterd_est_external_wrench_ = Eigen::VectorXd::Zero(6);
    external_wrench_lower_limit_ = Eigen::VectorXd::Zero(6);
    external_wrench_upper_limit_ = Eigen::VectorXd::Zero(6);
    rosParamInit();
    lpf_est_external_wrench_ = IirFilter(sample_freq_, cutoff_freq_, 6);
    estimate_external_wrench_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("estimated_external_wrench", 1);
    external_wrench_compensation_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("external_wrench_compensation", 1);
    est_external_wrench_ = Eigen::VectorXd::Zero(6);
    init_sum_momentum_ = Eigen::VectorXd::Zero(6);
    integrate_term_ = Eigen::VectorXd::Zero(6);
    
    prev_est_wrench_timestamp_ = 0;
    wrench_estimate_thread_ = boost::thread([this]()
                                            {
                                              ros::NodeHandle control_nh(nh_, "controller");
                                              double update_rate;
                                              control_nh.param ("wrench_estimate_update_rate", update_rate, 100.0);

                                              ros::Rate loop_rate(update_rate);
                                              while(ros::ok())
                                                {
                                                  BeetleController::externalWrenchEstimate();
                                                  loop_rate.sleep();
                                                }
                                            });
  }

  void BeetleController::controlCore()
  {
    Eigen::VectorXd wrench_compensation_term = Eigen::VectorXd::Zero(6);

    //positive wrench compensation
    Eigen::VectorXd external_wrench_upper_check = (external_wrench_upper_limit_.array() < filterd_est_external_wrench_.array()).cast<double>();
    wrench_compensation_term = (filterd_est_external_wrench_ - external_wrench_upper_limit_).cwiseProduct(external_wrench_upper_check);

    //negative wrench compensation
    Eigen::VectorXd external_wrench_lower_check = (filterd_est_external_wrench_.array() < external_wrench_lower_limit_.array()).cast<double>();
    wrench_compensation_term += (filterd_est_external_wrench_ - external_wrench_lower_limit_).cwiseProduct(external_wrench_lower_check);

    double mass_inv = 1 / beetle_robot_model_->getMass();
    Eigen::Matrix3d inertia_inv = (beetle_robot_model_->getInertia<Eigen::Matrix3d>()).inverse();

    int module_state = beetle_robot_model_-> getModuleState();
    if(module_state == FOLLOWER){
      feedforward_wrench_acc_cog_term_.head(3) = mass_inv * wrench_compensation_term.head(3);
      feedforward_wrench_acc_cog_term_.tail(3) = -inertia_inv * wrench_compensation_term.tail(3);
      geometry_msgs::WrenchStamped wrench_msg;
      wrench_msg.header.stamp.fromSec(estimator_->getImuLatestTimeStamp());
      wrench_msg.wrench.force.x = feedforward_wrench_acc_cog_term_(0);
      wrench_msg.wrench.force.y = feedforward_wrench_acc_cog_term_(1);
      wrench_msg.wrench.force.z = feedforward_wrench_acc_cog_term_(2);
      wrench_msg.wrench.torque.x = feedforward_wrench_acc_cog_term_(3);
      wrench_msg.wrench.torque.y = feedforward_wrench_acc_cog_term_(4);
      wrench_msg.wrench.torque.z = feedforward_wrench_acc_cog_term_(5);
      // wrench_msg.wrench.force.x = filterd_est_external_wrench_(0);
      // wrench_msg.wrench.force.y = filterd_est_external_wrench_(1);
      // wrench_msg.wrench.force.z = filterd_est_external_wrench_(2);
      // wrench_msg.wrench.torque.x = filterd_est_external_wrench_(3);
      // wrench_msg.wrench.torque.y = filterd_est_external_wrench_(4);
      // wrench_msg.wrench.torque.z = filterd_est_external_wrench_(5);
      external_wrench_compensation_pub_.publish(wrench_msg);

      GimbalrotorController::controlCore();

      //turn control mode into PD
      if(pre_module_state_ != FOLLOWER){
        ErrI_X_ = pid_controllers_.at(X).getErrI();
        ErrI_Y_ = pid_controllers_.at(Y).getErrI();
        ErrI_Z_ = pid_controllers_.at(Z).getErrI();
        ErrI_Yaw_ = pid_controllers_.at(YAW).getErrI();
        pre_module_state_ = FOLLOWER;
      }
      if(navigator_->getNaviState() == aerial_robot_navigation::HOVER_STATE){
        //fix i_term
        pid_controllers_.at(X).setErrI(ErrI_X_); 
        pid_controllers_.at(Y).setErrI(ErrI_Y_);
        pid_controllers_.at(Z).setErrI(ErrI_Z_);
        pid_controllers_.at(YAW).setErrI(ErrI_Yaw_);
      }
    }else{
      GimbalrotorController::controlCore();
      pre_module_state_ = module_state;
    }
  }

  void BeetleController::rosParamInit()
  {
    GimbalrotorController::rosParamInit();
    ros::NodeHandle control_nh(nh_, "controller");
    momentum_observer_matrix_ = Eigen::MatrixXd::Identity(6,6);
    double force_weight, torque_weight;
    getParam<double>(control_nh, "momentum_observer_force_weight", force_weight, 10.0);
    getParam<double>(control_nh, "momentum_observer_torque_weight", torque_weight, 10.0);
    momentum_observer_matrix_.topRows(3) *= force_weight;
    momentum_observer_matrix_.bottomRows(3) *= torque_weight;

    double external_force_upper_limit, external_force_lower_limit, external_torque_upper_limit, external_torque_lower_limit;
    getParam<double>(control_nh, "external_force_upper_limit", external_force_upper_limit, 0.5);
    getParam<double>(control_nh, "external_force_lower_limit", external_force_lower_limit, -0.5);
    getParam<double>(control_nh, "external_torque_upper_limit", external_torque_upper_limit, 0.01);
    getParam<double>(control_nh, "external_torque_lower_limit", external_torque_lower_limit, -0.01);
    external_wrench_upper_limit_.head(3) = Eigen::Vector3d::Constant(external_force_upper_limit);
    external_wrench_upper_limit_.tail(3) = Eigen::Vector3d::Constant(external_torque_upper_limit);
    external_wrench_lower_limit_.head(3) = Eigen::Vector3d::Constant(external_force_lower_limit);
    external_wrench_lower_limit_.tail(3) = Eigen::Vector3d::Constant(external_torque_lower_limit);
    ROS_INFO_STREAM("upper limit of external wrench : "<<external_wrench_upper_limit_.transpose());
    ROS_INFO_STREAM("lower limit of external wrench : "<<external_wrench_lower_limit_.transpose());

    getParam<double>(control_nh, "cutoff_freq", cutoff_freq_, 10.0);
    getParam<double>(control_nh, "sample_freq", sample_freq_, 40.0);
  }

  //copy from dragon's full vectoring control
  void BeetleController::externalWrenchEstimate()
  {
    if(navigator_->getNaviState() != aerial_robot_navigation::HOVER_STATE &&
       navigator_->getNaviState() != aerial_robot_navigation::LAND_STATE)
      {
        prev_est_wrench_timestamp_ = 0;
        integrate_term_ = Eigen::VectorXd::Zero(6);
        return;
      }

    Eigen::Vector3d vel_w, omega_cog; // workaround: use the filtered value
    auto imu_handler = boost::dynamic_pointer_cast<sensor_plugin::BeetleImu>(estimator_->getImuHandler(0));
    tf::vectorTFToEigen(imu_handler->getFilteredVelCog(), vel_w);
    tf::vectorTFToEigen(imu_handler->getFilteredOmegaCog(), omega_cog);
    Eigen::Matrix3d cog_rot;
    tf::matrixTFToEigen(estimator_->getOrientation(Frame::COG, estimate_mode_), cog_rot);
    Eigen::Matrix3d inertia = beetle_robot_model_->getInertia<Eigen::Matrix3d>();

    double mass = beetle_robot_model_->getMass();
    Eigen::VectorXd sum_momentum = Eigen::VectorXd::Zero(6);
    sum_momentum.head(3) = mass * vel_w;
    sum_momentum.tail(3) = inertia * omega_cog;
    Eigen::MatrixXd J_t = Eigen::MatrixXd::Identity(6,6);
    J_t.topLeftCorner(3,3) = cog_rot;
    Eigen::VectorXd N = mass * beetle_robot_model_->getGravity();
    N.tail(3) = aerial_robot_model::skew(omega_cog) * (inertia * omega_cog);

    const Eigen::VectorXd target_wrench_acc_cog = getTargetWrenchAccCog();
    Eigen::VectorXd target_wrench_cog = Eigen::VectorXd::Zero(6);
    target_wrench_cog.head(3) = mass * target_wrench_acc_cog.head(3);
    target_wrench_cog.tail(3) = inertia * target_wrench_acc_cog.tail(3);
    if(prev_est_wrench_timestamp_ == 0)
      {
        prev_est_wrench_timestamp_ = ros::Time::now().toSec();
        init_sum_momentum_ = sum_momentum; // not good
      }

    double dt = ros::Time::now().toSec() - prev_est_wrench_timestamp_;
    integrate_term_ += (J_t * target_wrench_cog - N + est_external_wrench_) * dt;
    est_external_wrench_ = momentum_observer_matrix_ * (sum_momentum - init_sum_momentum_ - integrate_term_);
    Eigen::VectorXd est_external_wrench_cog = est_external_wrench_;
    est_external_wrench_cog.head(3) = cog_rot.inverse() * est_external_wrench_.head(3);

    geometry_msgs::WrenchStamped wrench_msg;
    wrench_msg.header.stamp.fromSec(estimator_->getImuLatestTimeStamp());
    wrench_msg.wrench.force.x = est_external_wrench_(0);
    wrench_msg.wrench.force.y = est_external_wrench_(1);
    wrench_msg.wrench.force.z = est_external_wrench_(2);
    wrench_msg.wrench.torque.x = est_external_wrench_(3);
    wrench_msg.wrench.torque.y = est_external_wrench_(4);
    wrench_msg.wrench.torque.z = est_external_wrench_(5);
    estimate_external_wrench_pub_.publish(wrench_msg);

    prev_est_wrench_timestamp_ = ros::Time::now().toSec();

    if(!lpf_init_flag_){
      lpf_est_external_wrench_.setInitValues(est_external_wrench_);
      lpf_init_flag_ = true;
    }
    filterd_est_external_wrench_ = lpf_est_external_wrench_.filterFunction(est_external_wrench_);
  }
} //namespace aerial_robot_controller

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::BeetleController, aerial_robot_control::ControlBase);
