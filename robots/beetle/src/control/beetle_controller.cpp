#include <beetle/control/beetle_controller.h>

using namespace std;

namespace aerial_robot_control
{
  BeetleController::BeetleController():
    GimbalrotorController(),
    pd_wrench_comp_mode_(false),
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
    rosParamInit();
    if(pd_wrench_comp_mode_) ROS_ERROR("PD & Wrench comp mode");
    external_wrench_compensation_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("external_wrench_compensation", 1);
    startWrenchEstimation();
  }

  void BeetleController::controlCore()
  {
    Eigen::VectorXd wrench_compensation_term = Eigen::VectorXd::Zero(6);

    //positive wrench compensation
    Eigen::VectorXd external_wrench_upper_check = (external_wrench_upper_limit_.array() < est_external_wrench_.array()).cast<double>();
    wrench_compensation_term = (est_external_wrench_ - external_wrench_upper_limit_).cwiseProduct(external_wrench_upper_check);

    //negative wrench compensation
    Eigen::VectorXd external_wrench_lower_check = (est_external_wrench_.array() < external_wrench_lower_limit_.array()).cast<double>();
    wrench_compensation_term += (est_external_wrench_ - external_wrench_lower_limit_).cwiseProduct(external_wrench_lower_check);

    double mass_inv = 1 / beetle_robot_model_->getMass();
    Eigen::Matrix3d inertia_inv = (beetle_robot_model_->getInertia<Eigen::Matrix3d>()).inverse();

    int module_state = beetle_robot_model_-> getModuleState();
    if(module_state == FOLLOWER && navigator_->getNaviState() == aerial_robot_navigation::HOVER_STATE && pd_wrench_comp_mode_){
      feedforward_wrench_acc_cog_term_.head(3) = mass_inv * wrench_compensation_term.head(3);
      feedforward_wrench_acc_cog_term_.tail(3) = -inertia_inv * wrench_compensation_term.tail(3);

      /*regarding to force, only use xy term */
      feedforward_wrench_acc_cog_term_(2) = 0;

      /*regarding to torque, only use yaw term */
      feedforward_wrench_acc_cog_term_(3) = 0;
      feedforward_wrench_acc_cog_term_(4) = 0;
      
      geometry_msgs::WrenchStamped wrench_msg;
      wrench_msg.header.stamp.fromSec(estimator_->getImuLatestTimeStamp());
      wrench_msg.wrench.force.x = feedforward_wrench_acc_cog_term_(0);
      wrench_msg.wrench.force.y = feedforward_wrench_acc_cog_term_(1);
      wrench_msg.wrench.force.z = feedforward_wrench_acc_cog_term_(2);
      wrench_msg.wrench.torque.x = feedforward_wrench_acc_cog_term_(3);
      wrench_msg.wrench.torque.y = feedforward_wrench_acc_cog_term_(4);
      wrench_msg.wrench.torque.z = feedforward_wrench_acc_cog_term_(5);
      external_wrench_compensation_pub_.publish(wrench_msg);

      GimbalrotorController::controlCore();

      //turn control mode into PD
      if(pre_module_state_ != FOLLOWER){
        ErrI_X_ = pid_controllers_.at(X).getErrI();
        ErrI_Y_ = pid_controllers_.at(Y).getErrI();
        // ErrI_Z_ = pid_controllers_.at(Z).getErrI();
        // ErrI_ROLL_ = pid_controllers_.at(ROLL).getErrI();
        // ErrI_PITCH_ = pid_controllers_.at(PITCH).getErrI();
        ErrI_YAW_ = pid_controllers_.at(YAW).getErrI();
        pre_module_state_ = FOLLOWER;
      }
      //fix i_term
      pid_controllers_.at(X).setErrI(ErrI_X_);
      pid_controllers_.at(Y).setErrI(ErrI_Y_);
      // pid_controllers_.at(Z).setErrI(ErrI_Z_);
      // pid_controllers_.at(ROLL).setErrI(ErrI_ROLL_);
      // pid_controllers_.at(PITCH).setErrI(ErrI_PITCH_);
      pid_controllers_.at(YAW).setErrI(ErrI_YAW_);

    }else{
      feedforward_wrench_acc_cog_term_ = Eigen::VectorXd::Zero(6);
      GimbalrotorController::controlCore();
    }
  }

  void BeetleController::rosParamInit()
  {
    GimbalrotorController::rosParamInit();
    ros::NodeHandle control_nh(nh_, "controller");
    getParam<bool>(control_nh, "pd_wrench_comp_mode", pd_wrench_comp_mode_, false);

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
  }

} //namespace aerial_robot_controller

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::BeetleController, aerial_robot_control::ControlBase);
