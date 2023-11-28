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
    external_wrench_lower_limit_ = Eigen::VectorXd::Zero(6);
    external_wrench_upper_limit_ = Eigen::VectorXd::Zero(6);
    rosParamInit();
    if(pd_wrench_comp_mode_) ROS_ERROR("PD & Wrench comp mode");
    external_wrench_compensation_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("external_wrench_compensation", 1);
    tagged_external_wrench_pub_ = nh_.advertise<beetle::TaggedWrench>("tagged_wrench_compensation", 1);
    int max_modules_num = beetle_robot_model_->getMaxModuleNum();
    for(int i = 0; i < max_modules_num; i++){
      std::string module_name  = string("/beetle") + std::to_string(i+1);
      est_wrench_subs_.insert(make_pair(module_name, nh_.subscribe( module_name + string("/tagged_wrench_compensation"), 1, &BeetleController::estExternalWrenchCallback, this)));
      Eigen::VectorXd wrench = Eigen::VectorXd::Zero(6);
      est_wrench_list_.insert(make_pair(i, wrench));
      inter_wrench_list_.insert(make_pair(i, wrench));
      wrench_comp_list_.insert(make_pair(i, wrench));
    }
  }

  void BeetleController::controlCore()
  {
    calcInteractionWrench();

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
        ErrI_Z_ = pid_controllers_.at(Z).getErrI();
        ErrI_ROLL_ = pid_controllers_.at(ROLL).getErrI();
        ErrI_PITCH_ = pid_controllers_.at(PITCH).getErrI();
        ErrI_YAW_ = pid_controllers_.at(YAW).getErrI();
        pre_module_state_ = FOLLOWER;
      }
      //fix i_term
      pid_controllers_.at(X).setErrI(ErrI_X_);
      pid_controllers_.at(Y).setErrI(ErrI_Y_);
      pid_controllers_.at(Z).setErrI(ErrI_Z_);
      pid_controllers_.at(ROLL).setErrI(ErrI_ROLL_);
      pid_controllers_.at(PITCH).setErrI(ErrI_PITCH_);
      pid_controllers_.at(YAW).setErrI(ErrI_YAW_);
    }else{
      feedforward_wrench_acc_cog_term_ = Eigen::VectorXd::Zero(6);
      GimbalrotorController::controlCore();
    }
  }

  void BeetleController::calcInteractionWrench()
  {
    // note: Currently, this process doesn't include torque elements.

    /* 1. calculate external wrench W_w for whole system (e.g. ground effects, model error and etc..)*/
    Eigen::VectorXd W_w = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd W_sum = Eigen::VectorXd::Zero(6);
    int module_num = 0;
    std::map<int, bool> assembly_flag = beetle_robot_model_->getAssemblyFlags();
    for(const auto & item : est_wrench_list_){
      if(assembly_flag[item.first]){
        W_sum += item.second;
        module_num ++;
      }
    }
    if(!module_num) return;
    W_w = W_sum / module_num;

    /* 2. calculate interactional wrench for each module*/
    Eigen::VectorXd left_inter_wrench = Eigen::VectorXd::Zero(6);
    for(const auto & item : est_wrench_list_){
      if(assembly_flag[item.first]){
        Eigen::VectorXd right_inter_wrench = item.second - W_w + left_inter_wrench;
        inter_wrench_list_[item.first] = right_inter_wrench;
        left_inter_wrench = right_inter_wrench;
      }else{
        inter_wrench_list_[item.first] = Eigen::VectorXd::Zero(6);
      }
    }

    /* 3. calculate wrench compensation term for each module*/
    int leader_id = beetle_robot_model_->getLeaderID();
    /* 3.1. process from leader to left*/
    int right_module_id = leader_id;
    Eigen::VectorXd wrench_comp_sum_left = Eigen::VectorXd::Zero(6);
    for(int i = leader_id-1; i > 0; i--){
      if(assembly_flag[i]){
        wrench_comp_sum_left -= inter_wrench_list_[right_module_id];
        wrench_comp_list_[i] = wrench_comp_sum_left;
        right_module_id = i;
      }
    }
    /* 3.2. process from leader to right*/
    int max_modules_num = beetle_robot_model_->getMaxModuleNum();
    Eigen::VectorXd wrench_comp_sum_right = Eigen::VectorXd::Zero(6);
    for(int i = leader_id+1; i < max_modules_num; i++){
      if(assembly_flag[i]){
        wrench_comp_sum_right += inter_wrench_list_[i];
        wrench_comp_list_[i] = wrench_comp_sum_right;
      }
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

  void BeetleController::externalWrenchEstimate()
  {
    const Eigen::VectorXd target_wrench_acc_cog = getTargetWrenchAccCog();

    if(navigator_->getNaviState() != aerial_robot_navigation::HOVER_STATE &&
       navigator_->getNaviState() != aerial_robot_navigation::TAKEOFF_STATE &&
       navigator_->getNaviState() != aerial_robot_navigation::LAND_STATE)
      {
        prev_est_wrench_timestamp_ = 0;
        integrate_term_ = Eigen::VectorXd::Zero(6);
        return;
      }else if(target_wrench_acc_cog.size() == 0){
        ROS_WARN("Target wrench value for wrench estimation is not setted.");
        prev_est_wrench_timestamp_ = 0;
        integrate_term_ = Eigen::VectorXd::Zero(6);
        return;
      }

    Eigen::Vector3d vel_w, omega_cog; // workaround: use the filtered value
    auto imu_handler = boost::dynamic_pointer_cast<sensor_plugin::Imu>(estimator_->getImuHandler(0));
    tf::vectorTFToEigen(imu_handler->getFilteredVelCog(), vel_w);
    tf::vectorTFToEigen(imu_handler->getFilteredOmegaCog(), omega_cog);
    Eigen::Matrix3d cog_rot;
    tf::matrixTFToEigen(estimator_->getOrientation(Frame::COG, estimate_mode_), cog_rot);

    Eigen::Matrix3d inertia = robot_model_->getInertia<Eigen::Matrix3d>();
    double mass = robot_model_->getMass();

    Eigen::VectorXd sum_momentum = Eigen::VectorXd::Zero(6);
    sum_momentum.head(3) = mass * vel_w;
    sum_momentum.tail(3) = inertia * omega_cog;

    Eigen::VectorXd target_wrench_cog = Eigen::VectorXd::Zero(6);
    target_wrench_cog.head(3) = mass * target_wrench_acc_cog.head(3);
    target_wrench_cog.tail(3) = inertia * target_wrench_acc_cog.tail(3);

    Eigen::MatrixXd J_t = Eigen::MatrixXd::Identity(6,6);
    J_t.topLeftCorner(3,3) = cog_rot;

    Eigen::VectorXd N = mass * robot_model_->getGravity();
    N.tail(3) = aerial_robot_model::skew(omega_cog) * (inertia * omega_cog);

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

    beetle::TaggedWrench tagged_wrench;
    tagged_wrench.index = beetle_robot_model_->getMyID();
    tagged_wrench.wrench = wrench_msg;
    tagged_external_wrench_pub_.publish(tagged_wrench);

    prev_est_wrench_timestamp_ = ros::Time::now().toSec();
  }

  void BeetleController::estExternalWrenchCallback(const beetle::TaggedWrench & msg)
  {
    int id = msg.index;
    geometry_msgs::Wrench wrench_msg = msg.wrench.wrench;
    double time_stamp = msg.wrench.header.stamp.toSec();
    Eigen::VectorXd wrench = Eigen::VectorXd::Zero(6);
    wrench(0) =  wrench_msg.force.x;
    wrench(1) =  wrench_msg.force.y;
    wrench(2) =  wrench_msg.force.z;
    wrench(3) =  wrench_msg.torque.x;
    wrench(4) =  wrench_msg.torque.y;
    wrench(5) =  wrench_msg.torque.z;
    est_wrench_list_[id] = wrench;
  }

} //namespace aerial_robot_controller

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::BeetleController, aerial_robot_control::ControlBase);
