#include <ninja/control/ninja_controller.h>

using namespace std;

namespace aerial_robot_control
{
  NinjaController::NinjaController():
    BeetleController(),
    joint_control_timestamp_(-1)
  {}
  void NinjaController::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                    boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                    boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                    boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                    double ctrl_loop_rate
                                    )
  {
    BeetleController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);
    ninja_navigator_ = boost::dynamic_pointer_cast<aerial_robot_navigation::NinjaNavigator>(navigator);
    ninja_robot_model_ = boost::dynamic_pointer_cast<NinjaRobotModel>(robot_model);
    pid_controllers_.push_back(PID("joint_pitch", joint_p_gain_, joint_i_gain_, joint_d_gain_));
    pid_controllers_.push_back(PID("joint_yaw", joint_p_gain_, joint_i_gain_, joint_d_gain_));
    // ninja_robot_model_->copyTreeStructure(ninja_robot_model_->getInitModuleTree(), module_tree_for_control_);
  }
  bool NinjaController::update()
  {
    if(!ninja_navigator_->getControlFlag())
      joint_control_timestamp_ = -1;
    else if(ninja_navigator_->getControlFlag() && joint_control_timestamp_ < 0)
      joint_control_timestamp_ = ros::Time::now().toSec();
    return GimbalrotorController::update();
  }

  void NinjaController::controlCore()
  {
    if(ninja_navigator_->getCurrentAssembled() && joint_control_timestamp_ > 0 && ninja_navigator_->getFreeJointFlag())
      {
        int my_id = ninja_navigator_->getMyID();
        int leader_id = ninja_navigator_->getLeaderID();
        std::vector<int> assembled_modules_ids = ninja_navigator_->getAssemblyIds();
        double du = ros::Time::now().toSec() - joint_control_timestamp_;
        std::vector<double> joint_errs =  ninja_navigator_->getJointPosErr();
        pid_controllers_.at(JOINT_TY).updateWoVel(joint_errs.at(0),du);
        pid_controllers_.at(JOINT_TZ).updateWoVel(joint_errs.at(1),du);
        Eigen::VectorXd joint_ff_wrench = Eigen::VectorXd::Zero(6);
        joint_ff_wrench.topRows(4) = Eigen::VectorXd::Zero(4);
        joint_ff_wrench(4) = pid_controllers_.at(JOINT_TY).result();
        joint_ff_wrench(5) = pid_controllers_.at(JOINT_TZ).result();
        if(my_id < leader_id)
          {
            setFfInterWrench(my_id,-joint_ff_wrench);
          }
        else if(my_id > leader_id)
          {
            int left_module_id = assembled_modules_ids[ninja_navigator_->getMyIndex() -1];
            setFfInterWrench(left_module_id, joint_ff_wrench);
          }
      }
    joint_control_timestamp_ = ros::Time::now().toSec();
    BeetleController::controlCore();
  }

  void NinjaController::externalWrenchEstimate()
  {
    const Eigen::VectorXd target_wrench_acc_cog = getTargetWrenchAccCog();

    if(navigator_->getNaviState() != aerial_robot_navigation::HOVER_STATE &&
       navigator_->getNaviState() != aerial_robot_navigation::TAKEOFF_STATE &&
       navigator_->getNaviState() != aerial_robot_navigation:: LAND_STATE)
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
    // ROS_ERROR_STREAM(cog_rot);

    geometry_msgs::WrenchStamped wrench_msg;
    wrench_msg.header.stamp.fromSec(estimator_->getImuLatestTimeStamp());
    wrench_msg.wrench.force.x = est_external_wrench_cog(0);
    wrench_msg.wrench.force.y = est_external_wrench_cog(1);
    wrench_msg.wrench.force.z = est_external_wrench_cog(2);
    wrench_msg.wrench.torque.x = est_external_wrench_cog(3);
    wrench_msg.wrench.torque.y = est_external_wrench_cog(4);
    wrench_msg.wrench.torque.z = est_external_wrench_cog(5);
    estimate_external_wrench_pub_.publish(wrench_msg);

    //convert extimated external wrench from cog to com coordinates
    Eigen::VectorXd est_external_wrench_com = est_external_wrench_cog;
    Eigen::Matrix3d cog2com = (ninja_navigator_->getCom2Base<Eigen::Affine3d>() * ninja_robot_model_->getCog2Baselink<Eigen::Affine3d>().inverse()).rotation();
    est_external_wrench_com.head(3) = cog2com * est_external_wrench_cog.head(3);

    geometry_msgs::WrenchStamped wrench_msg_com;
    wrench_msg_com.header.stamp.fromSec(estimator_->getImuLatestTimeStamp());
    wrench_msg_com.wrench.force.x = est_external_wrench_com(0);
    wrench_msg_com.wrench.force.y = est_external_wrench_com(1);
    wrench_msg_com.wrench.force.z = est_external_wrench_com(2);
    wrench_msg_com.wrench.torque.x = est_external_wrench_com(3);
    wrench_msg_com.wrench.torque.y = est_external_wrench_com(4);
    wrench_msg_com.wrench.torque.z = est_external_wrench_com(5);
    
    beetle::TaggedWrench tagged_wrench_com;
    tagged_wrench_com.index = ninja_navigator_->getMyID();
    tagged_wrench_com.wrench = wrench_msg_com;
    tagged_external_wrench_pub_.publish(tagged_wrench_com);

    prev_est_wrench_timestamp_ = ros::Time::now().toSec();
  }

  void NinjaController::calcInteractionWrench()
  {
    /* 1. calculate external wrench W_w for whole system (e.g. ground effects, model error and etc..)*/
    Eigen::VectorXd W_w = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd W_sum = Eigen::VectorXd::Zero(6);
    int module_num = 0;
    std::map<int, bool> assembly_flag = ninja_navigator_->getAssemblyFlags();

    for(const auto & item : est_wrench_list_){
      if(assembly_flag[item.first]){
        W_sum += item.second;
        module_num ++;
      }
    }

    if(!module_num) return;
    W_w = W_sum / module_num;
    geometry_msgs::WrenchStamped wrench_msg;
    wrench_msg.header.stamp.fromSec(estimator_->getImuLatestTimeStamp());
    wrench_msg.wrench.force.x = W_w(0);
    wrench_msg.wrench.force.y = W_w(1);
    wrench_msg.wrench.force.z = W_w(2);
    wrench_msg.wrench.torque.x = W_w(3);
    wrench_msg.wrench.torque.y = W_w(4);
    wrench_msg.wrench.torque.z = W_w(5);
    whole_external_wrench_pub_.publish(wrench_msg);

    /* 2. calculate interactional wrench for each module*/
    Eigen::VectorXd left_inter_wrench = Eigen::VectorXd::Zero(6); //'left_inter_wrench' represents the wrench applied from right-side module to left-side module
    for(const auto & item : est_wrench_list_){
      if(assembly_flag[item.first]){
        Eigen::VectorXd right_inter_wrench = item.second - W_w + left_inter_wrench;
        inter_wrench_list_[item.first] = right_inter_wrench;
        left_inter_wrench = right_inter_wrench;
      }else{
        inter_wrench_list_[item.first] = Eigen::VectorXd::Zero(6);
      }
    }
    int my_id = ninja_navigator_->getMyID();
    wrench_msg.wrench.force.x = inter_wrench_list_[my_id](0);
    wrench_msg.wrench.force.y = inter_wrench_list_[my_id](1);
    wrench_msg.wrench.force.z = inter_wrench_list_[my_id](2);
    wrench_msg.wrench.torque.x = inter_wrench_list_[my_id](3);
    wrench_msg.wrench.torque.y = inter_wrench_list_[my_id](4);
    wrench_msg.wrench.torque.z = inter_wrench_list_[my_id](5);
    internal_wrench_pub_.publish(wrench_msg);
    /* 3. calculate wrench compensation term for each module*/
    int leader_id = ninja_navigator_->getLeaderID();
    /* 3.1. process of leader*/
    // wrench_comp_list_[leader_id] = -est_wrench_list_[leader_id] - W_w;
    /* 3.2. process from leader to left*/
    int right_module_id = leader_id;
    Eigen::VectorXd wrench_comp_sum_left = Eigen::VectorXd::Zero(6);
    for(int i = leader_id-1; i > 0; i--){
      if(assembly_flag[i]){
        wrench_comp_sum_left += -ff_inter_wrench_list_[i] + inter_wrench_list_[i];
        // wrench_comp_list_[i] += wrench_comp_gain_ *  wrench_comp_sum_left;
        wrench_comp_list_[i] = wrench_comp_sum_left;
        right_module_id = i;
      }else{
        wrench_comp_list_[i] = Eigen::VectorXd::Zero(6);
      }
    }
    /* 3.3. process from leader to right*/
    int max_modules_num = ninja_navigator_->getMaxModuleNum();
    int left_module_id = leader_id;
    Eigen::VectorXd wrench_comp_sum_right = Eigen::VectorXd::Zero(6);
    for(int i = leader_id+1; i <= max_modules_num; i++){
      if(assembly_flag[i]){
        wrench_comp_sum_right += ff_inter_wrench_list_[left_module_id] - inter_wrench_list_[left_module_id];
        // wrench_comp_list_[i] += wrench_comp_gain_ * wrench_comp_sum_right;
        wrench_comp_list_[i] = wrench_comp_sum_right;
        left_module_id = i;
      }else{
        wrench_comp_list_[i] = Eigen::VectorXd::Zero(6);
      }
    }
  }
  

  void NinjaController::rosParamInit()
  {
    BeetleController::rosParamInit();
    ros::NodeHandle control_nh(nh_, "controller");
    ros::NodeHandle joint_nh(control_nh, "joint_comp");
    getParam<double>(joint_nh, "p_gain", joint_p_gain_, 0.1);
    getParam<double>(joint_nh, "i_gain", joint_i_gain_, 0.005);
    getParam<double>(joint_nh, "d_gain", joint_d_gain_, 0.07);  
  }

  void NinjaController::reset()
  {
    BeetleController::reset();
    pid_controllers_.at(JOINT_TY).reset();
    pid_controllers_.at(JOINT_TZ).reset();
    for(const auto & id : ninja_navigator_->getAssemblyIds())
      {
        Eigen::VectorXd reset_ff_wrench = Eigen::VectorXd::Zero(6);
        setFfInterWrench(id,reset_ff_wrench); 
      }
    joint_control_timestamp_ = -1;
  }
} //namespace aerial_robot_controller

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::NinjaController, aerial_robot_control::ControlBase);
