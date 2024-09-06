#include <ninja/control/ninja_controller.h>

using namespace std;

namespace aerial_robot_control
{
  NinjaController::NinjaController():
    BeetleController()
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

} //namespace aerial_robot_controller

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::NinjaController, aerial_robot_control::ControlBase);
