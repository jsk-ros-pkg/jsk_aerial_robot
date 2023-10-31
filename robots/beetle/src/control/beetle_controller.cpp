#include <beetle/control/beetle_controller.h>

using namespace std;

namespace aerial_robot_control
{
  BeetleController::BeetleController():
    GimbalrotorController()
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
    rosParamInit();
    estimate_external_wrench_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("estimated_external_wrench", 1);
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
    Eigen::Matrix3d inertia = robot_model_->getInertia<Eigen::Matrix3d>();

    double mass = robot_model_->getMass();
    Eigen::VectorXd sum_momentum = Eigen::VectorXd::Zero(6);
    sum_momentum.head(3) = mass * vel_w;
    sum_momentum.tail(3) = inertia * omega_cog;
    Eigen::MatrixXd J_t = Eigen::MatrixXd::Identity(6,6);
    J_t.topLeftCorner(3,3) = cog_rot;
    Eigen::VectorXd N = mass * robot_model_->getGravity();
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
  }
} //namespace aerial_robot_controller

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::BeetleController, aerial_robot_control::ControlBase);
