#include <hydrus/hydrus_tilted_lqi_adaptive_controller.h>

using namespace aerial_robot_control;

void HydrusTiltedLQIAdaptiveController::initialize(ros::NodeHandle nh,
                                           ros::NodeHandle nhp,
                                           boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                           boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                           boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                           double ctrl_loop_rate)
{
  HydrusTiltedLQIController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);

  desire_yaw_pub_ = nh_.advertise<spinal::DesireYaw>("desire_yaw", 1);

  flight_state_ = ARM_OFF_STATE;
  prev_flight_state_ = ARM_OFF_STATE;

  // send spinal MRAC parameters
  ros::service::waitForService("set_mrac_params");
  ros::service::waitForService("spinal_mrac_trigger");
  ros::ServiceClient set_mrac_params_client = nh_.serviceClient<spinal::SetMRACParams>("set_mrac_params");
  spinal::SetMRACParams set_mrac_params_srv;
  ros::NodeHandle control_nh(nh_, "controller");
  ros::NodeHandle mrac_nh(nh_, "mrac");
  int log_rate;
  mrac_nh.getParam("log_rate", log_rate);
  set_mrac_params_srv.request.log_rate = log_rate;
  mrac_nh.getParam("mrac_ratio", set_mrac_params_srv.request.mrac_ratio);
  mrac_nh.getParam("k1m", set_mrac_params_srv.request.k1m);
  mrac_nh.getParam("k2m", set_mrac_params_srv.request.k2m);
  mrac_nh.getParam("K1", set_mrac_params_srv.request.K1);
  mrac_nh.getParam("K2", set_mrac_params_srv.request.K2);
  mrac_nh.getParam("kappa", set_mrac_params_srv.request.kappa);
  mrac_nh.getParam("sigma", set_mrac_params_srv.request.sigma);
  mrac_nh.getParam("attitude_p", set_mrac_params_srv.request.attitude_p);
  if (set_mrac_params_client.call(set_mrac_params_srv)) {
    ROS_INFO("Set MRAC Params in spinal");
  } else {
    ROS_ERROR("Failed to set MRAC Params in spinal");
  }

  mrac_nh.getParam("takeoff_wait_time", takeoff_wait_time_);

  // disable MRAC in the beginning
  spinalMRACTrigger(false);
  
  flight_state_sub_ = nh_.subscribe<std_msgs::UInt8>("flight_state", 10, &HydrusTiltedLQIAdaptiveController::flightStateCallback, this);
}

void HydrusTiltedLQIAdaptiveController::controlCore()
{
  HydrusTiltedLQIController::controlCore();

  desire_yaw_msg_.estimated_yaw = rpy_.z();
  desire_yaw_msg_.target_yaw = target_rpy_.z();
}

void HydrusTiltedLQIAdaptiveController::publishGain()
{
  HydrusTiltedLQIController::publishGain();

  desire_yaw_pub_.publish(desire_yaw_msg_);
} 

void HydrusTiltedLQIAdaptiveController::spinalMRACTrigger(bool trigger)
{
  ros::ServiceClient client = nh_.serviceClient<std_srvs::SetBool>("spinal_mrac_trigger");
  std_srvs::SetBool srv;
  srv.request.data = trigger;
  if (client.call(srv)) {
    is_using_mrac_ = trigger;
    if (is_using_mrac_) {
      ROS_INFO_STREAM("MRAC: Triggered true to spinal");
    }
  } else {
    ROS_ERROR("MRAC trigger failed");
  }
}

void HydrusTiltedLQIAdaptiveController::flightStateCallback(const std_msgs::UInt8::ConstPtr& msg)
{ 
  prev_flight_state_ = flight_state_;
  flight_state_ = msg->data;

  if (prev_flight_state_ == ARM_ON_STATE && flight_state_ == TAKEOFF_STATE) {
    takeoff_time_ = ros::Time::now();
    spinalMRACTrigger(false);
  } else if (prev_flight_state_ == TAKEOFF_STATE && flight_state_ == TAKEOFF_STATE) {
    if (!is_using_mrac_ && (ros::Time::now()-takeoff_time_).toSec()>takeoff_wait_time_) {
      spinalMRACTrigger(true);
    }
  } else if (prev_flight_state_ == TAKEOFF_STATE && flight_state_ == HOVER_STATE) {
    if (!is_using_mrac_) {
      spinalMRACTrigger(true);
    }
  } else if (flight_state_ == HOVER_STATE) {
    if (!is_using_mrac_) {
      spinalMRACTrigger(true);
    }
  } else if (flight_state_ == LAND_STATE) {
    // pass
  } else if (prev_flight_state_ != flight_state_) {
    // disable MRAC on state change
    spinalMRACTrigger(false);
  }
}

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::HydrusTiltedLQIAdaptiveController, aerial_robot_control::ControlBase);
