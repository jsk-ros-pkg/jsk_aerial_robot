#include <aerial_robot_base/aerial_robot_base.h>

AerialRobotBase::AerialRobotBase(ros::NodeHandle nh, ros::NodeHandle nh_private)
  : nh_(nh), nhp_(nh_private),
    controller_loader_("aerial_robot_control", "control_plugin::ControlBase"),
    navigator_loader_("aerial_robot_control", "aerial_robot_navigation::BaseNavigator")
{

  bool param_verbose;
  nhp_.param ("param_verbose", param_verbose, true);

  double main_rate;
  nhp_.param ("main_rate", main_rate, 0.0);

  // robot model
  robot_model_ros_ = boost::make_shared<aerial_robot_model::RobotModelRos>(nh_, nhp_);
  auto robot_model = robot_model_ros_->getRobotModel();

  // estimator
  estimator_ = boost::make_shared<aerial_robot_estimation::StateEstimator>();
  estimator_->initialize(nh_, nhp_, robot_model);

  // navigation
  std::string navi_plugin_name;
  if(nh_.getParam("flight_navigation_plugin_name", navi_plugin_name))
    {
      try
        {
          navigator_ = navigator_loader_.createInstance(navi_plugin_name);
        }
      catch(pluginlib::PluginlibException& ex)
        {
          ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
        }
    }
  else
    {
      ROS_DEBUG("use default class for flight navigation: aerial_robot_navigation::BaseNavigator");
      navigator_ = boost::make_shared<aerial_robot_navigation::BaseNavigator>();
    }
  navigator_->initialize(nh_, nhp_, robot_model, estimator_);

  //  controller
  try
    {
      std::string control_plugin_name;
      nh_.param ("control_plugin_name", control_plugin_name, std::string("control_plugin/flatness_pid"));
      controller_ = controller_loader_.createInstance(control_plugin_name);
      controller_->initialize(nh_, nhp_, robot_model, estimator_, navigator_, main_rate);
    }
  catch(pluginlib::PluginlibException& ex)
    {
      ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
    }

  if(param_verbose) cout << nhp_.getNamespace() << ": main_rate is " << main_rate << endl;
  if(main_rate <= 0)
    ROS_ERROR_STREAM("mian rate is negative, can not run the main timer");
  else
    {
      main_timer_ = nhp_.createTimer(ros::Duration(1.0 / main_rate), &AerialRobotBase::mainFunc, this);
    }
}

void AerialRobotBase::mainFunc(const ros::TimerEvent & e)
{
  navigator_->update();
  controller_->update();
}
