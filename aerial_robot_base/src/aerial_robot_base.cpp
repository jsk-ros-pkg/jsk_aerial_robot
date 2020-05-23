#include <aerial_robot_base/aerial_robot_base.h>

AerialRobotBase::AerialRobotBase(ros::NodeHandle nh, ros::NodeHandle nh_private)
  : nh_(nh), nhp_(nh_private),
    controller_loader_("aerial_robot_base", "control_plugin::ControlBase")
{

  bool param_verbose;
  nhp_.param ("param_verbose", param_verbose, true);

  nhp_.param ("main_rate", main_rate_, 0.0);
  if(param_verbose) cout << nhp_.getNamespace() << ": main_rate is " << main_rate_ << endl;

  //*** robot model
  robot_model_ros_ = boost::make_shared<aerial_robot_model::RobotModelRos>(nh_, nhp_);
  auto robot_model = robot_model_ros_->getRobotModel();

  //*** estimator
  estimator_ = new StateEstimator(nh_, nhp_, robot_model);
  //*** basic navigation
  navigator_ = new Navigator(nh_, nhp_, robot_model, estimator_);
  //*** flight controller
  try
    {
      std::string control_plugin_name;
      nh_.param ("control_plugin_name", control_plugin_name, std::string("control_plugin/flatness_pid"));
      controller_ = controller_loader_.createInstance(control_plugin_name);
      controller_->initialize(nh_, nhp_, robot_model, estimator_, navigator_, main_rate_);
    }
  catch(pluginlib::PluginlibException& ex)
    {
      ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
    }

  if(main_rate_ <= 0)
    ROS_ERROR("Disable the tx function\n");
  else
    {
      main_timer_ = nhp_.createTimer(ros::Duration(1.0 / main_rate_), &AerialRobotBase::mainFunc,this);
    }

}

AerialRobotBase::~AerialRobotBase()
{
  delete estimator_;
  delete navigator_;
}

void AerialRobotBase::mainFunc(const ros::TimerEvent & e)
{
  navigator_->update();
  controller_->update();
}
