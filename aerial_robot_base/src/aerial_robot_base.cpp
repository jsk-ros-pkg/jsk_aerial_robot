#include <aerial_robot_base/aerial_robot_base.h>

AerialRobotBase::AerialRobotBase(ros::NodeHandle nh, ros::NodeHandle nh_private)
  : nh_(nh), nhp_(nh_private)
{

  bool verbose;
  nhp_.param ("verbose", verbose, true);

  nhp_.param ("main_rate", main_rate_, 0.0);
  if(verbose) cout << nhp_.getNamespace() << ": main_rate is " << main_rate_ << endl;

  //*** estimator
  estimator_ = new RigidEstimator(nh_, nhp_);

  //*** basic navigation
  navigator_ = new Navigator(nh_, nhp_, estimator_);

  //*** flight controller
  controller_loader_ptr_ =  boost::shared_ptr< pluginlib::ClassLoader<control_plugin::ControlBase> >( new pluginlib::ClassLoader<control_plugin::ControlBase>("aerial_robot_base", "control_plugin::ControlBase"));
  std::string control_plugin_name;
  nhp_.param ("control_plugin_name", control_plugin_name, std::string("control_plugin/flatness_pid"));
  controller_ = controller_loader_ptr_->createInstance(control_plugin_name);
  controller_->initialize(nh_, nhp_, estimator_, navigator_, main_rate_);

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
