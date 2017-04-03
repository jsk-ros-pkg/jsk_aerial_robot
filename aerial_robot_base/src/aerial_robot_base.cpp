#include <aerial_robot_base/aerial_robot_base.h>

AerialRobotBase::AerialRobotBase(ros::NodeHandle nh, ros::NodeHandle nh_private)
  : nh_(nh), nhp_(nh_private)
{

  rosParamInit();

  //*** estimator
  estimator_ = new RigidEstimator(nh_, nhp_);

  //*** control input
  flight_ctrl_input_ = new FlightCtrlInput(motor_num_);

  //*** teleop navigation
  navigator_ = new TeleopNavigator(nh_, nhp_, estimator_, flight_ctrl_input_, main_rate_);

  //*** pid controller
  controller_ = new PidController(nh_, nhp_, estimator_, navigator_, flight_ctrl_input_, main_rate_);

  if(tf_pub_rate_ <= 0)
    ROS_ERROR("Disable the tfPub function\n");
  else
    {
      tf_thread_ = boost::thread(boost::bind(&AerialRobotBase::tfPubFunc, this));
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
  delete controller_;
  delete navigator_;
  delete flight_ctrl_input_;

  tf_thread_.interrupt();
  tf_thread_.join();
}

void AerialRobotBase::rosParamInit()
{

  ros::NodeHandle nh("~");
  bool param_verbose;
  nh.param ("param_verbose", param_verbose, true);

  nhp_.param ("main_rate", main_rate_, 0.0);
  if(param_verbose) cout << nhp_.getNamespace() << ": main_rate is " << main_rate_ << endl;

  nhp_.param ("tf_pub_rate", tf_pub_rate_, 100.0);
  if(param_verbose) cout << nhp_.getNamespace() << ": tf_pub_rate is " << tf_pub_rate_ << endl;

  ros::NodeHandle motor_info_node("motor_info");
  motor_info_node.param ("motor_num", motor_num_, 0);
  if(param_verbose) cout << motor_info_node.getNamespace() << ": motor_num is " << motor_num_ << endl;
}

void AerialRobotBase::mainFunc(const ros::TimerEvent & e)
{
  navigator_->teleopNavigation();
  controller_->pidFunction();
  navigator_->sendAttCmd();
}

void AerialRobotBase::tfPubFunc()
{

  ros::Rate loop_rate(tf_pub_rate_);

  while(ros::ok())
    {
      estimator_->tfPublish();

      ros::spinOnce();
      loop_rate.sleep();
    }
}

