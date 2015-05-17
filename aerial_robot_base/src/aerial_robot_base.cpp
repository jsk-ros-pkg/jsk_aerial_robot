#include "aerial_robot_base/aerial_robot_base.h"

AerialRobotBase::AerialRobotBase(ros::NodeHandle nh, ros::NodeHandle nh_private)
  : nh_(nh), nhp_(nh_private)
{

  rosParamInit(nhp_);

  //*** estimator1
  estimator_ = new Estimator1(nh_, nhp_, simulation_flag_);

  if(!simulation_flag_)
    { 
      //*** control input 
      flight_ctrl_input_ = new FlightCtrlInput();

      //*** teleop navigation
      navigator_ = new TeleopNavigator(nh_, nhp_, estimator_, flight_ctrl_input_, tx_loop_rate_);

      //*** pid controller
      controller_ = new PidController(nh_, nhp_, tx_loop_rate_);


      // if(trackingFlag_)
      //   tracker = new Tracking(quadcopterNodeHandle_, nhp_,
      //                          navigator, estimator);
    }


  if(tf_pub_loop_rate_ <= 0)
    ROS_ERROR("Disable the tfPub function\n");
  else
    {
      tf_thread_ = boost::thread(boost::bind(&AerialRobotBase::tfPubFunction, this));
    }


  if(tx_loop_rate_ <= 0)
    ROS_ERROR("Disable the tx function\n");
  else
    {
      txTimer_ = nhp_.createTimer(ros::Duration(1.0 / tx_loop_rate_), &AerialRobotBase::txFunction,this);
    }


  if(rx_loop_rate_ <= 0)
    ROS_ERROR("Disable the rx function\n");
  else
    rxTimer_ = nhp_.createTimer(ros::Duration(1.0 / rx_loop_rate_), &AerialRobotBase::rxFunction, this);
}

AerialRobotBase::~AerialRobotBase()
{
  delete estimator_;

  if(!simulation_flag_)
    {
      delete controller_;
      delete navigator_;
      delete  flight_ctrl_input_;
    }
  tf_thread_.interrupt();
  tf_thread_.join();

  printf("Deleted all!\n");
}

void AerialRobotBase::rosParamInit(ros::NodeHandle nh)
{
  std::string ns = nh.getNamespace();

  if (!nh.getParam ("rxLoopRate", rx_loop_rate_))
    rx_loop_rate_ = 0;
  printf("%s: rx_loop_rate_ is %.3f\n", ns.c_str(), rx_loop_rate_);

  if (!nh.getParam ("txLoopRate", tx_loop_rate_))
    tx_loop_rate_ = 0;
  printf("%s: tx_loop_rate_ is %.3f\n", ns.c_str(), tx_loop_rate_);


  if (!nh.getParam ("simulationFlag", simulation_flag_))
    simulation_flag_ = false;
  printf("%s: simulation_flag is %s\n", ns.c_str(), simulation_flag_ ? ("true") : ("false"));

  if (!nh.getParam ("tfPubLoopRate", tf_pub_loop_rate_))
    tf_pub_loop_rate_ = 0;
  printf("%s: tf_pub_loop_rate_ is %.3f\n", ns.c_str(),  tf_pub_loop_rate_);

  // if (!nh.getParam ("trackingFlag", trackingFlag_))
  //   trackingFlag_ = false;
  // printf("%s: trackingFlag is %s\n", ns.c_str(), trackingFlag_ ? ("true") : ("false"));

}

void AerialRobotBase::rxFunction(const ros::TimerEvent & e)
{
}


void AerialRobotBase::txFunction(const ros::TimerEvent & e)
{

  if(simulation_flag_)
    {
      navigator_->teleopNavigation();
      controller_->pidFunction();
      //feed forward control
      controller_->feedForwardFunction();

      if(controller_->getControlBoard() == controller_->KDUINO_BOARD)
        navigator_->sendRcCmd();
    }
}

void AerialRobotBase::tfPubFunction()
{

  ros::Rate loop_rate(tf_pub_loop_rate_);

  while(ros::ok())
    {
      estimator_->tfPublish();
      if(!simulation_flag_)
        navigator_->tfPublish();

      ros::spinOnce();
      loop_rate.sleep();
    }
}

