#include <jsk_quadcopter/tracking.h>

Tracking::Tracking(ros::NodeHandle nh, ros::NodeHandle nh_private): 
nh_(nh), nh_private_(nh_private, "tracking")
{

  navigator_ = new Navigator(nh, nh_private, 1);
  estimator_ = new Estimator();
}


Tracking::Tracking(ros::NodeHandle nh, ros::NodeHandle nh_private, Navigator* navigator, Estimator* estimator): nh_(nh), nh_private_(nh_private, "tracking"), estimator_(NULL), navigator_(NULL)
{
  //rosParamInit();
  navigator_ = navigator;
  estimator_ = estimator;

  tracking_joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 1, &Tracking::trackingCallback, this, ros::TransportHints().tcpNoDelay());

}

Tracking::~Tracking(){
}

void Tracking::rosParamInit()
{
#if 0
  std::string ns = nh_private_.getNamespace();
  if (!nh_private_.getParam ("tracking_flag", tracking_flag_))
    tracking_flag_ = false;
  printf("%s: tracking_flag is %s\n", ns.c_str(), tracking_flag_ ? ("true") : ("false"));
#endif
}

void Tracking::trackingCallback(const sensor_msgs::JoyConstPtr &joy_msg)
{
  static bool start_tracking_flag = false;

  if(joy_msg->buttons[12] == 1 && !start_tracking_flag)
    {
      ROS_INFO("start tracking");
      start_tracking_flag = true;

      //trial
      cam_shift_tracker_ = new CamShift(nh_, nh_private_, estimator_, navigator_);
    }
  if(joy_msg->buttons[14] == 1 && start_tracking_flag)
    {
      ROS_INFO("stop tracking");
      start_tracking_flag = false;

      //trial
      delete cam_shift_tracker_;
    }
}
