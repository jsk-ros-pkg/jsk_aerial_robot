#ifndef TRACKING_H
#define TRACKING_H

#include <ros/ros.h>

#include <jsk_quadcopter/flight_navigation.h>
#include <jsk_quadcopter/state_estimation.h>
#include <jsk_quadcopter/tracker/checkerboard.h>
#include <jsk_quadcopter/tracker/camshift.h>
#include <sensor_msgs/Joy.h>

class Tracking
{
 public:
  Tracking(ros::NodeHandle nh,	ros::NodeHandle nh_private);

  Tracking(ros::NodeHandle nh,	ros::NodeHandle nh_private, 
           Navigator* navigator, 
           Estimator* estimator);
  ~Tracking();

 protected:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber tracking_joy_sub_;

  bool tracking_flag_;

  Estimator* estimator_;
  Navigator* navigator_;
  CheckerBoard* checker_board_tracker_;
  CamShift* cam_shift_tracker_;

  void trackingCallback(const sensor_msgs::JoyConstPtr &joy_msg);
  void rosParamInit();

};


#endif


