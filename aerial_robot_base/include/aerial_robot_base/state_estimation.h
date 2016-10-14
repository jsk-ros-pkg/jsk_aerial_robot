#ifndef STATE_ESTIMATION_H
#define STATE_ESTIMATION_H

//* ros
#include <ros/ros.h>

#include <aerial_robot_base/basic_state_estimation.h>

//* for state estimate
#include <aerial_robot_base/sensor/sensor_base_plugin.h>

//* for search
#include <algorithm>

class RigidEstimator : public BasicEstimator
{
 public:
  RigidEstimator (ros::NodeHandle nh,
                  ros::NodeHandle nh_private);
  ~RigidEstimator ();

  void tfPublish();

 private:

  void statesBroadcast();

  void rosParamInit();


  boost::shared_ptr< pluginlib::ClassLoader<sensor_base_plugin::SensorBase> > sensor_loader_ptr_;
  int sensor_no_;
  std::vector<std::string> sensor_plugin_name_;
  std::vector< boost::shared_ptr<sensor_base_plugin::SensorBase> > sensors_;


};



#endif
