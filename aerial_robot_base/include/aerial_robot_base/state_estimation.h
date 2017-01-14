#ifndef STATE_ESTIMATION_H
#define STATE_ESTIMATION_H

//* ros
#include <ros/ros.h>

#include <aerial_robot_base/basic_state_estimation.h>

//* for state estimate
#include <aerial_robot_base/sensor_base_plugin.h>

//* for search
#include <fnmatch.h>

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

bool pattern_match(std::string &pl, std::string &pl_candidate);

vector< boost::shared_ptr<sensor_plugin::SensorBase> > sensors_;
  boost::shared_ptr< pluginlib::ClassLoader<sensor_plugin::SensorBase> > sensor_plugin_ptr_;


};



#endif
