#ifndef STATE_ESTIMATION_H
#define STATE_ESTIMATION_H

//* ros
#include <ros/ros.h>

#include <aerial_robot_base/basic_state_estimation.h>
//* for state estimate
#include <aerial_robot_base/sensor_base_plugin.h>

//* filter
#include <kalman_filter/kf_base_plugin.h>
#include <kalman_filter/digital_filter.h>

class RigidEstimator : public BasicEstimator
{
 public:
  RigidEstimator (ros::NodeHandle nh,
                  ros::NodeHandle nh_private,
                  bool simulation_flag);
  ~RigidEstimator ();

  void tfPublish();
//float getLaserToImuDistance();

 private:

  void rosParamInit();

  void statesBroadcast();



};



#endif
