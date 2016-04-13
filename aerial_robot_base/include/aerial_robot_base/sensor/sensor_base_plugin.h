#ifndef SENOR_BASE_PLUGIN_H
#define SENOR_BASE_PLUGIN_H


//* ros
#include <ros/ros.h>

//* for kalman filter
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>

#include <aerial_robot_base/basic_state_estimation.h>
#include <kalman_filter/kf_base_plugin.h>
#include <kalman_filter/digital_filter.h>


using namespace Eigen;
using namespace std;

namespace sensor_base_plugin
{
  class SensorBase
  {
  public:
    virtual void initialize(ros::NodeHandle nh, ros::NodeHandle nhp, BasicEstimator* estimator)  = 0;
    virtual ~SensorBase(){}

    static const uint8_t EGOMOTION_ESTIMATION_MODE = 0;
    static const uint8_t GROUND_TRUTH_MODE = 1;
    static const uint8_t EXPERIMENT_MODE = 2;

  protected:

    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;
    BasicEstimator* estimator_;
    int estimate_mode_;

    SensorBase(){}

    void baseRosParamInit()
    {
      std::string ns = nhp_.getNamespace();
      if (!nhp_.getParam ("estimate_mode", estimate_mode_))
        ROS_ERROR("%s, can not get param about estimate mode", ns.c_str());


    }


  };

};

#endif
