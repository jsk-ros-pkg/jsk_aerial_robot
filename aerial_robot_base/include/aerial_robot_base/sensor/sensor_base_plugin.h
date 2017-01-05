#ifndef SENOR_BASE_PLUGIN_H
#define SENOR_BASE_PLUGIN_H


//* ros
#include <ros/ros.h>

//* for kalman filter
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <aerial_robot_msgs/BoolFlag.h>
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
    virtual void initialize(ros::NodeHandle nh, ros::NodeHandle nhp, BasicEstimator* estimator, std::vector< boost::shared_ptr<sensor_base_plugin::SensorBase> > sensors, std::vector<std::string> sensor_names, int sensor_index)  = 0;
    virtual ~SensorBase(){}

    static const uint8_t EGOMOTION_ESTIMATION_MODE = 0;
    static const uint8_t GROUND_TRUTH_MODE = 1;
    static const uint8_t EXPERIMENT_MODE = 2;

    inline std::string getSensorName(){return sensor_name_;}

  protected:

    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;
    ros::Timer  health_check_timer_;
    ros::ServiceServer estimate_flag_service_;
    BasicEstimator* estimator_;
    std::string sensor_name_;
    int sensor_index_;
    int estimate_mode_;

    //std::vector< boost::shared_ptr<sensor_base_plugin::SensorBase> > sensors_;

    double sensor_hz_; // hz  of the sensor
    vector<int> estimate_indices_; // the fuser_egomation index
    vector<int> experiment_indices_; // the fuser_experiment indices

    int estimate_flag_;

    /* health check */
    bool health_;
    double health_check_rate_;
    double health_timeout_;
    int unhealth_level_;
    double health_stamp_;

    SensorBase(){}

    void baseParamInit(ros::NodeHandle nh, ros::NodeHandle nhp, BasicEstimator* estimator, std::string sensor_name, int sensor_index)
    {
      sensor_name_ = sensor_name;
      sensor_index_ = sensor_index;
      estimator_ = estimator;

      nh_ = ros::NodeHandle(nh, sensor_name_);
      nhp_ = ros::NodeHandle(nhp, sensor_name_);

      estimate_flag_ = true;
      sensor_hz_ = 0;
      estimate_indices_.resize(0);
      experiment_indices_.resize(0);
      health_ = false;

      estimate_flag_service_ = nh_.advertiseService("estimate_flag", &SensorBase::estimateFlag, this);

      std::string ns = nhp_.getNamespace();
      ROS_WARN("load sensor plugin %s:", ns.c_str());
      if (!nhp_.getParam ("estimate_mode", estimate_mode_))
        ROS_ERROR("%s, can not get param about estimate mode", ns.c_str());
      printf("%s,  estimate mode  is %d\n", ns.c_str(), estimate_mode_);

      nhp_.param("health_timeout", health_timeout_, 0.5);
      nhp_.param("unhealth_level", unhealth_level_, 0);
      nhp_.param("health_check_rate", health_check_rate_, 100.0);
      health_stamp_ =  ros::Time::now().toSec();

      health_check_timer_ = nhp_.createTimer(ros::Duration(1.0 / health_check_rate_), &SensorBase::healthCheck,this);

    }

    virtual void estimateProcess(){};
    /* check whether we get sensor data */
    void healthCheck(const ros::TimerEvent & e)
    {
      /* this will call only once, no recovery */
      if(ros::Time::now().toSec() - health_stamp_ > health_timeout_ && health_)
        {
          ROS_ERROR("%s: can not get fresh sensor data for %f[sec]", nhp_.getNamespace().c_str(), ros::Time::now().toSec() - health_stamp_);
          /* TODO: the solution to unhealth should be more clever */
          estimator_->setUnhealthLevel(unhealth_level_);
          health_ = false;
        }
    }

    bool estimateFlag(aerial_robot_msgs::BoolFlag::Request  &req,
                      aerial_robot_msgs::BoolFlag::Response &res)
    {
      std::string ns = nhp_.getNamespace();
      estimate_flag_ = req.flag;
      ROS_INFO("%s: %s", ns.c_str(), estimate_flag_?std::string("enable the estimate flag").c_str():std::string("disable the estimate flag").c_str());
    }

    inline void updateHealthStamp(double stamp)
    {
      if(!health_)
        {
          health_ = true;
          ROS_WARN("%s: get sensor data, du: %f", nhp_.getNamespace().c_str(), stamp - health_stamp_);
        }
      health_stamp_ = stamp;
    }

  };

};

#endif
