#ifndef AERIAL_ROBOT_BASE_PLUGIN_H
#define AERIAL_ROBOT_BASE_PLUGIN_H

/* ros */
#include <ros/ros.h>
#include <iostream>

/* pluginlib */
#include <pluginlib/class_loader.h>
#include <pluginlib/class_list_macros.h>

/* mavlink */
#include <mavros/mavros_uas.h>
#include <mavros_msgs/Mavlink.h>
#include <mavros_msgs/mavlink_convert.h>

using namespace std;

namespace aerial_robot_plugin
{
  class Base
  {
  public:
    virtual void initialize(ros::NodeHandle nh, ros::NodeHandle nhp)  = 0;
    virtual ~Base(){}

  protected:

    ros::NodeHandle nh_;
    ros::NodeHandle nhp_;
    ros::Publisher mavlink_pub_;
    ros::Timer  spin_timer_;

    mavlink_message_t mav_msg_;
    ros::Time prev_t_;
    bool valid_;

    /* base param */
    string topic_name_;
    bool test_;
    double mavlink_pub_rate_;
    int system_id_, component_id_;

    Base(){}

    void baseParamInit()
    {

      nhp_.param("test", test_, false);
      //std::cout << nhp_.getNamespace() << ": " << "test: " << test_ << std::endl;
      nhp_.param("mavlink_pub_rate", mavlink_pub_rate_, 0.0);
      //std::cout << nhp_.getNamespace() << ": " << "mavlink pub rate: " << mavlink_pub_rate_ << std::endl;
      nhp_.param("topic_name", topic_name_, string("temp"));
      nhp_.param("system_id", system_id_, 1);
      nhp_.param("component_id", component_id_, 240);

      prev_t_ = ros::Time::now();
      valid_ = false;

      mavlink_pub_ = nh_.advertise<mavros_msgs::Mavlink>("/mavlink/to", 5); //to mavros

      spin_timer_ = nhp_.createTimer(ros::Duration(0.01), &Base::spin, this); //100Hz

    }

    void baseProcess()
    {
      if(mavlink_pub_rate_ == 0) return;

      if(!valid_ && !test_) return;


      if(ros::Time::now().toSec() - prev_t_.toSec() > 1 / mavlink_pub_rate_)
        {
          mavros_msgs::Mavlink rmsg;
          mavros_msgs::mavlink::convert(mav_msg_, rmsg);

          mavlink_pub_.publish(rmsg);

          prev_t_ = ros::Time::now();
        }

    }

    virtual void spin(const ros::TimerEvent & e) = 0;

  };

};

#endif
