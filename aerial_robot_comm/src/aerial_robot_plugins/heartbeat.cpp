#include <aerial_robot_comm/aerial_robot_plugins/aerial_robot_base_plugin.h>
#include <std_msgs/Empty.h>
#include <mavros_msgs/State.h>

using namespace std;

namespace aerial_robot_plugin
{
  class Heartbeat :public Base
  {
  public:
    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp)
    {
      nh_ = ros::NodeHandle(nh, "heartbeat");
      nhp_ = ros::NodeHandle(nhp, "heartbeat");

      baseParamInit();

      nhp_.param("timeout", timeout_, 1.0);

      heartbeat_sub_= nh_.subscribe<mavros_msgs::State>("/mavros/state", 10, &Heartbeat::heartbeatCb, this);
      flight_command_pub_= nh_.advertise<std_msgs::Empty>(topic_name_, 1);

      timeout_check_ = false;
    }

    Heartbeat(){}
    ~Heartbeat(){}

  protected:

    ros::Publisher flight_command_pub_;
    ros::Subscriber heartbeat_sub_;
    string topic_name_;
    double timeout_;
    bool timeout_check_;

    void spin(const ros::TimerEvent & e)
    {
      if(!timeout_check_) return;

      if(ros::Time::now().toSec() - prev_t_.toSec() > timeout_)
        {
          /* publish the flight command */
          ROS_WARN("mavlink heartbeat: timeout");
          timeout_check_ = false;
          flight_command_pub_.publish(std_msgs::Empty());
        }
    }


    void heartbeatCb(const mavros_msgs::State::ConstPtr & rmsg)
    {
      if(!rmsg->connected) return; // from onboard node, not via wireless transmission

      prev_t_ = ros::Time::now();
      if(!timeout_check_) timeout_check_ = true;
    }
  };
};


PLUGINLIB_EXPORT_CLASS(aerial_robot_plugin::Heartbeat, aerial_robot_plugin::Base);
