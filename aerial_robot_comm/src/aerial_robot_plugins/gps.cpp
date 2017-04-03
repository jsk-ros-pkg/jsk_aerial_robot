#include <aerial_robot_comm/aerial_robot_plugins/aerial_robot_base_plugin.h>
#include <aerial_robot_msgs/Gps.h>

using namespace std;

namespace aerial_robot_plugin
{
  class Gps :public Base
  {
  public:
    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp)
    {
      nh_ = ros::NodeHandle(nh, "gps");
      nhp_ = ros::NodeHandle(nhp, "gps");

      baseParamInit();

      if(test_)
        {
          mavlink_msg_gps_raw_int_pack(system_id_, component_id_, &mav_msg_,
                                       prev_t_.toNSec(), 3,
                                       35.893571 * 1e7, 139.944632 * 1e7,
                                       0, 0, 0, 0, 0, 13);
        }

      gps_sub_= nh_.subscribe<aerial_robot_msgs::Gps>(topic_name_, 5, &Gps::gpsCb, this);


    }

    Gps(){}
    ~Gps(){}

  protected:

    ros::Subscriber gps_sub_;
    string topic_name_;

    void spin(const ros::TimerEvent & e)
    {
      baseProcess();
    }


    void gpsCb(const aerial_robot_msgs::GpsConstPtr & rmsg)
    {
      mavlink_msg_gps_raw_int_pack(system_id_, component_id_, &mav_msg_,
                                   rmsg->stamp.toNSec(), 3,
                                   rmsg->location[0] * 1e7, rmsg->location[1] * 1e7,
                                   0, 0, 0, 0, 0, rmsg->sat_num);
    }
  };
};


PLUGINLIB_EXPORT_CLASS(aerial_robot_plugin::Gps, aerial_robot_plugin::Base);
