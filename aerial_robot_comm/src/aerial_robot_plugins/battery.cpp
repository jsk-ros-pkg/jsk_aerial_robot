#include <aerial_robot_comm/aerial_robot_plugins/aerial_robot_base_plugin.h>
#include <std_msgs/UInt8.h>

#define VOLTAGE 3.7f
#define VOLTAGE_100P 4.2f
#define VOLTAGE_90P 4.085f
#define VOLTAGE_80P 3.999f
#define VOLTAGE_70P 3.936f
#define VOLTAGE_60P 3.883f
#define VOLTAGE_50P 3.839f
#define VOLTAGE_40P 3.812f
#define VOLTAGE_30P 3.791f
#define VOLTAGE_20P 3.747f
#define VOLTAGE_10P 3.683f
#define VOLTAGE_0P 3.209f

using namespace std;

namespace aerial_robot_plugin
{
  class Bat :public Base
  {
  public:
    void initialize(ros::NodeHandle nh, ros::NodeHandle nhp)
    {
      nh_ = ros::NodeHandle(nh, "bat");
      nhp_ = ros::NodeHandle(nhp, "bat");

      baseParamInit();

      cell_ = 0;

      nhp_.param("percentage_mode", percentage_mode_, false); //is [V] or [%]

      if(test_)
        {
          mavlink_msg_sys_status_pack(system_id_, component_id_, &mav_msg_,
                                      0, 0, 0, 0, 22200, 0, 85, 0, 0, 0, 0, 0, 0);
        }

      bat_sub_= nh_.subscribe<std_msgs::UInt8>(topic_name_, 5, &Bat::batCb, this);
    }

    Bat(){}
    ~Bat(){}

  protected:

    ros::Subscriber bat_sub_;
    string topic_name_;
    bool percentage_mode_;
    uint8_t cell_;

    void spin(const ros::TimerEvent & e)
    {
      baseProcess();
    }


    void batCb(const std_msgs::UInt8ConstPtr & rmsg)
    {
      float percentage = 0;

      /* we have to calculate the percentage from voltage */
      if(!percentage_mode_)
        {
          if(cell_ == 0) cell_ = rmsg->data / 10.0f / VOLTAGE;

          float average_voltage = rmsg->data / cell_;

          if(average_voltage  > VOLTAGE_90P) percentage = (average_voltage - VOLTAGE_90P) / (VOLTAGE_100P - VOLTAGE_90P) * 10 + 90;

          else if (average_voltage  > VOLTAGE_80P) percentage = (average_voltage - VOLTAGE_80P) / (VOLTAGE_90P - VOLTAGE_80P) * 10 + 80;

          else if (average_voltage  > VOLTAGE_70P) percentage = (average_voltage - VOLTAGE_70P) / (VOLTAGE_80P - VOLTAGE_70P) * 10 + 70;

          else if (average_voltage  > VOLTAGE_60P) percentage = (average_voltage - VOLTAGE_60P) / (VOLTAGE_70P - VOLTAGE_60P) * 10 + 60;

          else if (average_voltage  > VOLTAGE_50P) percentage = (average_voltage - VOLTAGE_50P) / (VOLTAGE_60P - VOLTAGE_50P) * 10 + 50;

          else if (average_voltage  > VOLTAGE_40P) percentage = (average_voltage - VOLTAGE_40P) / (VOLTAGE_50P - VOLTAGE_40P) * 10 + 40;

          else if (average_voltage  > VOLTAGE_30P) percentage = (average_voltage - VOLTAGE_30P) / (VOLTAGE_40P - VOLTAGE_30P) * 10 + 30;

          else if (average_voltage  > VOLTAGE_20P) percentage = (average_voltage - VOLTAGE_20P) / (VOLTAGE_30P - VOLTAGE_20P) * 10 + 20;

          else if (average_voltage  > VOLTAGE_10P) percentage = (average_voltage - VOLTAGE_10P) / (VOLTAGE_20P - VOLTAGE_10P) * 10 + 10;

          else percentage = (average_voltage - VOLTAGE_0P) / (VOLTAGE_10P - VOLTAGE_0P) * 10;

          if (percentage > 100) percentage = 100;

          else if (percentage < 0) percentage = 0;
        }

      mavlink_msg_sys_status_pack(system_id_, component_id_, &mav_msg_,
                                  0, 0, 0, 0, rmsg->data * 100, 0, percentage, 0, 0, 0, 0, 0, 0);

    }
  };
};


PLUGINLIB_EXPORT_CLASS(aerial_robot_plugin::Bat, aerial_robot_plugin::Base);
