#include <ros/ros.h>

#include <string>
#include <iostream>
#include <cstdio>

#include <unistd.h>

#include <serial/serial.h>

#include <std_msgs/Int8.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Joy.h>

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;

class TelemetryBase
{
public:
  TelemetryBase(ros::NodeHandle nh, ros::NodeHandle nhp):nh_(nh), nhp_(nhp)
  {
    nhp_.param("port", port_, std::string("/dev/ttyUSB0"));
    nhp_.param("baudrate", baudrate_, 115200);
    nhp_.param("pub_rate", pub_rate_, 0.1);

    telemetry_serial_ = new serial::Serial(port_, baudrate_, serial::Timeout::simpleTimeout(1000));
    if(!telemetry_serial_->isOpen()) 
      {
        ROS_FATAL("can not open serial port");
      }

    joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 1, &TelemetryBase::joyStickControl, this, ros::TransportHints().udp());
  }

  ~TelemetryBase()
  {
    delete telemetry_serial_;
      }

  void joyStickControl(const sensor_msgs::JoyConstPtr & joy_msg)
  {
    //int buttons_size = joy_msg->buttons.size();
    //int axes_size = joy_msg->axes.size();
    //hard coding: 14 buttons and 4 axes
    static ros::Time prev_time = ros::Time::now();
  uint32_t chksum = 0;

    // 2 + 15 * 1 + 4 * 4 + 1 = 34
    uint8_t write_buffer[34];
    size_t message_len = 34;

    write_buffer[0] = 0xfe;
    write_buffer[1] = 0xfd;

    for(int i = 0; i < 15; i ++)
      {
        write_buffer[i + 2] = joy_msg->buttons[i];
        chksum += write_buffer[i + 2];
      }

    uint8_t axes_temp[] = {0,0,0,0};
    for(int i = 0; i < 4; i ++)
      {
        memcpy(&axes_temp, &(joy_msg->axes[i]), sizeof(4));
        for(int j = 0; j < 4; j ++)
          {
            write_buffer[i*4 + j + 17] = axes_temp[j];
            chksum += write_buffer[i*4 + j + 17];
          }
      }
    write_buffer[33] = 255 - chksum%256;

    if(ros::Time::now().toSec() - prev_time.toSec() > pub_rate_)
      {
        prev_time = ros::Time::now();
        telemetry_serial_->write(write_buffer, message_len);
      }
  }


private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::Subscriber joy_sub_;

  serial::Serial* telemetry_serial_;

  std::string port_;
  int baudrate_;
  double pub_rate_;

};

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "telemetry_base");
  ros::NodeHandle nh("telemetry_base");
  ros::NodeHandle nhp("~");

  TelemetryBase* telemetry_base_;
  telemetry_base_ = new TelemetryBase(nh, nhp);

  ros::spin();

  delete telemetry_base_;

  return 0;
}

