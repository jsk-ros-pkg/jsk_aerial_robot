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

#define MESSAGE_LEN 31

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;

class TelemetryDrone
{
public:
  TelemetryDrone(ros::NodeHandle nh, ros::NodeHandle nhp):nh_(nh), nhp_(nhp)
  {
    nhp_.param("port", port_, std::string("/dev/ttyUSB0"));
    nhp_.param("baudrate", baudrate_, 115200);
    nhp_.param("telemetry_rate", telemetry_rate_, 20.0);

    telemetry_serial_ = new serial::Serial(port_, baudrate_, serial::Timeout::simpleTimeout(1000));
    if(!telemetry_serial_->isOpen()) 
      {
      ROS_FATAL("can't open serial port of %s", port_.c_str());
      ros::shutdown();
      }
    else
      {
        ROS_INFO("open the serial port: %s", port_.c_str());
      }

    phase_ = PHASE_FIRST_HEADER;
    data_len_ = 0;
    chk_sum_ = 0;

    joy_pub_ = nh_.advertise<sensor_msgs::Joy>("joy", 1);
    telemetry_timer_ = nhp_.createTimer(ros::Duration(1.0 / telemetry_rate_), &TelemetryDrone::telemetryFunction, this);
  }

  ~TelemetryDrone()
  {
    delete telemetry_serial_;
  }

  void telemetryFunction(const ros::TimerEvent & e)
  {
    while(telemetry_serial_->available())
      {
        uint8_t read_data = 0;
        telemetry_serial_->read(&read_data, 1);
        switch(phase_)
          {
          case PHASE_FIRST_HEADER:
            {
              if(read_data == FIRST_HEADER)
                {
                  phase_ ++;
                }
              else
                phase_ = PHASE_FIRST_HEADER;
              break;
            }
          case PHASE_SECOND_HEADER:
            {
              if(read_data == SECOND_HEADER)
                {
                  phase_ ++;
                  data_len_ = 0;
                  chk_sum_ = 0;
                }
              else
                phase_ = PHASE_FIRST_HEADER;
              break;
            }
          case PHASE_MESSAGE_DATA:
            {
              telemetry_data_[data_len_] = read_data;
              chk_sum_ += read_data;
              data_len_++;
              if(data_len_ == MESSAGE_LEN)
                {
                  phase_++;
                }
              break;
            }
          case PHASE_MESSAGE_CHKSUM:
            {
              phase_ = PHASE_FIRST_HEADER;
              if(chk_sum_%256 + read_data == 255)
                {

                  //pub the node
                  sensor_msgs::Joy joy_msg;
                  joy_msg.header.stamp = ros::Time::now();
                  for(int i = 0; i < 15; i++)
                    joy_msg.buttons.push_back(telemetry_data_[i]);
                  for(int i = 0; i < 4; i++)
                    {
                      float axes_value = 0;
                      memcpy(&axes_value, &telemetry_data_[i*4 + 15], sizeof(4));
                      joy_msg.axes.push_back(axes_value);
                    }
                  joy_pub_.publish(joy_msg);
                }
              break;
            }
          default:
            break;
          }

      }
  }

  static const uint8_t FIRST_HEADER = 0xfe;
  static const uint8_t SECOND_HEADER = 0xfd;
  
  static const uint8_t PHASE_FIRST_HEADER = 0;
  static const uint8_t PHASE_SECOND_HEADER = 1;
  static const uint8_t PHASE_MESSAGE_DATA = 2;
  static const uint8_t PHASE_MESSAGE_CHKSUM = 3;
  

private:
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::Timer  telemetry_timer_;
  ros::Publisher joy_pub_;

  serial::Serial* telemetry_serial_;

  std::string port_;
  int baudrate_;
  double telemetry_rate_;

  int phase_;
  int data_len_;
  int chk_sum_;
  uint8_t telemetry_data_[MESSAGE_LEN];

};

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "telemetry_drone");
  ros::NodeHandle nh("telemetry_drone");
  ros::NodeHandle nhp("~");

  TelemetryDrone* telemetry_drone_;
  telemetry_drone_ = new TelemetryDrone(nh, nhp);

  ros::spin();

  delete telemetry_drone_;

  return 0;
}

