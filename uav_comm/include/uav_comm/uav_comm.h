#ifndef UAV_COMM_H
#define UAV_COMM_H

/* ros */
#include <ros/ros.h>

/* XBEE Lib */
#include <uav_comm/xbee/xbee.h>

/* MAVLINK */
#include <mavros_msgs/mavlink_convert.h>
#include <mavconn/msgbuffer.h>

/* MSG */
#include <mavros_msgs/Mavlink.h>
#include <mavros_msgs/RadioStatus.h>


using namespace xbee;

class UavComm
{
private:

  //*** ROS basic
  ros::Timer rssi_timer_;
  ros::Timer rx_timer_;
  ros::NodeHandle nh_;
  ros::NodeHandle nhp_;
  ros::Publisher msg_from_uav_pub_;
  ros::Publisher rssi_pub_;
  ros::Subscriber msg_to_uav_sub_;

  //*** xbee gneneral library
  XBee* xbee_handler_;
  XBeeAddress64 addr64_;
  uint16_t addr16_;

  //*** ROS param
  std::string port_;
  int baudrate_;
  int mode_;
  double rx_rate_;
  double rssi_rate_;
  bool debug_verbose_;
  bool tx_debug_;
  bool gcs_debug_;

  //*** mavlink
  int mav_sys_id_;
  int mav_comp_id_;

  /* check the rssi of UAV */
  void rssiCheck(const ros::TimerEvent & e);
/* process for xbee rx */
  void rxSpin(const ros::TimerEvent & e);

  /* callback function of msg from ros(mavros) */
  void mavlinkCb(const mavros_msgs::Mavlink::ConstPtr &rmsg);
public:

  static const uint8_t UAV_MODE = 0; // for uav side
  static const uint8_t GC_MODE = 1;  // for ground control side

  UavComm (ros::NodeHandle nh, ros::NodeHandle nhp);
  virtual ~UavComm();

};

#endif
