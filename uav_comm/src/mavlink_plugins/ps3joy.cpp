/**
 * @brief Play Station3 Joystick plugin
 * @file ps3joy.cpp
 * @author Moju Zhao <chou@jsk.imi.i.u-tokyo.ac.jp>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2015 JSK
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 *
 */

#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>

#include <sensor_msgs/Joy.h>

namespace mavplugin {
/**
 * @brief PS3 joy plugin.
 */
class JoyControlPlugin : public MavRosPlugin {
public:
    JoyControlPlugin() :
        nh("~"),
        uas(nullptr)
    { }

    /**
     * Plugin initializer. Constructor should not do this.
     */
    void initialize(UAS &uas_)
    {
        uas = &uas_;
        nh.param("mode", mode_, 0);
        //ROS_INFO("MAV joy: mode is %d", mode_);
        nh.param("joy/throttle", joy_throttle_, 15.0); // Hz
        if(mode_ == UAV_MODE)
          joy_pub = nh.advertise<sensor_msgs::Joy>("/joy", 10);
        else if(mode_ == GC_MODE)
          joy_sub = nh.subscribe("/joy", 10, &JoyControlPlugin::joy_cb, this);

        joy_stamp_ = ros::Time::now();
    }

  const message_map get_rx_handlers() {
    return {
      MESSAGE_HANDLER(MAVLINK_MSG_ID_MANUAL_CONTROL, &JoyControlPlugin::handle_joy_control),
        };
  }


  static const uint8_t UAV_MODE = 0; // uav side
  static const uint8_t GC_MODE = 1; // ground contorl side

private:
    ros::NodeHandle nh;
    UAS *uas;
    std::string frame_id;

    ros::Publisher joy_pub;
    ros::Subscriber joy_sub;

  int mode_;
  double joy_throttle_; // the filtering transmission rate for joystick
  ros::Time joy_stamp_;

  void joy_cb(const sensor_msgs::Joy::ConstPtr msg)
  {
    if(ros::Time::now().toSec() - joy_stamp_.toSec() >  1/ joy_throttle_)
      {
        mavlink_message_t mav_msg;

        joy_stamp_ = ros::Time::now();

        uint16_t buttons = 0;
        for(int i = 0; i < 16; i++)
          buttons |= (msg->buttons[i] << i);

        mavlink_msg_manual_control_pack_chan(UAS_PACK_CHAN(uas),
                                             &mav_msg,
                                             0,
                                             msg->axes[0] * 1000.0,
                                             msg->axes[1] * 1000.0,
                                             msg->axes[2] * 1000.0,
                                             msg->axes[3] * 1000.0,
                                             buttons
                                             );
        UAS_FCU(uas)->send_message(&mav_msg);
      }
  }

  void handle_joy_control(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid) {
        mavlink_manual_control_t joy_control;
        mavlink_msg_manual_control_decode(msg, &joy_control);

        auto ros_msg = boost::make_shared<sensor_msgs::Joy>();
        ros_msg->header.stamp = ros::Time::now();
        ros_msg->axes.resize(4);
        ros_msg->axes[0] = (joy_control.x / 1000.0);
        ros_msg->axes[1] = (joy_control.y / 1000.0);
        ros_msg->axes[2] = (joy_control.z / 1000.0);
        ros_msg->axes[3] = (joy_control.r / 1000.0);

        ros_msg->buttons.resize(16);
        for(int i = 0; i < 16; i++)
          ros_msg->buttons[i] = (joy_control.buttons & (1 << i))?1:0;

        joy_pub.publish(ros_msg);
    }

};
};  // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::JoyControlPlugin, mavplugin::MavRosPlugin)

