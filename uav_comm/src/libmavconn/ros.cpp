/**
 * @brief MAVConn ros link class
 * @file ros.cpp
 * @author Moju Zhao <chou@jsk.imi.i.u-tokyo.ac.jp>
 *
 * @addtogroup mavconn
 * @{
 */
/*
 * libmavconn
 * Copyright 2017 JSK, All rights reserved.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 */

#include <cassert>
#include <console_bridge/console.h>

#include <mavconn/thread_utils.h>
#include <uav_comm/libmavconn/ros.h>

namespace mavconn {

#define PFXd "mavconn: ros%d: "


  MAVConnRos::MAVConnRos(uint8_t system_id, uint8_t component_id) : MAVConnInterface(system_id, component_id), nh_()
  {
    logInform(PFXd "device: ros based interface");

    /* mavlink message from ros based interface (e.g. xbee of uav_comm) */
    msg_to_ros_pub_ = nh_.advertise<mavros_msgs::Mavlink>("xbee/to", 5);
    /* mavlink message to ros based interface(e.g. xbee of uav_comm) */
    msg_from_ros_sub_ = nh_.subscribe("xbee/from", 10, &MAVConnRos::mavlinkCb, this,
                                      ros::TransportHints().tcpNoDelay());
  }

  void MAVConnRos::send_message(const mavlink_message_t *message, uint8_t sysid, uint8_t compid)
  {
    assert(message != nullptr);

    logDebug(PFXd "send: Message-Id: %d [%d bytes] Sys-Id: %d Comp-Id: %d Seq: %d",
             channel, message->msgid, message->len, sysid, compid, message->seq);

    mavros_msgs::Mavlink rmsg;
    mavros_msgs::mavlink::convert(*message, rmsg);
    msg_to_ros_pub_.publish(rmsg);
  }

void MAVConnRos::mavlinkCb(const mavros_msgs::Mavlink::ConstPtr &rmsg)
{
  mavlink_message_t mmsg;
  mavros_msgs::mavlink::convert(*rmsg, mmsg);

  message_received(&mmsg, mmsg.sysid, mmsg.compid);
}

}; // namespace mavconn
