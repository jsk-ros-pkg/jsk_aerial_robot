/**
 * @brief MAVConn ros link class
 * @file ros.h
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

#pragma once

#include <list>
#include <atomic>
#include <mavconn/interface.h>
#include <mavconn/msgbuffer.h>

#include <ros/ros.h>
/* MAVLINK */
#include <mavros_msgs/mavlink_convert.h>
#include <mavros_msgs/Mavlink.h>

namespace mavconn {

  /**
   * @brief Ros interface
   */
  class MAVConnRos : public MAVConnInterface {
  public:
    /**
     * Open and run ros link.
     *
     */
    MAVConnRos(uint8_t system_id = 1, uint8_t component_id = MAV_COMP_ID_UDP_BRIDGE);
    ~MAVConnRos(){};

    void close() {};
    void send_bytes(const uint8_t *bytes, size_t length) {};
    bool is_open() {};


    using MAVConnInterface::send_message;
    void send_message(const mavlink_message_t *message, uint8_t sysid, uint8_t compid);

  private:
    ros::NodeHandle nh_;
    ros::Publisher msg_to_ros_pub_;
    ros::Subscriber msg_from_ros_sub_;

    void mavlinkCb(const mavros_msgs::Mavlink::ConstPtr &rmsg);
  };

}; // namespace mavconn

