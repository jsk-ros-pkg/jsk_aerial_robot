/**
 * @brief Odom plugin
 * @file odom.cpp
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

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_listener.h>

namespace mavplugin {
  /**
   * @brief Odom plugin.
   */
  class OdomPlugin : public MavRosPlugin {
  public:
    OdomPlugin() :
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
      nh.param("odom/test", test_, false);
      nh.param("odom/throttle", odom_throttle_, 1.0); // Hz
      if(mode_ == UAV_MODE)
        {
          odom_sub_ = nh.subscribe("/uav_state", 10, &OdomPlugin::odom_cb, this); 
          if(test_) test_timer_ = nh.createTimer(ros::Duration(1), &OdomPlugin::test, this); //1Hz
        }
      else if(mode_ == GC_MODE)
        {
          /* temporarily */
          gps_sub_ = nh.subscribe("/mavros/global_position/raw/fix", 10, &OdomPlugin::gps_cb, this);
          odom_pub_ = nh.advertise<nav_msgs::Odometry>("/mavlink/uav/state", 10);

        }

      odom_stamp_ = ros::Time::now();
    }

    const message_map get_rx_handlers() {
      return {
        MESSAGE_HANDLER(MAVLINK_MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET, &OdomPlugin::handle_odom),
          };
    }

    static const uint8_t UAV_MODE = 0; // uav side
    static const uint8_t GC_MODE = 1; // ground contorl side

  private:
    ros::NodeHandle nh;
    UAS *uas;
    std::string frame_id;

    ros::Publisher odom_pub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber gps_sub_; //Temporary for old qgroundcontrol
    ros::Timer test_timer_;

    int mode_;
    bool test_;
    double odom_throttle_; // the filtering transmission rate for joystick
    ros::Time odom_stamp_;

    sensor_msgs::NavSatFix gps_msg_; //Temporary for old qgroundcontrol

    void test(const ros::TimerEvent & e)
    {
      //ROS_WARN("mav odom: dummy pub");
      mavlink_message_t mav_msg;
      mavlink_msg_local_position_ned_system_global_offset_pack_chan(UAS_PACK_CHAN(uas),
                                                                    &mav_msg,
                                                                    1,
                                                                    0,
                                                                    0,
                                                                    25,
                                                                    0.2, -0.2, 1.57);

      UAS_FCU(uas)->send_message(&mav_msg);

    }

    void gps_cb(const sensor_msgs::NavSatFix::ConstPtr msg)
    {
      gps_msg_ = *msg;
    }

    void odom_cb(const nav_msgs::Odometry::ConstPtr msg)
    {
      if(ros::Time::now().toSec() - odom_stamp_.toSec() >  1/ odom_throttle_)
        {
          mavlink_message_t mav_msg;

          odom_stamp_ = msg->header.stamp;

          tfScalar y = 0;
          tfScalar r = 0;
          tfScalar p = 0;

          tf::Matrix3x3 basis(tf::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w));
          basis.getRPY(r, p, y);
          mavlink_msg_local_position_ned_system_global_offset_pack_chan(UAS_PACK_CHAN(uas),
                                                                        &mav_msg,
                                                                        odom_stamp_.toNSec() / 1e6,
                                                                        msg->pose.pose.position.x,
                                                                        msg->pose.pose.position.y,
                                                                        msg->pose.pose.position.z,
                                                                        r, -p, -y);

          UAS_FCU(uas)->send_message(&mav_msg);
        }
    }

    void handle_odom(const mavlink_message_t *msg, uint8_t sysid, uint8_t compid)
    {
      mavlink_local_position_ned_system_global_offset_t odom;
      mavlink_msg_local_position_ned_system_global_offset_decode(msg, &odom);

      nav_msgs::Odometry ros_msg;
      ros_msg.header.stamp.fromNSec(odom.time_boot_ms * 1e6);
      tf::Quaternion q;
      q.setRPY(odom.roll, -odom.pitch, -odom.yaw);
      ros_msg.pose.pose.position.x = odom.x;
      ros_msg.pose.pose.position.y = odom.y;
      ros_msg.pose.pose.position.z = odom.z;
      ros_msg.pose.pose.orientation.x = q.x();
      ros_msg.pose.pose.orientation.y = q.y();
      ros_msg.pose.pose.orientation.z = q.z();
      ros_msg.pose.pose.orientation.w = q.w();

      odom_pub_.publish(ros_msg);

      /* mavlink: position & attitude */
      mavlink_message_t mav_msg;
      mavlink_msg_attitude_pack_chan(UAS_PACK_CHAN(uas), &mav_msg,
                                     odom.time_boot_ms,
                                     odom.roll, odom.pitch, odom.yaw,
                                     0, 0, 0);
      UAS_FCU(uas)->message_received(&mav_msg, mav_msg.sysid, mav_msg.compid);

#if 0
      mavlink_msg_altitude_pack_chan(UAS_PACK_CHAN(uas), &mav_msg,
                                     odom.time_boot_ms,
                                     0, odom.z, odom.z, odom.z, odom.z, 0);
#else

      mavlink_msg_global_position_int_pack_chan(UAS_PACK_CHAN(uas), &mav_msg,
                                                odom.time_boot_ms, gps_msg_.latitude * 1e7, gps_msg_.longitude * 1e7,  gps_msg_.altitude * 1e3, gps_msg_.altitude * 1e3, 0, 0, 0, odom.yaw * 100 * 180 / M_PI);
#endif
      UAS_FCU(uas)->message_received(&mav_msg, mav_msg.sysid, mav_msg.compid);
    }

  };
};  // namespace mavplugin

PLUGINLIB_EXPORT_CLASS(mavplugin::OdomPlugin, mavplugin::MavRosPlugin)

