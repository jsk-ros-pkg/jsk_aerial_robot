#include "uav_comm/uav_comm.h"

using namespace xbee;
using namespace mavconn;

UavComm::UavComm(ros::NodeHandle nh, ros::NodeHandle nhp):
  nh_(nh), nhp_(nhp)
{
  //**** rosparam for XBee
  nhp_.param("port", port_, std::string("/dev/ttyUSB0"));
  nhp_.param("baudrate", baudrate_, 9600);
  nhp_.param("mode", mode_, 0);
  nhp_.param("rx_rate", rx_rate_, 20.0);
  nhp_.param("rssi_rate", rssi_rate_, 1.0);
  nhp_.param("debug_verbose", debug_verbose_, false);
  nhp_.param("tx_debug", tx_debug_, false);
  nhp_.param("gcs_debug", gcs_debug_, false);

  /* the id for UAV, not GCS! */
  nhp_.param("mav_sys_id", mav_sys_id_, 1);
  nhp_.param("mav_comp_id", mav_comp_id_, 200);

  xbee_handler_ = new XBee(port_, baudrate_);

  msg_from_uav_pub_ = nh_.advertise<mavros_msgs::Mavlink>("xbee/from", 5);
  msg_to_uav_sub_ = nh_.subscribe("xbee/to", 10, &UavComm::mavlinkCb, this,
                                     ros::TransportHints().tcpNoDelay());

  if(mode_ == GC_MODE)
    {
      rssi_pub_ = nh_.advertise<mavros_msgs::RadioStatus>("radio_status", 10);
      rssi_timer_ = nhp_.createTimer (ros::Duration(1 / rssi_rate_), &UavComm::rssiCheck, this);
    }

  rx_timer_ = nhp_.createTimer (ros::Duration(1 / rx_rate_), &UavComm::rxSpin, this);
}

UavComm::~UavComm ()
{
  delete xbee_handler_;
}

void UavComm::rssiCheck(const ros::TimerEvent & e)
{
  /* check rssi */
  uint8_t rssiCmd[2];
  rssiCmd[0] = 'D'; rssiCmd[1] = 'B';
  RemoteAtCommandRequest remote_at_req(RemoteAtCommandRequest::broadcastAddress64, rssiCmd, NULL, 0);
  xbee_handler_->send(remote_at_req);
}

void UavComm::mavlinkCb(const mavros_msgs::Mavlink::ConstPtr &rmsg)
{
  mavlink_message_t mmsg;

  if (mavros_msgs::mavlink::convert(*rmsg, mmsg))
    {
      MsgBuffer buf(&mmsg);
      /* send to xbee tx */
      /* TODO: broadcast is not good, using specified adress */
      ZBTxRequest zb_tx_req(buf.dpos(), buf.nbytes());
      xbee_handler_->send(zb_tx_req);
    }
  else
    ROS_ERROR("Drop mavlink packet: illegal payload64 size");
}

void UavComm::rxSpin(const ros::TimerEvent & e)
{
  /* test of the comm */
      static ros::Time prev_t = ros::Time::now();
      if(ros::Time::now().toSec() - prev_t.toSec() > 1.0)
        {
          prev_t = ros::Time::now();

          mavlink_message_t mmsg;
          if(tx_debug_)
            {
              /* send dummy rssi data as tx packet */
              mavlink_msg_radio_status_pack(mav_sys_id_, mav_comp_id_, &mmsg,
                                            255, 255, 10, 10, 10, 10, 10);
              MsgBuffer buf(&mmsg);
              ZBTxRequest zb_tx_req(buf.dpos(), buf.nbytes());
              xbee_handler_->send(zb_tx_req);
              ROS_INFO("TX Debug: send dummy rssi data");
            }

          if(gcs_debug_)
            {
              /* publish heartbeat */
              mavros_msgs::Mavlink rmsg;
              mavlink_msg_heartbeat_pack(mav_sys_id_, mav_comp_id_, &mmsg, MAV_TYPE_HELICOPTER, MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 0, MAV_STATE_ACTIVE);
              mavros_msgs::mavlink::convert(mmsg, rmsg);
              msg_from_uav_pub_.publish(rmsg);
              /* publish gps */
              mavlink_msg_gps_raw_int_pack(mav_sys_id_, mav_comp_id_, &mmsg,
                                           1000, 3, 358935660, 1399446280, 1000, UINT16_MAX, UINT16_MAX, 0, UINT16_MAX, 13);
              mavros_msgs::mavlink::convert(mmsg, rmsg);
              msg_from_uav_pub_.publish(rmsg);
            }
    }

  //**** receive
  xbee_handler_->readPacket();

  XBeeResponse packet = xbee_handler_->getResponse();
  if(!packet.isAvailable()) return;

  switch(packet.getApiId())
    {
    case ZB_RX_RESPONSE:
          {
            ZBRxResponse zb_rx_res;
            packet.getZBRxResponse(zb_rx_res);
            if(debug_verbose_) ROS_INFO("ZB_RX: receive data from %d", zb_rx_res.getRemoteAddress16());

            /* convert to mavlink_message_t */
            mavlink_message_t message;
            mavlink_status_t status;
            int channel = 0;
            for (size_t i = 0; i < zb_rx_res.getDataLength(); i++)
              {
                if (mavlink_parse_char(channel, zb_rx_res.getData(i), &message, &status))
                  {
                    mavros_msgs::Mavlink rmsg;
                    if(debug_verbose_) ROS_INFO("ZB_RX: convert to mavlink message");

                    mavros_msgs::mavlink::convert(message, rmsg);
                    msg_from_uav_pub_.publish(rmsg);
                  }
                }
            ROS_WARN("ZB_RX: can not convert to mavlink message");

            break;
          }
    case REMOTE_AT_COMMAND_RESPONSE:
      {
        /* rssi at command broadcasted from this device */
        RemoteAtCommandResponse remote_at_res;
        packet.getRemoteAtCommandResponse(remote_at_res);
        if (remote_at_res.isOk() && remote_at_res.getValueLength() > 0)
          {
            if(remote_at_res.getCommand()[0] =='D' && remote_at_res.getCommand()[1] =='B')
              {
                //uint16_t address = remote_at_res.getRemoteAddress16();
                XBeeAddress64 address = remote_at_res.getRemoteAddress64();
                uint8_t rssi = remote_at_res.getValue()[0];
                if(debug_verbose_)
                  ROS_INFO("Get DB Response from 0x%x , rssi is %d", address.getLsb(), rssi);

                /* publish the rssi */
                mavros_msgs::RadioStatus rssi_msg;
                rssi_msg.header.stamp = ros::Time::now();
                rssi_msg.rssi = rssi;
                rssi_msg.remrssi = rssi;
                rssi_msg.rssi_dbm = -rssi;
                rssi_pub_.publish(rssi_msg);

                /* publish to the mavros (gcs_link) */
                mavlink_message_t mmsg;
                mavlink_msg_radio_status_pack(mav_sys_id_, mav_comp_id_, &mmsg, rssi, rssi, 0, 0, 0, 0, 0);
                mavros_msgs::Mavlink rmsg;
                mavros_msgs::mavlink::convert(mmsg, rmsg);
                msg_from_uav_pub_.publish(rmsg);

              }
            else
              {
                ROS_WARN("Remote AT Response: other remote at response: %c%c",
                         remote_at_res.getCommand()[0],
                         remote_at_res.getCommand()[1]);
              }
          }
        else
          {
            ROS_WARN("Remote AT Response: can not get valid response");
          }
            break;
      }
    default:
      {
        ROS_WARN("RX_API ID:  0x%x", packet.getApiId());
      }
    }
}
