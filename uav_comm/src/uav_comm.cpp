#include "uav_comm/uav_comm.h"

using namespace xbee;
using namespace mavconn;

UavComm::UavComm(ros::NodeHandle nh, ros::NodeHandle nhp):
  nh_(nh), nhp_(nhp), addr64_(RemoteAtCommandRequest::broadcastAddress64),  addr16_(ZB_BROADCAST_ADDRESS)
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
      if(rssi_rate_ != 0)
        rssi_timer_ = nhp_.createTimer (ros::Duration(1.0), &UavComm::rssiCheck, this);
    }

  rx_timer_ = nhp_.createTimer (ros::Duration(1 / rx_rate_), &UavComm::rxSpin, this);
}

UavComm::~UavComm ()
{
  delete xbee_handler_;
}

void UavComm::rssiCheck(const ros::TimerEvent & e)
{
  static ros::Time prev_rssi_t = ros::Time::now();

  /* check rssi */
  uint8_t rssiCmd[2];
  rssiCmd[0] = 'D'; rssiCmd[1] = 'B';

  RemoteAtCommandRequest remote_at_req(addr64_, addr16_, rssiCmd, NULL, 0);
  if(addr16_ == ZB_BROADCAST_ADDRESS)
    {
      if(debug_verbose_) ROS_INFO("Broadcast remote at command request:DB");
      xbee_handler_->send(remote_at_req);
    }
  else
    {
      if(ros::Time::now().toSec() - prev_rssi_t.toSec() >  1 / rssi_rate_)
        {
          xbee_handler_->send(remote_at_req);
          prev_rssi_t = ros::Time::now();
        }
    }
}

void UavComm::mavlinkCb(const mavros_msgs::Mavlink::ConstPtr &rmsg)
{
  /* we skip the transmission, if we do not know the adress */
  if(addr16_ == ZB_BROADCAST_ADDRESS) return;

  mavlink_message_t mmsg;

  if (mavros_msgs::mavlink::convert(*rmsg, mmsg))
    {
      MsgBuffer buf(&mmsg);
      /* send to xbee tx */

      /* filter some mavlink form qGroundControl */
      if(rmsg->sysid == 255 && rmsg->compid == 0 && rmsg->msgid < MAVLINK_MSG_ID_PARAM_VALUE)
        return;

      //ROS_INFO("seq:%d, sysid: %d, compid:%d, msgid:%d, len:%d", rmsg->seq, rmsg->sysid, rmsg->compid, rmsg->msgid, rmsg->len);


      //XBeeAddress64 addr64 = RemoteAtCommandRequest::broadcastAddress64;
      XBeeAddress64 addr64 = XBeeAddress64(0x0, 0x0);

      ZBTxRequest zb_tx_req(addr64_, buf.dpos(), buf.nbytes());
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
              /* this is a broadcast mode, not good */
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
            if(debug_verbose_) ROS_INFO("ZB_RX: receive data from 0x%0x", zb_rx_res.getRemoteAddress16());

            /* update the address */
            if(addr16_ == ZB_BROADCAST_ADDRESS)
              {
                addr16_ = zb_rx_res.getRemoteAddress16();
                addr64_ = zb_rx_res.getRemoteAddress64();
                //addr64_ = XBeeAddress64(0xFFFFFFFF, 0xFFFFFFFF); // mode unknown
                ROS_INFO("Update, address16: 0x%x, address64: 0x%x", addr16_, addr64_.getLsb());
              }

            /* convert to mavlink_message_t */
            mavlink_message_t message;
            mavlink_status_t status;
            int channel = 0;
            for (size_t i = 0; i < zb_rx_res.getDataLength(); i++)
              {
                if (mavlink_parse_char(channel, zb_rx_res.getData(i), &message, &status))
                  {
                    mavros_msgs::Mavlink rmsg;
                    if(debug_verbose_) ROS_INFO("ZB_RX: convert to mavlink message, msg id: #%d", message.msgid);

                    mavros_msgs::mavlink::convert(message, rmsg);
                    msg_from_uav_pub_.publish(rmsg);
                    return;
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
            if(addr16_ == ZB_BROADCAST_ADDRESS)
              {
                addr16_ = remote_at_res.getRemoteAddress16();
                addr64_ = remote_at_res.getRemoteAddress64();
                //addr64_ = XBeeAddress64(0xFFFFFFFF, 0xFFFFFFFF); // mode unknown
                ROS_INFO("Update address16: 0x%x, address64: 0x%x", addr16_, addr64_.getLsb());
              }

            if(remote_at_res.getCommand()[0] =='D' && remote_at_res.getCommand()[1] =='B')
              {
                //uint16_t address = remote_at_res.getRemoteAddress16();
                XBeeAddress64 address = remote_at_res.getRemoteAddress64();
                int rssi = -remote_at_res.getValue()[0];
                if(debug_verbose_)
                  ROS_INFO("Get DB Response from 0x%x , rssi is %d", address.getLsb(), rssi);

                /* publish the rssi */
                mavros_msgs::RadioStatus rssi_msg;
                rssi_msg.header.stamp = ros::Time::now();
                rssi_msg.rssi = 0;
                rssi_msg.remrssi = 0;
                rssi_msg.rssi_dbm = rssi;
                rssi_pub_.publish(rssi_msg);

                /* publish to the mavros (gcs_link) */
                mavlink_message_t mmsg;
                //uint8_t rssi_3dr = (rssi + 127) * 1.9; // convert to 3dr rule
                uint8_t rssi_3dr = rssi; // convert to 3dr rule
                mavlink_msg_radio_status_pack(mav_sys_id_, mav_comp_id_, &mmsg, rssi_3dr, rssi_3dr, 0, 0, 0, 0, 0);
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
