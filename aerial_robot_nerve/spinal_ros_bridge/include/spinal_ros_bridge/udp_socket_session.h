/**
 *
 *  \file
 *  \brief      Reconnecting class for a UDP rosserial session.
 *  \author     Mike Purvis <mpurvis@clearpathrobotics.com>
 *  \copyright  Copyright (c) 2016, Clearpath Robotics, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL CLEARPATH ROBOTICS, INC. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Please send comments, questions, or patches to code@clearpathrobotics.com
 *
 */

#ifndef ROSSERIAL_SERVER_UDP_SOCKET_SESSION_H
#define ROSSERIAL_SERVER_UDP_SOCKET_SESSION_H

#include <iostream>
#include <boost/bind.hpp>
#include <boost/asio.hpp>

#include <ros/ros.h>

#include "spinal_ros_bridge/session.h"
#include "spinal_ros_bridge/udp_stream.h"


namespace rosserial_server
{

using boost::asio::ip::udp;

class UdpSocketSession : public Session<UdpStream>
{
public:
  UdpSocketSession(boost::asio::io_service& io_service,
                   udp::endpoint server_endpoint,
                   udp::endpoint client_endpoint)
    : Session(io_service), timer_(io_service),
      server_endpoint_(server_endpoint), client_endpoint_(client_endpoint),
      interval_(boost::posix_time::milliseconds(2000))
  {
    ROS_INFO_STREAM("rosserial_server UDP session created between " << server_endpoint << " and " << client_endpoint);
    connect_with_reconnection();
  }

private:
  ~UdpSocketSession()
  {
    ROS_WARN("UDP socket session shutting down. Waiting 1 second for system state to settle.");

    boost::shared_ptr<boost::asio::deadline_timer> timer
      (new boost::asio::deadline_timer(
                                       socket().get_io_service(),
                                       boost::posix_time::seconds(1)));

    // The timer instance is only passed to the callback in order to keep it alive for the
    // required lifetime. When the callback completes, it goes out of scope and is destructed.
    timer->async_wait(
                      boost::bind(&UdpSocketSession::restart_session,
                                  boost::ref(socket().get_io_service()), server_endpoint_, client_endpoint_));
  }

  static void restart_session(boost::asio::io_service& io_service, udp::endpoint server_endpoint, udp::endpoint client_endpoint)
  {
    if (ros::ok()) {
      ROS_INFO("Recreating UDP socket session.");
      new UdpSocketSession(io_service, server_endpoint, client_endpoint);
    } else {
      std::cout << "In shutdown, avoiding recreating UDP socket session." << std::endl;
    }
  }

  bool attempt_connection(bool log_errors = true)
  {
    socket().open(server_endpoint_, client_endpoint_);
    if (log_errors) ROS_INFO("Opening UDP port.");

    // Kick off the session.
    start();
    return true;
  }

  void connect_with_reconnection(bool log_errors = true) {
    if (!attempt_connection(log_errors)) {
      if (log_errors) {
        ROS_INFO_STREAM("Attempting reconnection every " << interval_.total_milliseconds() << " ms.");
      }
      timer_.expires_from_now(interval_);
      timer_.async_wait(boost::bind(&UdpSocketSession::connect_with_reconnection, this, false));
    } else {
    }
  }

  boost::posix_time::time_duration interval_;
  boost::asio::deadline_timer timer_;
  udp::endpoint server_endpoint_;
  udp::endpoint client_endpoint_;
};

}  // namespace

#endif  // ROSSERIAL_SERVER_UDP_SOCKET_SESSION_H
