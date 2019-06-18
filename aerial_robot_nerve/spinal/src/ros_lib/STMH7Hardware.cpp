/* 
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2019, UT.
 * All rights reserved.
 */

#include "STMH7Hardware.h"
#include <string.h>

namespace
{
  struct udp_pcb *pcb_;
  RingBuffer<uint8_t, RX_BUFFER_SIZE>  rx_buf_;
  struct pbuf *p_;
  ip4_addr_t dst_addr_;
  uint16_t src_port_;
  uint16_t dst_port_;


  void udpRecvCb(void *arg, struct udp_pcb *tpcb, struct pbuf *p,
                     const ip_addr_t *addr, uint16_t port)
  {
    if (p != NULL)
      {
        if(port == src_port_)
          {
            rx_buf_.push(reinterpret_cast<uint8_t*>(p->payload), p->len);
            pbuf_free(p);
          }
      }
  }
}


void STMH7Hardware::init(const ip4_addr_t dst_addr, const uint16_t src_port, const uint16_t dst_port)
{
  dst_addr_ = dst_addr;
  src_port_ = src_port;
  dst_port_ = dst_port;

  /* get new pcb */
  pcb_ = udp_new();
  if (pcb_ == NULL) return;

  /* bind to any IP address on port 7 */
  if (udp_bind(pcb_, IP_ADDR_ANY, src_port_) != ERR_OK)
    {
      memp_free(MEMP_UDP_PCB, pcb_);
      return;
    }

  udp_recv(pcb_, udpRecvCb, NULL);
}

int STMH7Hardware::read()
{
  if(rx_buf_.length() == 0) return -1;

  uint8_t r_data = 0;
  rx_buf_.pop(&r_data);
  return  r_data;
}

int STMH7Hardware::read(uint8_t* data, uint16_t length)
{
  if(rx_buf_.length() == 0) return -1;
  if(length > rx_buf_.length()) return -1;

  rx_buf_.pop(data, length);

  return  1;
}

void STMH7Hardware::write(uint8_t * new_data, uint16_t new_size)
{
  p_ = pbuf_alloc(PBUF_TRANSPORT, new_size, PBUF_RAM);
  p_->payload = new_data;
  udp_sendto(pcb_, p_, &dst_addr_, dst_port_);
  pbuf_free(p_);  /* necessary: free the pbuf */
}


