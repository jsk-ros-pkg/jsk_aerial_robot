/*
 * The MIT License (MIT)
 *
 * Copyright (c) [2018] [Marco Russi]
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
*/

#ifndef __CONF_ETH_H
#define __CONF_ETH_H

/* Disable lwIP checksum (performed by hardware). */
#define CHECKSUM_GEN_IP                               0
#define CHECKSUM_GEN_UDP                              0
#define CHECKSUM_GEN_TCP                              0
#define CHECKSUM_GEN_ICMP                             0
#define CHECKSUM_CHECK_IP                             0
#define CHECKSUM_CHECK_UDP                            0
#define CHECKSUM_CHECK_TCP                            0

/* Number of buffer for RX */
#define NETIF_RX_BUFFERS                              2

/* Number of buffer for TX */
#define NETIF_TX_BUFFERS                              2

/* MAC address definition.  The MAC address must be unique on the network. */
#define ETHERNET_CONF_ETHADDR0                        0x00
#define ETHERNET_CONF_ETHADDR1                        0x80
#define ETHERNET_CONF_ETHADDR2                        0xe1
#define ETHERNET_CONF_ETHADDR3                        0x00
#define ETHERNET_CONF_ETHADDR4                        0x00
#define ETHERNET_CONF_ETHADDR5                        0x00

/* The IP address being used. */
#define ETHERNET_CONF_IPADDR0                         192
#define ETHERNET_CONF_IPADDR1                         168
#define ETHERNET_CONF_IPADDR2                         25
#define ETHERNET_CONF_IPADDR3                         42

/* The gateway address being used. */
#define ETHERNET_CONF_GATEWAY_ADDR0                   192
#define ETHERNET_CONF_GATEWAY_ADDR1                   168
#define ETHERNET_CONF_GATEWAY_ADDR2                   25
#define ETHERNET_CONF_GATEWAY_ADDR3                   253

/* The DNS server address being used. */
#define ETHERNET_CONF_DNS_SERVER_ADDR0                192
#define ETHERNET_CONF_DNS_SERVER_ADDR1                168
#define ETHERNET_CONF_DNS_SERVER_ADDR2                25
#define ETHERNET_CONF_DNS_SERVER_ADDR3                6

/* The destination IP address */
#define ETHERNET_DST_IPADDR0                      		192
#define ETHERNET_DST_IPADDR1                         	168
#define ETHERNET_DST_IPADDR2                         	25
#define ETHERNET_DST_IPADDR3                         	40

/* The destination port number */
#define ETHERNET_DST_PORT                      			10000

/* The network mask being used. */
#define ETHERNET_CONF_NET_MASK0                       255
#define ETHERNET_CONF_NET_MASK1                       255
#define ETHERNET_CONF_NET_MASK2                       255
#define ETHERNET_CONF_NET_MASK3                       0

#endif
