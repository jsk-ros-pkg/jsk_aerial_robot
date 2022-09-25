#include "udp_echoserver.h"
#include "lwip/debug.h"
#include "lwip/stats.h"
#include "lwip/udp.h"

#define SERVER_PORT 5005

// Reference: https://lists.nongnu.org/archive/html/lwip-users/2007-06/msg00078.html

namespace
{
  struct udp_pcb *pcb_;
  int i = 0;
  struct pbuf *p_;


  void udp_echo_recv(void *arg, struct udp_pcb *tpcb, struct pbuf *p,
                     const ip_addr_t *addr, uint16_t port)
  {
    if (p != NULL) {
      /* send received packet back to sender */

    	p_ = pbuf_alloc(PBUF_TRANSPORT, sizeof(char) * p->len, PBUF_RAM);
    	p_->payload = p->payload;
    	udp_sendto(tpcb, p_, addr, port);
    	pbuf_free(p_);       /* necessary: free the pbuf */

    	p_ = pbuf_alloc(PBUF_TRANSPORT, sizeof(int), PBUF_RAM);
    	i++;
    	p_->payload = &i;
    	udp_sendto(tpcb, p_, addr, port);
    	pbuf_free(p_);  /* necessary: free the pbuf */



    	pbuf_free(p);
    }
  }
};


void udp_echoserver_init(void)
{
  /* get new pcb */
  pcb_ = udp_new();
  if (pcb_ == NULL) return;

  /* bind to any IP address on port 7 */
  if (udp_bind(pcb_, IP_ADDR_ANY, SERVER_PORT) != ERR_OK)
    {
      memp_free(MEMP_UDP_PCB, pcb_);
      return;
    }

  /* set udp_echo_recv() as callback function
     for received packets */
  udp_recv(pcb_, udp_echo_recv, NULL);
}
