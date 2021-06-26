/**
  ******************************************************************************
  * @file    LwIP/LwIP_TCP_Echo_Server/Src/app_ethernet.c 
  * @author  MCD Application Team
  * @brief   Ethernet specefic module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V.
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "lwip/opt.h"
#include "main.h"
#if LWIP_DHCP 
#include "lwip/dhcp.h"
#endif
#include "app_ethernet.h"
#include "ethernetif.h"
#ifdef USE_LCD
#include "Log/lcd_log.h"
#endif

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

uint32_t EthernetLinkTimer;

#if LWIP_DHCP 
#define MAX_DHCP_TRIES  4
uint32_t DHCPfineTimer = 0;
uint8_t DHCP_state = DHCP_OFF;
#endif

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Notify the User about the nework interface config status 
  * @param  netif: the network interface
  * @retval None
  */
void ethernet_link_status_updated(struct netif *netif) 
{
  if (netif_is_link_up(netif))
  {
#if LWIP_DHCP 
    /* Update DHCP state machine */
    DHCP_state = DHCP_START;
#elif defined(USE_LCD)   
    uint8_t iptxt[20];
    sprintf((char *)iptxt, "%s", ip4addr_ntoa(netif_ip4_addr(netif)));
    LCD_UsrLog ("Static IP address: %s\n", iptxt);
#else
//    BSP_LED_On(LED1);
//    BSP_LED_Off(LED2);
#endif /* LWIP_DHCP */
  }
  else
  {  
#if LWIP_DHCP 
    /* Update DHCP state machine */
    DHCP_state = DHCP_LINK_DOWN;
#elif defined(USE_LCD)
    LCD_UsrLog ("The network cable is not connected \n");
#else
//    BSP_LED_Off(LED1);
//    BSP_LED_On(LED2);
#endif /* LWIP_DHCP */
  } 
}

#if LWIP_NETIF_LINK_CALLBACK 
/**
  * @brief  Ethernet Link periodic check
  * @param  netif 
  * @retval None
  */
void Ethernet_Link_Periodic_Handle(struct netif *netif)
{
  /* Ethernet Link every 100ms */
  if (HAL_GetTick() - EthernetLinkTimer >= 100)
  {
    EthernetLinkTimer = HAL_GetTick();
    ethernet_link_check_state(netif);
  }
}
#endif

#if LWIP_DHCP 
/**
  * @brief  DHCP_Process_Handle
  * @param  None
  * @retval None
  */
void DHCP_Process(struct netif *netif)
{
  ip_addr_t ipaddr;
  ip_addr_t netmask;
  ip_addr_t gw;
  struct dhcp *dhcp;
#ifdef USE_LCD  
  uint8_t iptxt[20];
#endif
  
  switch (DHCP_state)
  {
    case DHCP_START:
    {
#ifdef USE_LCD
      LCD_UsrLog ("  State: Looking for DHCP server ...\n");
#else
      BSP_LED_Off(LED1);
      BSP_LED_Off(LED2); 
#endif
      ip_addr_set_zero_ip4(&netif->ip_addr);
      ip_addr_set_zero_ip4(&netif->netmask);
      ip_addr_set_zero_ip4(&netif->gw);
      dhcp_start(netif);
      DHCP_state = DHCP_WAIT_ADDRESS;
    }
    break;
    
  case DHCP_WAIT_ADDRESS:
    {
      if (dhcp_supplied_address(netif)) 
      {
        DHCP_state = DHCP_ADDRESS_ASSIGNED;
#ifdef USE_LCD        
        sprintf((char *)iptxt, "%s", ip4addr_ntoa(netif_ip4_addr(netif)));
        LCD_UsrLog ("IP address assigned by a DHCP server: %s\n", iptxt); 
#else
        BSP_LED_On(LED1);
        BSP_LED_Off(LED2); 
#endif
      }
      else
      {
        dhcp = (struct dhcp *)netif_get_client_data(netif, LWIP_NETIF_CLIENT_DATA_INDEX_DHCP);
    
        /* DHCP timeout */
        if (dhcp->tries > MAX_DHCP_TRIES)
        {
          DHCP_state = DHCP_TIMEOUT;
          
          /* Stop DHCP */
          dhcp_stop(netif);
          
          /* Static address used */
          IP_ADDR4(&ipaddr, IP_ADDR0 ,IP_ADDR1 , IP_ADDR2 , IP_ADDR3 );
          IP_ADDR4(&netmask, NETMASK_ADDR0, NETMASK_ADDR1, NETMASK_ADDR2, NETMASK_ADDR3);
          IP_ADDR4(&gw, GW_ADDR0, GW_ADDR1, GW_ADDR2, GW_ADDR3);
          netif_set_addr(netif, &ipaddr, &netmask, &gw);

#ifdef USE_LCD
          sprintf((char *)iptxt, "%s", ip4addr_ntoa(netif_ip4_addr(netif)));
          LCD_UsrLog ("DHCP Timeout !! \n");
          LCD_UsrLog ("Static IP address: %s\n", iptxt);
#else
          BSP_LED_On(LED1);
          BSP_LED_Off(LED2); 
#endif
        }
      }
    }
    break;
  case DHCP_LINK_DOWN:
    {
      /* Stop DHCP */
      dhcp_stop(netif);
      DHCP_state = DHCP_OFF;
#ifdef USE_LCD
      LCD_UsrLog ("The network cable is not connected \n");
#else
      BSP_LED_Off(LED1);
      BSP_LED_On(LED2);
#endif
    }
    break;
  default: break;
  }
}

/**
  * @brief  DHCP periodic check
  * @param  netif 
  * @retval None
  */
void DHCP_Periodic_Handle(struct netif *netif)
{  
  /* Fine DHCP periodic process every 500ms */
  if (HAL_GetTick() - DHCPfineTimer >= DHCP_FINE_TIMER_MSECS)
  {
    DHCPfineTimer =  HAL_GetTick();
    /* process DHCP state machine */
    DHCP_Process(netif);  
  }
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
