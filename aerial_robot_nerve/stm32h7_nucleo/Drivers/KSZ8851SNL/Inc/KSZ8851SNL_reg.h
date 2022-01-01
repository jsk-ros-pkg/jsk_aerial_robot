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

#ifndef __KSZ8851SNL_REG_H
#define __KSZ8851SNL_REG_H

#ifdef __cplusplus
extern "C"
{
#endif


#define REG_ADDR_MASK              		0x3F0		/* Register address mask */
#define OPCODE_MASK                		(3 << 14)
#define CMD_READ                   		(0 << 14)
#define CMD_WRITE                  		(1 << 14)
#define FIFO_READ                  		0x80
#define FIFO_WRITE                 		0xC0

#define MIB_MASK   0x1C00

/*
 * MAC Registers
 * (Offset 0x00 - 0x25)
 */
#define REG_BUS_ERROR_STATUS    			0x06    	/* BESR */
#define BUS_ERROR_IBEC              	0x8000
#define BUS_ERROR_IBECV_MASK        	0x7800  	/* Default IPSec clock at 166Mhz */

#define REG_CHIP_CFG_STATUS        		0x08    	/* CCFG */
#define LITTLE_ENDIAN_BUS_MODE      	0x0400  	/* Bus in little endian mode */
#define EEPROM_PRESENCE             	0x0200  	/* External EEPROM is used */
#define SPI_BUS_MODE                	0x0100 	/* In SPI bus mode */
#define DATA_BUS_8BIT               	0x0080  	/* In 8-bit bus mode operation */
#define DATA_BUS_16BIT              	0x0040  	/* In 16-bit bus mode operation */
#define DATA_BUS_32BIT              	0x0020  	/* In 32-bit bus mode operation */
#define MULTIPLEX_MODE              	0x0010  	/* Data and address bus are shared */
#define CHIP_PACKAGE_128PIN         	0x0008  	/* 128-pin package */
#define CHIP_PACKAGE_80PIN          	0x0004 	/* 80-pin package */
#define CHIP_PACKAGE_48PIN          	0x0002  	/* 48-pin package */
#define CHIP_PACKAGE_32PIN          	0x0001  	/* 32-pin package for SPI host interface only */

#define REG_MAC_ADDR_0             		0x10  	/* MARL */
#define REG_MAC_ADDR_1             		0x11    	/* MARL */
#define REG_MAC_ADDR_2             		0x12    	/* MARM */
#define REG_MAC_ADDR_3             		0x13   	/* MARM */
#define REG_MAC_ADDR_4             		0x14    	/* MARH */
#define REG_MAC_ADDR_5             		0x15   	/* MARH */

#define LOW_QMU_MAC_REG             	0x10 /**< MAC Address Low */
#define MID_QMU_MAC_REG             	0x12 /**< MAC Address Middle*/
#define HIGH_QMU_MAC_REG            	0x14 /**< MAC Address High*/

#define OBC_REG                     	0x20 /**< On-Chip Bus Control Register */
#define REG_BUS_CLOCK_CTRL         		0x20   	/* OBCR */
#define BUS_CLOCK_166               	0x0004  	/* 166 MHz on-chip bus clock (defaul is 125MHz) */
#define BUS_CLOCK_DIVIDEDBY_5       	0x0003  	/* Bus clock devided by 5 */
#define BUS_CLOCK_DIVIDEDBY_3       	0x0002 	/* Bus clock devided by 3 */
#define BUS_CLOCK_DIVIDEDBY_2       	0x0001 	/* Bus clock devided by 2 */
#define BUS_CLOCK_DIVIDEDBY_1       	0x0000  	/* Bus clock devided by 1 */
#define BUS_CLOCK_DIVIDED_MASK      	0x0003 	/* Bus clock devider mask */

#define BUS_SPEED_166_MHZ           	0x0004  	/* Set bus speed to 166 MHz */
#define BUS_SPEED_125_MHZ           	0x0000  	/* Set bus speed to 125 MHz */
#define BUS_SPEED_83_MHZ            	0x0005  	/* Set bus speed to 83 MHz (166/2)*/
#define BUS_SPEED_62_5_MHZ          	0x0001 	/* Set bus speed to 62.5 MHz (125/2) */
#define BUS_SPEED_53_3_MHZ          	0x0006  	/* Set bus speed to 53.3 MHz (166/3) */
#define BUS_SPEED_41_7_MHZ          	0x0002 	/* Set bus speed to 41.67 MHz (125/3) */
#define BUS_SPEED_33_2_MHZ          	0x0007  	/* Set bus speed to 33.2 MHz (166/5) */
#define BUS_SPEED_25_MHZ            	0x0003  	/* Set bus speed to 25 MHz (125/5) */

#define REG_EEPROM_CTRL            		0x22   	/* EEPCR */
#define EEPROM_ACCESS_ENABLE        	0x0010  	/* Enable software to access EEPROM through bit 3 to bit 0 */
#define EEPROM_DATA_IN              	0x0008  	/* Data receive from EEPROM (EEDI pin) */
#define EEPROM_DATA_OUT             	0x0004  	/* Data transmit to EEPROM (EEDO pin) */
#define EEPROM_SERIAL_CLOCK        		0x0002  	/* Serial clock (EESK pin) */
#define EEPROM_CHIP_SELECT          	0x0001  	/* EEPROM chip select (EECS pin) */

#define REG_MEM_BIST_INFO          		0x24    	/* MBIR */
#define TX_MEM_TEST_FINISHED        	0x1000  	/* TX memeory BIST test finish */
#define TX_MEM_TEST_FAILED          	0x0800  	/* TX memory BIST test fail */
#define TX_MEM_TEST_FAILED_COUNT    	0x0700  	/* TX memory BIST test fail count */
#define RX_MEM_TEST_FINISHED        	0x0010  	/* RX memory BIST test finish */
#define RX_MEM_TEST_FAILED          	0x0008 	/* RX memory BIST test fail */
#define RX_MEM_TEST_FAILED_COUNT    	0x0003  	/* RX memory BIST test fail count */

#define GLOBAL_RESET_REG            	0x26 /**< Global Reset Register */
#define REG_RESET_CTRL             		0x26    	/* GRR */
#define QMU_SOFTWARE_RESET          	0x0002 	/* QMU soft reset (clear TxQ, RxQ) */
#define GLOBAL_SOFTWARE_RESET       	0x0001 	/* Global soft reset (PHY, MAC, QMU) */

/*
 * Wake On Lan Control Registers
 * (Offset 0x2A - 0x6B)
 */
#define REG_WOL_CTRL               		0x2A  	/* WFCR */
#define WOL_MAGIC_ENABLE            	0x0080  	/* Enable the magic packet pattern detection */
#define WOL_FRAME3_ENABLE           	0x0008  	/* Enable the wake up frame 3 pattern detection */
#define WOL_FRAME2_ENABLE           	0x0004 	/* Enable the wake up frame 2 pattern detection */
#define WOL_FRAME1_ENABLE           	0x0002 	/* Enable the wake up frame 1 pattern detection */
#define WOL_FRAME0_ENABLE           	0x0001 	/* Enable the wake up frame 0 pattern detection */

#define REG_WOL_FRAME0_CRC0        		0x30    	/* WF0CRC0 */
#define REG_WOL_FRAME0_CRC1        		0x32   	/* WF0CRC1 */
#define REG_WOL_FRAME0_BYTE_MASK0  		0x34   	/* WF0BM0 */
#define REG_WOL_FRAME0_BYTE_MASK1  		0x36 		/* WF0BM1 */
#define REG_WOL_FRAME0_BYTE_MASK2  		0x38   	/* WF0BM2 */
#define REG_WOL_FRAME0_BYTE_MASK3  		0x3A    	/* WF0BM3 */

#define REG_WOL_FRAME1_CRC0        		0x40   	/* WF1CRC0 */
#define REG_WOL_FRAME1_CRC1        		0x42    	/* WF1CRC1 */
#define REG_WOL_FRAME1_BYTE_MASK0  		0x44   	/* WF1BM0 */
#define REG_WOL_FRAME1_BYTE_MASK1  		0x46    	/* WF1BM1 */
#define REG_WOL_FRAME1_BYTE_MASK2  		0x48   	/* WF1BM2 */
#define REG_WOL_FRAME1_BYTE_MASK3  		0x4A   	/* WF1BM3 */

#define REG_WOL_FRAME2_CRC0        		0x50   	/* WF2CRC0 */
#define REG_WOL_FRAME2_CRC1        		0x52    	/* WF2CRC1 */
#define REG_WOL_FRAME2_BYTE_MASK0  		0x54  	/* WF2BM0 */
#define REG_WOL_FRAME2_BYTE_MASK1  		0x56   	/* WF2BM1 */
#define REG_WOL_FRAME2_BYTE_MASK2  		0x58   	/* WF2BM2 */
#define REG_WOL_FRAME2_BYTE_MASK3  		0x5A   	/* WF2BM3 */

#define REG_WOL_FRAME3_CRC0        		0x60 		/* WF3CRC0 */
#define REG_WOL_FRAME3_CRC1        		0x62   	/* WF3CRC1 */
#define REG_WOL_FRAME3_BYTE_MASK0  		0x64   	/* WF3BM0 */
#define REG_WOL_FRAME3_BYTE_MASK1  		0x66   	/* WF3BM1 */
#define REG_WOL_FRAME3_BYTE_MASK2  		0x68   	/* WF3BM2 */
#define REG_WOL_FRAME3_BYTE_MASK3  		0x6A    	/* WF3BM3 */

/*
 * Transmit/Receive Control Registers
 * (Offset 0x70 - 0x9F)
 */

/* Transmit Frame Header */
#define REG_QDR_DUMMY              		0x00   	/* Dummy address to access QMU RxQ, TxQ */
#define TX_CTRL_INTERRUPT_ON        	0x8000 	/* Transmit Interrupt on Completion */

#define TX_FLOW_CTRL_REG            	0x70 /**< Transmit Flow Control Register */
#define REG_TX_CTRL                		0x70   	/* TXCR */
#define TX_CTRL_ICMP_CHECKSUM       	0x0100 	/* Enable ICMP frame checksum generation */
#define TX_CTRL_UDP_CHECKSUM        	0x0080   /* Enable UDP frame checksum generation */
#define TX_CTRL_TCP_CHECKSUM        	0x0040   /* Enable TCP frame checksum generation */
#define TX_CTRL_IP_CHECKSUM         	0x0020   /* Enable IP frame checksum generation */
#define TX_CTRL_FLUSH_QUEUE         	0x0010   /* Clear transmit queue, reset tx frame pointer */
#define TX_CTRL_FLOW_ENABLE         	0x0008   /* Enable transmit flow control */
#define TX_CTRL_PAD_ENABLE          	0x0004   /* Eanble adding a padding to a packet shorter than 64 bytes */
#define TX_CTRL_CRC_ENABLE          	0x0002   /* Enable adding a CRC to the end of transmit frame */
#define TX_CTRL_ENABLE              	0x0001   /* Enable tranmsit */

#define REG_TX_STATUS              		0x72     /* TXSR */
#define TX_STAT_LATE_COL            	0x2000   /* Tranmsit late collision occurs */
#define TX_STAT_MAX_COL             	0x1000   /* Tranmsit maximum collision is reached */
#define TX_FRAME_ID_MASK            	0x003F   /* Transmit frame ID mask */
#define TX_STAT_ERRORS             		( TX_STAT_MAX_COL | TX_STAT_LATE_COL )

#define RX_FLOW_CTRL1_REG           	0x74 /**< Receive Flow Control Register 1 */
#define REG_RX_CTRL1               		0x74     /* RXCR1 */
#define RX_CTRL_FLUSH_QUEUE         	0x8000 	/* Clear receive queue, reset rx frame pointer */
#define RX_CTRL_UDP_CHECKSUM        	0x4000   /* Enable UDP frame checksum verification */
#define RX_CTRL_TCP_CHECKSUM        	0x2000   /* Enable TCP frame checksum verification */
#define RX_CTRL_IP_CHECKSUM         	0x1000   /* Enable IP frame checksum verification */
#define RX_CTRL_MAC_FILTER          	0x0800   /* Receive with address that pass MAC address filtering */
#define RX_CTRL_FLOW_ENABLE         	0x0400   /* Enable receive flow control */
#define RX_CTRL_BAD_PACKET          	0x0200   /* Eanble receive CRC error frames */
#define RX_CTRL_MULTICAST           	0x0100   /* Receive multicast frames that pass the CRC hash filtering */
#define RX_CTRL_BROADCAST           	0x0080   /* Receive all the broadcast frames */
#define RX_CTRL_ALL_MULTICAST       	0x0040   /* Receive all the multicast frames (including broadcast frames) */
#define RX_CTRL_UNICAST             	0x0020   /* Receive unicast frames that match the device MAC address */
#define RX_CTRL_PROMISCUOUS         	0x0010   /* Receive all incoming frames, regardless of frame's DA */
#define RX_CTRL_STRIP_CRC           	0x0008   /* Enable strip CRC on the received frames */
#define RX_CTRL_INVERSE_FILTER      	0x0002   /* Receive with address check in inverse filtering mode */
#define RX_CTRL_ENABLE              	0x0001   /* Enable receive */

/* Address filtering scheme mask */
#define RX_CTRL_FILTER_MASK    			( RX_CTRL_INVERSE_FILTER | RX_CTRL_PROMISCUOUS | RX_CTRL_MULTICAST | RX_CTRL_MAC_FILTER )

#define RX_FLOW_CTRL2_REG           	0x76 /**< Receive Flow Control Register 2 */
#define REG_RX_CTRL2               		0x76     /* RXCR2 */
#define RX_CTRL_IPV6_UDP_NOCHECKSUM 	0x0010   /* No checksum generation and verification if IPv6 UDP is fragment */
#define RX_CTRL_IPV6_UDP_CHECKSUM   	0x0008   /* Receive pass IPv6 UDP frame with UDP checksum is zero */
#define RX_CTRL_UDP_LITE_CHECKSUM   	0x0004   /* Enable UDP Lite frame checksum generation and verification */
#define RX_CTRL_ICMP_CHECKSUM       	0x0002   /* Enable ICMP frame checksum verification */
#define RX_CTRL_BLOCK_MAC           	0x0001   /* Receive drop frame if the SA is same as device MAC address */
#define RX_CTRL_BURST_LEN_MASK      	0x00e0  	/* SRDBL SPI Receive Data Burst Length */
#define RX_CTRL_BURST_LEN_4         	0x0000
#define RX_CTRL_BURST_LEN_8         	0x0020
#define RX_CTRL_BURST_LEN_16        	0x0040
#define RX_CTRL_BURST_LEN_32        	0x0060
#define RX_CTRL_BURST_LEN_FRAME     	0x0080
#define RXCR2_SRDBL_MASK			(0x7 << 5)
#define RXCR2_SRDBL_SHIFT			(5)
#define RXCR2_SRDBL_4B				(0x0 << 5)
#define RXCR2_SRDBL_8B				(0x1 << 5)
#define RXCR2_SRDBL_16B				(0x2 << 5)
#define RXCR2_SRDBL_32B				(0x3 << 5)
#define RXCR2_SRDBL_FRAME			(0x4 << 5)
#define RXCR2_IUFFP				    (1 << 4)
#define RXCR2_RXIUFCEZ				(1 << 3)
#define RXCR2_UDPLFE				(1 << 2)
#define RXCR2_RXICMPFCC				(1 << 1)
#define RXCR2_RXSAF				    (1 << 0)

#define TX_MEM_INFO_REG             	0x78 /**< TXQ Memory Information Register */
#define REG_TX_MEM_INFO            		0x78     /* TXMIR */
#define TX_MEM_AVAILABLE_MASK       	0x1FFF   /* The amount of memory available in TXQ */

#define RX_FRH_STAT_REG             	0x7C /**< Receive Frame Header Status Register */
#define REG_RX_FHR_STATUS          		0x7C     /* RXFHSR */
#define RX_VALID                    	0x8000   /* Frame in the receive packet memory is valid */
#define RX_ICMP_ERROR               	0x2000   /* ICMP checksum field doesn't match */
#define RX_IP_ERROR                 	0x1000   /* IP checksum field doesn't match */
#define RX_TCP_ERROR                	0x0800   /* TCP checksum field doesn't match */
#define RX_UDP_ERROR                	0x0400   /* UDP checksum field doesn't match */
#define RX_BROADCAST                	0x0080   /* Received frame is a broadcast frame */
#define RX_MULTICAST                	0x0040   /* Received frame is a multicast frame */
#define RX_UNICAST                  	0x0020   /* Received frame is a unicast frame */
#define RX_PHY_ERROR                	0x0010   /* Received frame has runt error */
#define RX_FRAME_ETHER              	0x0008   /* Received frame is an Ethernet-type frame */
#define RX_TOO_LONG                 	0x0004   /* Received frame length exceeds max size 0f 2048 bytes */
#define RX_RUNT_ERROR               	0x0002   /* Received frame was demaged by a collision */
#define RX_BAD_CRC                  	0x0001   /* Received frame has a CRC error */
#define RX_ERRORS                   	( RX_BAD_CRC | RX_TOO_LONG | RX_RUNT_ERROR | RX_PHY_ERROR | \
                                   		RX_ICMP_ERROR | RX_IP_ERROR | RX_TCP_ERROR | RX_UDP_ERROR )

#define RX_FRH_BC_REG               	0x7E /**< Receive Frame Header Bytecount Register */
#define REG_RX_FHR_BYTE_CNT        		0x7E     /* RXFHBCR */
#define RX_BYTE_CNT_MASK            	0x0FFF   /* Received frame byte size mask */

#define TXQ_CMD_REG                 	0x80 /**< TXQ Command Register */
#define REG_TXQ_CMD                		0x80     /* TXQCR */
#define TXQ_AUTO_ENQUEUE            	0x0004   /* Enable enqueue tx frames from tx buffer automatically */
#define TXQ_MEM_AVAILABLE_INT       	0x0002   /* Enable generate interrupt when tx memory is available */
#define TXQ_ENQUEUE                 	0x0001   /* Enable enqueue tx frames one frame at a time */

#define RXQ_CMD_REG                 	0x82 /**< RXQ Command Register */
#define REG_RXQ_CMD                		0x82     /* RXQCR */
#define RXQ_STAT_TIME_INT           	0x1000   /* RX interrupt is occured by timer duration */
#define RXQ_STAT_BYTE_CNT_INT       	0x0800   /* RX interrupt is occured by byte count threshold */
#define RXQ_STAT_FRAME_CNT_INT      	0x0400   /* RX interrupt is occured by frame count threshold */
#define RXQ_TWOBYTE_OFFSET          	0x0200   /* Enable adding 2-byte before frame header for IP aligned with DWORD */
#define RXQ_TIME_INT                	0x0080   /* Enable RX interrupt by timer duration */
#define RXQ_BYTE_CNT_INT            	0x0040   /* Enable RX interrupt by byte count threshold */
#define RXQ_FRAME_CNT_INT           	0x0020   /* Enable RX interrupt by frame count threshold */
#define RXQ_AUTO_DEQUEUE            	0x0010   /* Enable release rx frames from rx buffer automatically */
#define RXQ_START                   	0x0008   /* Start QMU transfer operation */
#define RXQ_CMD_FREE_PACKET         	0x0001   /* Manual dequeue (release the current frame from RxQ) */

#define RXQ_CMD_CNTL                	(RXQ_FRAME_CNT_INT|RXQ_AUTO_DEQUEUE)

#define TX_FD_PTR_REG               	0x84 /**< TX Frame Data Pointer Register */
#define REG_TX_ADDR_PTR            		0x84     /* TXFDPR */
#define TXFDPR_TXFPAI				(1 << 14)
#define TXFDPR_TXFP_MASK			(0x7ff << 0)
#define TXFDPR_TXFP_SHIFT			(0)

#define RX_FD_PTR_REG               	0x86 /**< RX Frame Data Pointer Register */
#define REG_RX_ADDR_PTR            		0x86     /* RXFDPR */
#define RXFDPR_RXFPAI				(1 << 14)
#define ADDR_PTR_AUTO_INC           	0x4000   /* Enable Frame data pointer increments automatically */
#define ADDR_PTR_MASK               	0x03ff   /* Address pointer mask */

#define REG_RX_TIME_THRES          		0x8C     /* RXDTTR */
#define RX_TIME_THRESHOLD_MASK      	0xFFFF   /* Set receive timer duration threshold */

#define REG_RX_BYTE_CNT_THRES      		0x8E     /* RXDBCTR */
#define RX_BYTE_THRESHOLD_MASK      	0xFFFF 	/* Set receive byte count threshold */

#define INT_ENABLE_REG              	0x90 /**< Interrupt Enable Register */
#define REG_INT_MASK               		0x90     /* IER */
#define INT_PHY                    		0x8000   /* Enable link change interrupt */
#define INT_TX                      	0x4000   /* Enable transmit done interrupt */
#define INT_RX                      	0x2000   /* Enable receive interrupt */
#define INT_RX_OVERRUN              	0x0800   /* Enable receive overrun interrupt */
#define INT_TX_STOPPED              	0x0200   /* Enable transmit process stopped interrupt */
#define INT_RX_STOPPED              	0x0100   /* Enable receive process stopped interrupt */
#define INT_TX_SPACE                	0x0040   /* Enable transmit space available interrupt */
#define INT_RX_WOL_FRAME            	0x0020   /* Enable WOL on receive wake-up frame detect interrupt */
#define INT_RX_WOL_MAGIC            	0x0010   /* Enable WOL on receive magic packet detect interrupt */
#define INT_RX_WOL_LINKUP           	0x0008   /* Enable WOL on link up detect interrupt */
#define INT_RX_WOL_ENERGY           	0x0004   /* Enable WOL on energy detect interrupt */
#define INT_RX_SPI_ERROR            	0x0002   /* Enable receive SPI bus error interrupt */
#define INT_RX_WOL_DELAY_ENERGY     	0x0001   /* Enable WOL on delay energy detect interrupt */
#define INT_MASK                    	( INT_RX | INT_TX | INT_PHY )
#define INT_NO_INT                     	0x0000

/* Interrupt Enable Register Options */
#define   INT_LINK_CHANGE     			0x8000	/** Enable link change interrupt */
#define   INT_TX_DONE         			0x4000	/** Enable transmit done interrupt */
#define   INT_RX_DONE         			0x2000	/** Enable receive interrupt */
#define   INT_RX_OVERRUN      			0x0800	/** Enable receive overrun interrupt */
#define   INT_TX_STOPPED      			0x0200	/** Enable transmit process stopped interrupt */
#define   INT_RX_STOPPED      			0x0100	/** Enable receive process stopped interrupt */
#define   INT_TX_SPACE        			0x0040	/** Enable transmit space available interrupt */
#define   INT_RX_WOL_FRAME    			0x0020	/** Enable WOL on receive wake-up frame detect interrupt */
#define   INT_MAGIC           			0x0010	/** Enable magic packet detect interrupt */
#define   INT_LINKUP          			0x0008	/** Enable link up detect interrupt */
#define   INT_ENERGY          			0x0004	/** Enable detect interrupt */
#define   INT_SPI_ERROR       			0x0002	/** Enable receive SPI bus error interrupt */

/** Interrupt mask initialization collection */
#define INT_MASK_EXAMPLE    	(KSZ8851SNL_INT_RX_DONE |    \
                                 KSZ8851SNL_INT_RX_OVERRUN | \
                                 KSZ8851SNL_INT_TX_STOPPED | \
                                 KSZ8851SNL_INT_RX_STOPPED | \
                                 KSZ8851SNL_INT_TX_DONE |    \
                                 KSZ8851SNL_INT_LINK_CHANGE)

/* Interrupt Enable Register Options */
#define KSZ8851SNL_INT_LINK_CHANGE     		0x8000
#define KSZ8851SNL_INT_TX_DONE         		0x4000
#define KSZ8851SNL_INT_RX_DONE         		0x2000
#define KSZ8851SNL_INT_RX_OVERRUN      		0x0800
#define KSZ8851SNL_INT_TX_STOPPED      		0x0200
#define KSZ8851SNL_INT_RX_STOPPED      		0x0100
#define KSZ8851SNL_INT_TX_SPACE        		0x0040
#define KSZ8851SNL_INT_RX_WOL_FRAME    		0x0020
#define KSZ8851SNL_INT_RX_WOL_MAGIC    		0x0010
#define KSZ8851SNL_INT_RX_WOL_LINKUP        0x0008
#define KSZ8851SNL_INT_RX_WOL_ENERGY        0x0004
#define KSZ8851SNL_INT_RX_SPI_ERROR       	0x0002
#define KSZ8851SNL_INT_RX_WOL_DELAY_ENERGY	0x0001
#define KSZ8851SNL_INT_ENABLE_MASK			(KSZ8851SNL_INT_RX_DONE | \
											 KSZ8851SNL_INT_RX_STOPPED | \
											 KSZ8851SNL_INT_TX_STOPPED | \
											 KSZ8851SNL_INT_LINK_CHANGE | \
											 KSZ8851SNL_INT_RX_SPI_ERROR)

#define INT_STATUS_REG              	0x92 /**< Interrupt Status Register */
#define REG_INT_STATUS             		0x92    	/* ISR */

#define IRQ_LCI					(1 << 15)
#define IRQ_TXI					(1 << 14)
#define IRQ_RXI					(1 << 13)
#define IRQ_RXOI				(1 << 11)
#define IRQ_TXPSI				(1 << 9)
#define IRQ_RXPSI				(1 << 8)
#define IRQ_TXSAI				(1 << 6)
#define IRQ_RXWFDI				(1 << 5)
#define IRQ_RXMPDI				(1 << 4)
#define IRQ_LDI					(1 << 3)
#define IRQ_EDI					(1 << 2)
#define IRQ_SPIBEI				(1 << 1)
#define IRQ_DEDI				(1 << 0)

#define RX_FRAME_THRES_REG          	0x9C /**< RX Frame Count & Threshold Register */
#define REG_RX_FRAME_CNT_THRES     		0x9C     /* RXFCTFC */
#define RX_FRAME_CNT_MASK           	0xFF00   /* Received frame count mask */
#define RX_FRAME_THRESHOLD_MASK     	0x00FF   /* Set receive frame count threshold mask */

#define KS_RXFCTR				    0x9D
#define RXFCTR_RXFC_MASK			(0xff << 8)
#define RXFCTR_RXFC_SHIFT			(8)
#define RXFCTR_RXFC_GET(_v)			(((_v) >> 8) & 0xff)
#define RXFCTR_RXFCT_MASK			(0xff << 0)
#define RXFCTR_RXFCT_SHIFT			(0)

#define TX_NEXT_FRS_REG             0x9E /**< TX Next Frame size register */
#define REG_TX_TOTAL_FRAME_SIZE    	0x9E     /* TXNTFSR */
#define TX_TOTAL_FRAME_SIZE_MASK    0xFFFF   /* Set next total tx frame size mask */

/*
 * MAC Address Hash Table Control Registers
 * (Offset 0xA0 - 0xA7)
 */
#define REG_MAC_HASH_0             		0xA0  	/* MAHTR0 */
#define REG_MAC_HASH_1             		0xA1
#define REG_MAC_HASH_2             		0xA2    	/* MAHTR1 */
#define REG_MAC_HASH_3             		0xA3
#define REG_MAC_HASH_4             		0xA4   	/* MAHTR2 */
#define REG_MAC_HASH_5             		0xA5
#define REG_MAC_HASH_6             		0xA6    	/* MAHTR3 */
#define REG_MAC_HASH_7             		0xA7

/*
 * QMU Receive Queue Watermark Control Registers
 * (Offset 0xB0 - 0xB5)
 */
#define FLOW_CTRL_LOW_WATERMARK     	0xB0 /**< Configure Low Watermark to 6KByte */
#define REG_RX_LOW_WATERMARK       		0xB0   	/* FCLWR */
#define RX_LOW_WATERMARK_MASK       	0x0FFF   /* Set QMU RxQ low watermark mask */

#define FLOW_CTRL_HIGH_WATERMARK    	0xB2 /**< Configure High Watermark to 4KByte */
#define REG_RX_HIGH_WATERMARK      		0xB2     /* FCHWR */
#define RX_HIGH_WATERMARK_MASK      	0x0FFF   /* Set QMU RxQ high watermark mask */

#define REG_RX_OVERRUN_WATERMARK   		0xB4     /* FCOWR */
#define RX_OVERRUN_WATERMARK_MASK  		0x0FFF  	/* Set QMU RxQ overrun watermark mask */

/*
 * Global Control Registers
 * (Offset 0xC0 - 0xD3)
 */
#define CIDER_REG                   	0xC0 /**< Chip ID and Enable Register */
#define REG_CHIP_ID                		0xC0    	/* CIDER */
#define CHIP_ID_MASK                	0xFFF0   /* Family ID and chip ID mask */
#define REVISION_MASK               	0x000E   /* Chip revision mask */
#define CHIP_ID_SHIFT               	4
#define REVISION_SHIFT              	1
#define CHIP_ID_8851_16             	0x8870   /* KS8851-16/32MQL chip ID */

#define REG_LED_CTRL               		0xC6     /* CGCR */
#define LED_CTRL_SEL1            		0x8000   /* Select LED3/LED2/LED1/LED0 indication */
#define LED_CTRL_SEL0               	0x0200   /* Select LED3/LED2/LED1/LED0 indication */

#define IND_ACC_CTRL_REG            	0xC8 /**< Indirect access control Register */
#define REG_IND_IACR               		0xC8   	/* IACR */
#define TABLE_READ                  	0x1000  	/* Indirect read */
#define TABLE_MIB                   	0x0C00  	/* Select MIB counter table */
#define TABLE_ENTRY_MASK            	0x001F  	/* Set table entry to access */

#define IND_ACC_DATA_LOW_REG        	0xD0 /**< Indirect access data low Register */
#define REG_IND_DATA_LOW           		0xD0  	/* IADLR */

#define IND_ACC_DATA_HIGH_REG       	0xD2 /**< Indirect access data low Register */
#define REG_IND_DATA_HIGH          		0xD2   	/* IADHR */

/*
 * Power Management Control Registers
 * (Offset 0xD4 - 0xD7)
 */
#define REG_POWER_CNTL             		0xD4    	/* PMECR */
#define PME_DELAY_ENABLE            	0x4000   /* Enable the PME output pin assertion delay */
#define PME_ACTIVE_HIGHT            	0x1000   /* PME output pin is active high */
#define PME_FROM_WKFRAME            	0x0800   /* PME asserted when wake-up frame is detected */
#define PME_FROM_MAGIC              	0x0400   /* PME asserted when magic packet is detected */
#define PME_FROM_LINKUP             	0x0200 	/* PME asserted when link up is detected */
#define PME_FROM_ENERGY             	0x0100   /* PME asserted when energy is detected */
#define PME_EVENT_MASK              	0x0F00   /* PME asserted event mask */
#define WAKEUP_AUTO_ENABLE          	0x0080   /* Enable auto wake-up in energy mode */
#define WAKEUP_NORMAL_AUTO_ENABLE   	0x0040   /* Enable auto goto normal mode from energy detecion mode */
#define WAKEUP_FROM_WKFRAME         	0x0020   /* Wake-up from wake-up frame event detected */
#define WAKEUP_FROM_MAGIC           	0x0010   /* Wake-up from magic packet event detected */
#define WAKEUP_FROM_LINKUP          	0x0008   /* Wake-up from link up event detected */
#define WAKEUP_FROM_ENERGY          	0x0004   /* Wake-up from energy event detected */
#define WAKEUP_EVENT_MASK           	0x003C   /* Wake-up event mask */
#define POWER_STATE_D1              	0x0003   /* Power saving mode */
#define POWER_STATE_D3              	0x0002   /* Power down mode */
#define POWER_STATE_D2              	0x0001   /* Power detection mode */
#define POWER_STATE_D0              	0x0000   /* Normal operation mode (default) */
#define POWER_STATE_MASK            	0x0003   /* Power management mode mask */

#define REG_WAKEUP_TIME          		0xD6   	/* GSWUTR */
#define WAKEUP_TIME                 	0xFF00   /* Min time (sec) wake-uo after detected energy */
#define GOSLEEP_TIME                	0x00FF   /* Min time (sec) before goto sleep when in energy mode */

/*
 * PHY Control Registers
 * (Offset 0xD8 - 0xF9)
 */
#define PHY_RST_REG                 	0xD8 /**< PHY Reset Register  */
#define REG_PHY_RESET              		0xD8     /* PHYRR */
#define PHY_RESET_REG_VALUE      		0x0001 	/* Reset PHY */

#define PHY1_CTRL_REG               	0xE4 /**< PHY1 MII-Register Basic Control Register */
#define REG_PHY_CNTL               		0xE4     /* P1MBCR */
#define PHY_SPEED_100MBIT           	0x2000   /* Force PHY 100Mbps */
#define PHY_AUTO_NEG_ENABLE         	0x1000   /* Enable PHY auto-negotiation */
#define PHY_POWER_DOWN              	0x0800   /* Set PHY power-down */
#define PHY_AUTO_NEG_RESTART        	0x0200   /* Restart PHY auto-negotiation */
#define PHY_FULL_DUPLEX             	0x0100   /* Force PHY in full duplex mode */
#define PHY_HP_MDIX                 	0x0020   /* Set PHY in HP auto MDI-X mode */
#define PHY_FORCE_MDIX              	0x0010   /* Force MDI-X */
#define PHY_AUTO_MDIX_DISABLE       	0x0008   /* Disable auto MDI-X */
#define PHY_TRANSMIT_DISABLE        	0x0002   /* Disable PHY transmit */
#define PHY_LED_DISABLE             	0x0001  	/* Disable PHY LED */

#define REG_PHY_STATUS             		0xE6     /* P1MBSR */
#define PHY_100BT4_CAPABLE          	0x8000   /* 100 BASE-T4 capable */
#define PHY_100BTX_FD_CAPABLE       	0x4000   /* 100BASE-TX full duplex capable */
#define PHY_100BTX_CAPABLE          	0x2000   /* 100BASE-TX half duplex capable */
#define PHY_10BT_FD_CAPABLE         	0x1000   /* 10BASE-TX full duplex capable */
#define PHY_10BT_CAPABLE            	0x0800   /* 10BASE-TX half duplex capable */
#define PHY_AUTO_NEG_ACKNOWLEDGE    	0x0020   /* Auto-negotiation complete */
#define PHY_AUTO_NEG_CAPABLE        	0x0008   /* Auto-negotiation capable */
#define PHY_LINK_UP                 	0x0004   /* PHY link is up */
#define PHY_EXTENDED_CAPABILITY     	0x0001  	/* PHY extended register capable */

#define REG_PHY_ID_LOW             		0xE8     /* PHY1ILR */
#define REG_PHY_ID_HIGH            		0xEA     /* PHY1IHR */

#define REG_PHY_AUTO_NEGOTIATION   		0xEC     /* P1ANAR */
#define PHY_AUTO_NEG_SYM_PAUSE      	0x0400   /* Advertise pause capability */
#define PHY_AUTO_NEG_100BTX_FD      	0x0100   /* Advertise 100 full-duplex capability */
#define PHY_AUTO_NEG_100BTX         	0x0080   /* Advertise 100 half-duplex capability */
#define PHY_AUTO_NEG_10BT_FD        	0x0040   /* Advertise 10 full-duplex capability */
#define PHY_AUTO_NEG_10BT           	0x0020   /* Advertise 10 half-duplex capability */
#define PHY_AUTO_NEG_SELECTOR       	0x001F   /* Selector field mask */
#define PHY_AUTO_NEG_802_3          	0x0001 	/* 802.3 */

#define REG_PHY_REMOTE_CAPABILITY  		0xEE     /* P1ANLPR */
#define PHY_REMOTE_SYM_PAUSE        	0x0400   /* Link partner pause capability */
#define PHY_REMOTE_100BTX_FD        	0x0100   /* Link partner 100 full-duplex capability */
#define PHY_REMOTE_100BTX           	0x0080   /* Link partner 100 half-duplex capability */
#define PHY_REMOTE_10BT_FD          	0x0040   /* Link partner 10 full-duplex capability */
#define PHY_REMOTE_10BT             	0x0020  	/* Link partner 10 half-duplex capability */

#define REG_PORT_LINK_MD           		0xF4     /* P1SCLMD */
#define PORT_CABLE_10M_SHORT        	0x8000   /* Cable length is less than 10m short */
#define PORT_CABLE_STAT_FAILED      	0x6000   /* Cable diagnostic test fail */
#define PORT_CABLE_STAT_SHORT       	0x4000   /* Short condition detected in the cable */
#define PORT_CABLE_STAT_OPEN        	0x2000   /* Open condition detected in the cable */
#define PORT_CABLE_STAT_NORMAL     	 	0x0000   /* Normal condition */
#define PORT_CABLE_DIAG_RESULT      	0x6000   /* Cable diagnostic test result mask */
#define PORT_START_CABLE_DIAG       	0x1000   /* Enable cable diagnostic test */
#define PORT_FORCE_LINK             	0x0800   /* Enable force link pass */
#define PORT_POWER_SAVING           	0x0400   /* Disable power saving */
#define PORT_REMOTE_LOOPBACK        	0x0200   /* Enable remote loopback at PHY */
#define PORT_CABLE_FAULT_COUNTER    	0x01FF  	/* Cable length distance to the fault */

#define PORT1_CTRL_REG              	0xF6 /**< Port 1 Control Register */
#define REG_PORT_CTRL              		0xF6   	/* P1CR */
#define PORT_LED_OFF                	0x8000   /* Turn off all the port LEDs (LED3/LED2/LED1/LED0) */
#define PORT_TX_DISABLE             	0x4000   /* Disable port transmit */
#define PORT_AUTO_NEG_RESTART      	 	0x2000   /* Restart auto-negotiation */
#define PORT_POWER_DOWN             	0x0800   /* Set port power-down */
#define PORT_AUTO_MDIX_DISABLE      	0x0400   /* Disable auto MDI-X */
#define PORT_FORCE_MDIX             	0x0200   /* Force MDI-X */
#define PORT_AUTO_NEG_ENABLE        	0x0080   /* Enable auto-negotiation */
#define PORT_FORCE_100_MBIT         	0x0040   /* Force PHY 100Mbps */
#define PORT_FORCE_FULL_DUPLEX      	0x0020   /* Force PHY in full duplex mode */
#define PORT_AUTO_NEG_SYM_PAUSE     	0x0010   /* Advertise pause capability */
#define PORT_AUTO_NEG_100BTX_FD     	0x0008   /* Advertise 100 full-duplex capability */
#define PORT_AUTO_NEG_100BTX        	0x0004   /* Advertise 100 half-duplex capability */
#define PORT_AUTO_NEG_10BT_FD       	0x0002   /* Advertise 10 full-duplex capability */
#define PORT_AUTO_NEG_10BT          	0x0001  	/* Advertise 10 half-duplex capability */

#define REG_PORT_STATUS            		0xF8     /* P1SR */
#define PORT_HP_MDIX                	0x8000   /* Set PHY in HP auto MDI-X mode */
#define PORT_REVERSED_POLARITY      	0x2000   /* Polarity is reversed */
#define PORT_RX_FLOW_CTRL           	0x1000   /* Reeive flow control feature is active */
#define PORT_TX_FLOW_CTRL           	0x0800   /* Transmit flow control feature is active */
#define PORT_STAT_SPEED_100MBIT     	0x0400   /* Link is 100Mbps */
#define PORT_STAT_FULL_DUPLEX       	0x0200   /* Link is full duplex mode */
#define PORT_MDIX_STATUS            	0x0080   /* Is MDI */
#define PORT_AUTO_NEG_COMPLETE      	0x0040   /* Auto-negotiation complete */
#define PORT_STATUS_LINK_GOOD       	0x0020   /* PHY link is up */
#define PORT_REMOTE_SYM_PAUSE       	0x0010   /* Link partner pause capability */
#define PORT_REMOTE_100BTX_FD       	0x0008   /* Link partner 100 full-duplex capability */
#define PORT_REMOTE_100BTX          	0x0004   /* Link partner 100 half-duplex capability */
#define PORT_REMOTE_10BT_FD         	0x0002   /* Link partner 10 full-duplex capability */
#define PORT_REMOTE_10BT            	0x0001  	/* Link partner 10 half-duplex capability */

/* Register definitions */
#define MARL      0x10
#define MARM      0x12
#define MARH      0x14
#define OBCR      0x20
#define GRR       0x26
#define TXCR      0x70
#define RXCR1     0x74
#define RXCR2     0x76
#define TXMIR     0x78
#define RXFHSR    0x7C
#define RXFHBCR   0x7E
#define TXQCR     0x80
#define RXQCR     0x82
#define TXFDPR    0x84
#define RXFDPR    0x86
#define IER       0x90
#define ISR       0x92
#define RXFCTR    0x9C
#define TXNTFSR   0x9E
#define FCLWR     0xB0
#define FCHWR     0xB2
#define CIDER     0xC0
#define IACR      0xC8
#define IADLR     0xD0
#define IADHR     0xD2
#define PMECR     0xD4
#define PHYRR     0xD8
#define P1MBCR    0xE4
#define P1CR      0xF6
#define P1SR      0xF8

/* Magic numbers */
#define KSZ8851SNL_CHIP_ID           	0x8870 /**< Default Chip ID for KSZ8851SNL */
#define CHIP_ID_MASK                 	0xFFF0 /**< Used to mask the revision ID */
#define ONE_FRAME_THRES              	0x0001 /**< RX INT after one frame */
#define FD_PTR_AUTO_INC              	0x4000 /**< Used to reset the FD pointer */
#define CLEAR_INT                    	0xFFFF /**< Used to clear INT_STATUS_REG */
#define NO_INT                       	0x0000 /**< Used to disable the interupts */
#define TX_MEM_AVAIL_MASK            	0x1FFF /**< Used to mask the reserved bits */
#define FRAME_ID_MASK                	0x003F /**< Used to mask the reserved bits */
#define CHECKSUM_VALID_FRAME_MASK    	0x3C17 /**< CRC OK for ICMP, IP, TCP, UDP; /
                                             *   MII error; /
                                             *   Frame too long error */
#define RECEIVE_VALID_FRAME_MASK     	0x3C17 /**< CRC OK for ICMP, IP, TCP, UDP; /
                                             *   MII error; /
                                             *   Frame too long error */
#define VALID_FRAME_MASK             	0x8000
#define RX_BYTE_CNT_MASK             	0x0FFF /**< Used to mask the reserved bits */
#define LSB_MASK                     	0x00FF /**< Used to mask the LSB */
#define MSB_POS                      	0x0008 /**< Used to mark the MSB pos */
#define TX_INT_on_COMPLETION         	0x8000 /**< TX INT on completion */
#define WORD_SIZE                    	0x0004 /**< Word size in # of bytes */
#define EXTRA_SIZE                   	0x0008 /**< Needed for the frame header */
#define BLOCKING_RECEIVE             	0      /**< Determines if receive will block */
#define WATERMARK_6KB                	0x0600 /**< 6KByte Watermark */
#define WATERMARK_4KB                	0x0400 /**< 4KByte Watermark */
#define RECEIVED_FRAME_VALID_POS     	0x0010 /**< Received valid frame byte pos */

/* Silicon Laboratories MAC address space */
//#define HIGH_QMU_MAC_H               0x00
//#define HIGH_QMU_MAC_L               0x0B
//#define MID_QMU_MAC_H                0x57
/* Energy Micro's MAC address space */
#define HIGH_QMU_MAC_H             		0xD0   /**< 1st segment of the MAC address */
#define HIGH_QMU_MAC_L              	0xCF   /**< 2nd segment of the MAC address */
#define MID_QMU_MAC_H               	0x5E   /**< 3rd segment of the MAC address */
#define MID_QMU_MAC_L               	0x00   /**< 4th segment of the MAC address */
#define LOW_QMU_MAC_H               	0x00   /**< 5th segment of the MAC address */
#define LOW_QMU_MAC_L               	0x00   /**< 6th segment of the MAC address */
#define BYTE_MASK                   	0x00FF /**< Used to mask the LSB */
#define BYTE_SIZE                   	0x0008 /**< Used to mark the MSB pos */

/* TX Flow Control Register Options */
#define   TX_FLOW_CTRL_ICMP_CHECKSUM    0x0100 /** Enable Transmit Checksum Generation for ICMP */
#define   TX_FLOW_CTRL_UDP_CHECKSUM     0x0080 /** Enable Transmit Checksum Generation for UDP */
#define   TX_FLOW_CTRL_TCP_CHECKSUM     0x0040 /** Enable Transmit Checksum Generation for TCP */
#define   TX_FLOW_CTRL_IP_CHECKSUM      0x0020 /** Enable Transmit Checksum Generation for IP */
#define   TX_FLOW_CTRL_FLUSH_QUEUE      0x0010 /** Flush Transmit Queue */
#define   TX_FLOW_CTRL_FLOW_ENABLE      0x0008 /** Transmit flow control enable*/
#define   TX_FLOW_CTRL_PAD_ENABLE       0x0004 /** Transmit Padding enable */
#define   TX_FLOW_CTRL_CRC_ENABLE       0x0002 /** Transmit CRC Enable */
#define   TX_FLOW_CTRL_ENABLE           0x0001 /** Enable tranmsit */
/** TX FLOW CONTROL Initialization collection */
#define   TX_FLOW_CTRL_CONFIG           (TX_FLOW_CTRL_ICMP_CHECKSUM | \
                                         TX_FLOW_CTRL_UDP_CHECKSUM  |  \
                                         TX_FLOW_CTRL_TCP_CHECKSUM  | \
                                         TX_FLOW_CTRL_IP_CHECKSUM   | \
                                         TX_FLOW_CTRL_FLOW_ENABLE   | \
                                         TX_FLOW_CTRL_PAD_ENABLE    | \
                                         TX_FLOW_CTRL_CRC_ENABLE)

/* TXQ Command Register Options */
#define   TXQ_AUTO_ENQUEUE         		0x0004 /** Enable Auto-Enqueue TXQ Frame */
#define   TXQ_MEM_AVAILABLE_INT    		0x0002 /** Enable INT generation when TXQ Memory Available */
#define   TXQ_ENQUEUE              		0x0001 /** Enable Manual Engueue TXQ Frame */

/* RX Flow Control Register 1 Options */
#define   RX_FLOW_CTRL_FLUSH_QUEUE       0x8000 /** Flush Receive Queue */
#define   RX_FLOW_CTRL_UDP_CHECKSUM      0x4000 /** Enable Receive UDP Frame Checksum Check */
#define   RX_FLOW_CTRL_TCP_CHECKSUM      0x2000 /** Enable Receive TCP Frame Checksum Check */
#define   RX_FLOW_CTRL_IP_CHECKSUM       0x1000 /** Enable Receive IP Frame Checksum Check */
#define   RX_FLOW_CTRL_MAC_FILTER        0x0800 /** Receive Physical Address Filtering with MAC Address Enable */
#define   RX_FLOW_CTRL_FLOW_ENABLE       0x0400 /** Enable Receive Flow Control */
#define   RX_FLOW_CTRL_BAD_PACKET        0x0200 /** Enable Receive Error Frames */
#define   RX_FLOW_CTRL_MULTICAST         0x0100 /** Receive Multicast Address Filtering with MAC Address Enable */
#define   RX_FLOW_CTRL_BROADCAST_ENABLE  0x0080 /** Enable Receive Broadcast frames */
#define   RX_FLOW_CTRL_MULTICAST_ENABLE  0x0040 /** Enable Receive Multicast frames */
#define   RX_FLOW_CTRL_UNICAST_ENABLE    0x0020 /** Enable Receive Unicast frames */
#define   RX_FLOW_CTRL_PROMISCUOUS_MODE  0x0012 /** Receive all incoming frames */
#define   RX_FLOW_CTRL_RX_ALL            0x0010
#define   RX_FLOW_CTRL_INVERSE_FILTER    0x0002 /** Receive Inverse Filtering */
#define   RX_FLOW_CTRL_RX_ENABLE         0x0001 /** Enable receive */
/** RX FLOW CONTROL1 Initialization collection */
#define   RX_FLOW_CTRL1_CONFIG           (RX_FLOW_CTRL_UDP_CHECKSUM     | \
                                          RX_FLOW_CTRL_TCP_CHECKSUM     | \
                                          RX_FLOW_CTRL_IP_CHECKSUM      | \
                                          RX_FLOW_CTRL_MAC_FILTER       | \
                                          RX_FLOW_CTRL_FLOW_ENABLE      | \
                                          RX_FLOW_CTRL_BROADCAST_ENABLE | \
                                          RX_FLOW_CTRL_MULTICAST_ENABLE | \
                                          RX_FLOW_CTRL_UNICAST_ENABLE)

/* RX Flow Control Register 2 Options */
/* SPI Receive Data Burst Length */
#define   RX_FLOW_CTRL_BURST_LEN_MASK        0x00E0 /** Receive Flow Control Burst Length mask */
#define   RX_FLOW_CTRL_BURST_LEN_4           0x0000 /** 4 bytes length */
#define   RX_FLOW_CTRL_BURST_LEN_8           0x0020 /** 8 Bytes length */
#define   RX_FLOW_CTRL_BURST_LEN_16          0x0040 /** 16 Bytes length */
#define   RX_FLOW_CTRL_BURST_LEN_32          0x0060 /** 32 Bytes length */
#define   RX_FLOW_CTRL_BURST_LEN_FRAME       0x0080 /** Full frame length */
#define   RX_FLOW_CTRL_IPV6_UDP_FRAG_PASS    0x0010 /** IPV4/IPV6/UDP Fragment Frame Pass */
#define   RX_FLOW_CTRL_IPV6_UDP_ZERO_PASS    0x0008 /** IPV4/IPV6/UDP Frame Checksum Equal Zero */
#define   RX_FLOW_CTRL_UDP_LITE_CHECKSUM     0x0004 /** Enable UDP Lite frame */
#define   RX_FLOW_CTRL_ICMP_CHECKSUM         0x0002 /** Enable ICMP frame */
#define   RX_FLOW_CTRL_BLOCK_MAC             0x0001 /** Receive Source Address Filtering */
/** RX FLOW CONTROL2 Initialization collection */
#define   RX_FLOW_CTRL2_CONFIG               (RX_FLOW_CTRL_IPV6_UDP_FRAG_PASS | \
                                              RX_FLOW_CTRL_UDP_LITE_CHECKSUM |  \
                                              RX_FLOW_CTRL_ICMP_CHECKSUM |      \
                                              RX_FLOW_CTRL_BURST_LEN_FRAME)

/* RXQ Command Register Options */
#define   RXQ_ON_TIME_INT           		0x1000 /** RX interrupt is occured on timer duration */
#define   RXQ_ON_BYTE_CNT_INT        		0x0800 /** RX interrupt is occured on byte count threshold */
#define   RXQ_ON_FRAME_CNT_INT       		0x0400 /** RX interrupt is occured on frame count threshold */
#define   RXQ_TWOBYTE_OFFSET         		0x0200 /** Enable adding 2-bytes offset before IP frame header */
#define   RXQ_EN_ON_TIME_INT         		0x0080 /** Enable RX interrupt on timer duration */
#define   RXQ_EN_ON_BYTE_CNT_INT     		0x0040 /** Enable RX interrupt on byte count threshold */
#define   RXQ_EN_ON_FRAME_CNT_INT    		0x0020 /** Enable RX interrupt on frame count threshold */
#define   RXQ_AUTO_DEQUEUE           		0x0010 /** Enable Auto Dequeue RXQ Frame */
#define   RXQ_START_DMA              		0x0008 /** Start QMU transfer operation */
#define   RXQ_RELEASE_ERROR_FRAME    		0x0001 /** Release RX Error Frame */
/** RX COMMAND Initialization collection */
#define   RXQ_CMD_CONFIG             	   (RXQ_EN_ON_FRAME_CNT_INT | \
                                      	  	RXQ_AUTO_DEQUEUE)
/** RX COMMAND Initialization collection */
#define   RXQ_CMD_EXAMPLE                  (RXQ_EN_ON_FRAME_CNT_INT | \
                                      	    RXQ_TWOBYTE_OFFSET |      \
											RXQ_AUTO_DEQUEUE)

/* Port 1 Status Register */
#define PORT1_AN_DONE                 		0x0040
#define PORT1_LINK_GOOD               		0x0020

/* Port 1 Control Register Options */
#define   PORT1_LED_OFF               		0x8000 /** Turn off port LEDs */
#define   PORT1_TX_DISABLE            		0x4000 /** Disable port transmit */
#define   PORT1_AUTO_NEG_RESTART      		0x2000 /** Restart auto-negotiation */
#define   PORT1_POWER_DOWN            		0x0800 /** Set port power-down */
#define   PORT1_AUTO_MDIX_DISABLE     		0x0400 /** Disable auto MDI/MDI-X */
#define   PORT1_FORCE_MDIX            		0x0200 /** Force MDI-X */
#define   PORT1_AUTO_NEG_ENABLE       		0x0080 /** Enable auto-negotiation */
#define   PORT1_FORCE_100_MBIT        		0x0040 /** Force PHY 100Mbps */
#define   PORT1_FORCE_FULL_DUPLEX     		0x0020 /** Force PHY in full duplex mode */
#define   PORT1_AUTO_NEG_FLOW_CTRL    		0x0010 /** Advertise flow control capability */
#define   PORT1_AUTO_NEG_SYM_PAUSE    		0x0010 /** Advertise flow control capability */
#define   PORT1_AUTO_NEG_100BTX_FD    		0x0008 /** Advertise 100BT full-duplex capability */
#define   PORT1_AUTO_NEG_100BTX       		0x0004 /** Advertise 100BT half-duplex capability */
#define   PORT1_AUTO_NEG_10BT_FD      		0x0002 /** Advertise 10BT full-duplex capability  */
#define   PORT1_AUTO_NEG_10BT         		0x0001 /** Advertise 10BT half-duplex capability  */
/* Port 1 Control Register Options Initialization collection */
#define   PORT1_CONFIG          	(PORT1_AUTO_NEG_ENABLE    | \
                                    PORT1_FORCE_100_MBIT     | \
                                    PORT1_FORCE_FULL_DUPLEX  | \
                                    PORT1_AUTO_NEG_FLOW_CTRL | \
                                    PORT1_AUTO_NEG_100BTX_FD | \
                                    PORT1_AUTO_NEG_100BTX    | \
                                    PORT1_AUTO_NEG_10BT_FD   | \
                                    PORT1_AUTO_NEG_10BT      )

/* Global Reset Register Options */
#define QMU_MODULE_SOFT_RESET    	0x0002 /** QMU Reset */
#define GLOBAL_SOFT_RESET        	0x0001 /** Global reset */
#define PHY_RESET                	0x0001 /** PHY Reset Register Options */
/* P1MBCR register */
#define DIGITAL_LOOPBACK     		0x4000 /** Enable Digital loopback mode */
#define FORCE_100            		0x2000 /** Force the speed to 100MBps */
#define AUTO_NEG             		0x1000 /** Force auto negotiation */
#define RESTART_AUTO_NEG     		0x0200 /** Restart auto negotiation */
#define FORCE_FULL_DUPLEX    		0x0100 /** Force full duplex */

#define TX_MEMORY_WAIT_MS    		500
#define FRAME_COUNT_THRESHOLD 		1

/* Management information base registers */
#define MIB_MASK                    0x1C00       /**< MIB Mask */
#define MIB_RxByte                  0x00         /**< # of received bytes */
#define MIB_XXX                     0x01         /**< MIB Reserved byte */
#define MIB_RxUndersizePkt          0x02         /**< # of received undersized packets */
#define MIB_RxFragments             0x03         /**< # of received fragments */
#define MIB_RxOversize              0x04         /**< # of received oversized packets */
#define MIB_RxJabbers               0x05         /**< # of received jabbers */
#define MIB_RxSymbolError           0x06         /**< # of received error symbols */
#define MIB_RxCRCError              0x07         /**< # of received packets with CRC error */
#define MIB_RxAlignmentError        0x08         /**< # of received missaligned packets */
#define MIB_RxControl8808Pkts       0x09         /**< # of received control packets */
#define MIB_RxPausePkts             0x0A         /**< # of received pause packets */
#define MIB_RxBroadcast             0x0B         /**< # of received broadcast packets */
#define MIB_RxMulticast             0x0C         /**< # of received multicast packets */
#define MIB_RxUnicast               0x0D         /**< # of received unicast packets */
#define MIB_Rx64Octets              0x0E         /**< # of received packets with size of 64 bytes */
#define MIB_Rx65to127Octets         0x0F         /**< # of received packets with size between 65 and 127 bytes */
#define MIB_Rx128to255Octets        0x10         /**< # of received packets with size between 128 and 255 bytes */
#define MIB_Rx256to511Octets        0x11         /**< # of received packets with size between 256 and 511 bytes */
#define MIB_Rx512to1023Octets       0x12         /**< # of received packets with size between 512 and 1023 bytes */
#define MIB_Rx1024to1521Octets      0x13         /**< # of received packets with size between 1024 and 1521 bytes */
#define MIB_Rx1522to2000Octets      0x14         /**< # of received packets with size between 1522 and 2000 bytes */
#define MIB_TxByte                  0x15         /**< # of transmitted bytes */
#define MIB_TxLateCollision         0x16         /**< # of transmitted late collision packets */
#define MIB_TxPausePkts             0x17         /**< # of transmitted pause packets */
#define MIB_TxBroadcastPkts         0x18         /**< # of transmitted broadcast packets */
#define MIB_TxMulticastPkts         0x19         /**< # of transmitted multicast packets */
#define MIB_TxUnicastPkts           0x1A         /**< # of transmitted unicast packets */
#define MIB_TxDeferred              0x1B         /**< # of transmitted deferred packets */
#define MIB_TxTotalCollision        0x1C         /**< # of transmitted total collisions */
#define MIB_TxExcessiveCollision    0x1D         /**< # of transmitted excessive collisions */
#define MIB_TxSingleCollision       0x1E         /**< # of transmitted single collisions */
#define MIB_TxMultipleCollision     0x1F         /**< # of transmitted multiple collisions */

#define READ_UNSAFE_REGISTERS       0

#ifdef __cplusplus
}
#endif


#endif
