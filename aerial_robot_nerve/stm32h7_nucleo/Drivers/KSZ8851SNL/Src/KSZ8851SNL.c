/*
 * The MIT License (MIT)
 *
 *
 * https://github.com/fabiobaltieri/avr-nrf/blob/master/firmware/ksz8851snl.c
 * https://github.com/marcorussi/iot_eth_server
 * https://siliconlabs.github.io/Gecko_SDK_Doc/efr32mg1/html/ksz8851snl_8c_source.html
 * https://github.com/EnergyMicro/kit_common/blob/master/drivers/ksz8851snl.c
 * https://github.com/avrxml/asf/blob/master/sam/components/ethernet_phy/ksz8851snl/ksz8851snl.c
 * https://www.oryx-embedded.com/doc/ksz8851_8c_source.html
 * https://code.zoetrope.io/bfo/nrf52-freertos/blob/edf7bd8206281b3e5d9c71e39c9c8ed2b0710ce6/lib/FreeRTOS-Plus-TCP/portable/NetworkInterface/ksz8851snl/ksz8851snl.c
 * TODO Check below code
 * https://github.com/atx/avr-uip-2/blob/master/drivers/ksz8851/ksz8851.c
 * https://github.com/Velleman/VM204-Firmware/blob/master/cyclone_tcp/drivers/ksz8851.c
 * https://github.com/RevolutionPi/piControl/blob/master/ksz8851.c
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

/* ------------------- Local inclusions -------------------- */
#include <dmc_print.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <gpio.h>
#include "stm32h7xx_hal.h"
#include "conf_eth.h"
#include "KSZ8851SNL.h"
#include "KSZ8851SNL_reg.h"

/** Network interface identifier. */
#define IFNAME0								'e'
#define IFNAME1								'n'

/** Maximum transfer unit. */
#define NET_MTU								1500

/** Network link speed. */
#define NET_LINK_SPEED						100000000

/* ------------------- Local variables -------------------- */

/**
 * ksz8851snl driver structure.
 */
struct ksz8851snl_device
{
	/** Set to 1 when owner is software (ready to read), 0 for Micrel. */
	uint32_t rx_desc[NETIF_RX_BUFFERS];
	/** Set to 1 when owner is Micrel, 0 for software. */
	uint32_t tx_desc[NETIF_TX_BUFFERS];
	/** RX pbuf pointer list */
	struct pbuf *rx_pbuf[NETIF_RX_BUFFERS];
	/** TX pbuf pointer list */
	struct pbuf *tx_pbuf[NETIF_TX_BUFFERS];

	/** Circular buffer head pointer for packet received. */
	uint32_t us_rx_head;
	/** Circular buffer tail pointer for packet to be read. */
	uint32_t us_rx_tail;
	/** Circular buffer head pointer by upper layer (buffer to be sent). */
	uint32_t us_tx_head;
	/** Circular buffer tail pointer incremented by handlers (buffer sent). */
	uint32_t us_tx_tail;

	/** Reference to lwIP netif structure. */
	struct netif *netif;

#if NO_SYS == 0
/** RX task notification semaphore. */
//	sys_sem_t sync_sem;
#endif
};

/**
 * MAC address to use.
 */
static uint8_t gs_uc_mac_address[] =
{
ETHERNET_CONF_ETHADDR0,
ETHERNET_CONF_ETHADDR1,
ETHERNET_CONF_ETHADDR2,
ETHERNET_CONF_ETHADDR3,
ETHERNET_CONF_ETHADDR4,
ETHERNET_CONF_ETHADDR5 };

typedef struct KSZ8851SLN_mib_s
{
	uint32_t RxByteCnt;
	uint32_t RxUndersizePktCnt;
	uint32_t RxFragmentsCnt;
	uint32_t RxOversizeCnt;
	uint32_t RxJabbersCnt;
	uint32_t RxSymbolErrorCnt;
	uint32_t RxCRCErrorCnt;
	uint32_t RxPausePktsCnt;
	uint32_t RxBroadcastCnt;
	uint32_t RxMulticastCnt;
	uint32_t RxUnicastCnt;
	uint32_t TxByteCnt;
	uint32_t TxPausePktsCnt;
	uint32_t TxBroadcastPktsCnt;
	uint32_t TxMulticastPktsCnt;
	uint32_t TxUnicastPktsCnt;
	uint32_t TxDeferredCnt;
	uint32_t TxTotalCollisionCnt;
} KSZ8851SLN_mib_t;

static struct ksz8851snl_device gs_ksz8851snl_dev;

static uint16_t pending_frame = 0;

static SPI_HandleTypeDef SpiHandle;

static uint16_t rxqcr;
static uint8_t fid;

static uint16_t frameId = 0;
static uint8_t macAddress[6];
static uint16_t rxFrameCount;
static uint32_t interruptCnt = 0;
static KSZ8851SLN_mib_t mibCounters;

/* ------------------- Local functions prototypes -------------------- */
uint8_t ksz8851_interface_init(void);
void ksz8851_hard_reset(void);
void ksz8851_soft_reset(uint8_t queue_only);

/* ------------------- Local functions -------------------- */
//// SPI2_RX
//void DMA1_Stream3_IRQHandler(void)
//{
//	HAL_DMA_IRQHandler(SpiHandle.hdmarx);
//}
//
//// SPI2_TX
//void DMA1_Stream4_IRQHandler(void)
//{
//	HAL_DMA_IRQHandler(SpiHandle.hdmatx);
//}
//
//void SPI2_IRQHandler(void)
//{
//	HAL_SPI_IRQHandler(&SpiHandle);
//}
uint8_t ksz8851_interface_init(void)
{
	bool success = true;

	dmc_puts("ksz8851_interface_init\n");

	/* Configure the SPI peripheral */
	/* Set the SPI parameters */
	SpiHandle.Instance = SPI2;
	SpiHandle.Init.Mode = SPI_MODE_MASTER;
	SpiHandle.Init.Direction = SPI_DIRECTION_2LINES;
	SpiHandle.Init.DataSize = SPI_DATASIZE_8BIT;
	SpiHandle.Init.CLKPolarity = SPI_POLARITY_LOW;
	SpiHandle.Init.CLKPhase = SPI_PHASE_1EDGE;
	SpiHandle.Init.NSS = SPI_NSS_HARD_OUTPUT;	// SPI_NSS_SOFT
	SpiHandle.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;	// KSZ8851SNL can handle up to 50 MHz.
	SpiHandle.Init.FirstBit = SPI_FIRSTBIT_MSB;
	SpiHandle.Init.TIMode = SPI_TIMODE_DISABLE;
	SpiHandle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	SpiHandle.Init.CRCPolynomial = 7;
	SpiHandle.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	SpiHandle.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
	SpiHandle.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
	SpiHandle.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
	SpiHandle.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
	SpiHandle.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
	SpiHandle.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
	SpiHandle.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
	SpiHandle.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
	SpiHandle.Init.IOSwap = SPI_IO_SWAP_DISABLE;
	if (HAL_SPI_Init(&SpiHandle) != HAL_OK)
	{
		success = false;
	}

	return success;
}

void ksz8851_hard_reset(void)
{
	/* Perform hardware reset with respect to the reset timing from the datasheet. */
	HAL_GPIO_WritePin(KSZ8851_RST_GPIO_Port, KSZ8851_RST_Pin, GPIO_PIN_RESET);
	HAL_Delay(2);
	HAL_GPIO_WritePin(KSZ8851_RST_GPIO_Port, KSZ8851_RST_Pin, GPIO_PIN_SET);
	HAL_Delay(2);
}

void ksz8851_soft_reset(uint8_t queue_only)
{
	if (queue_only)
		ksz8851_reg_write_0(REG_RESET_CTRL, QMU_SOFTWARE_RESET);
	else
		ksz8851_reg_write_0(REG_RESET_CTRL, GLOBAL_SOFTWARE_RESET);
	HAL_Delay(1);
	ksz8851_reg_write_0(REG_RESET_CTRL, 0x00);
	HAL_Delay(1);
}

// Required
void ksz8851_set_registers(void)
{
	/* Init step2-4: write QMU MAC address (low, middle then high). */
	ksz8851_reg_write_0(REG_MAC_ADDR_0, (ETHERNET_CONF_ETHADDR4 << 8) | ETHERNET_CONF_ETHADDR5);
	ksz8851_reg_write_0(REG_MAC_ADDR_2, (ETHERNET_CONF_ETHADDR2 << 8) | ETHERNET_CONF_ETHADDR3);
	ksz8851_reg_write_0(REG_MAC_ADDR_4, (ETHERNET_CONF_ETHADDR0 << 8) | ETHERNET_CONF_ETHADDR1);

	/* Init step5: enable QMU Transmit Frame Data Pointer Auto Increment. */
	ksz8851_reg_write_0(REG_TX_ADDR_PTR, ADDR_PTR_AUTO_INC);

	/* Init step6: configure QMU transmit control register. */
	ksz8851_reg_write_0(REG_TX_CTRL,
			TX_CTRL_ICMP_CHECKSUM |
			TX_CTRL_UDP_CHECKSUM |
			TX_CTRL_TCP_CHECKSUM |
			TX_CTRL_IP_CHECKSUM |
			TX_CTRL_FLOW_ENABLE |
			TX_CTRL_PAD_ENABLE |
			TX_CTRL_CRC_ENABLE
		);

	/* Init step7: enable QMU Receive Frame Data Pointer Auto Increment. */
	ksz8851_reg_write_0(REG_RX_ADDR_PTR, ADDR_PTR_AUTO_INC);

	/* Init step8: configure QMU Receive Frame Threshold for one frame. */
	ksz8851_reg_write_0(REG_RX_FRAME_CNT_THRES, 1);

	/* Init step9: configure QMU receive control register1. */
	ksz8851_reg_write_0(REG_RX_CTRL1,
			RX_CTRL_UDP_CHECKSUM |
			RX_CTRL_TCP_CHECKSUM |
			RX_CTRL_IP_CHECKSUM |
			RX_CTRL_MAC_FILTER |
			RX_CTRL_FLOW_ENABLE |
			RX_CTRL_BROADCAST |
			RX_CTRL_ALL_MULTICAST|
			RX_CTRL_UNICAST);
//	ksz8851_reg_write(REG_RX_CTRL1,
//			RX_CTRL_UDP_CHECKSUM |
//			RX_CTRL_TCP_CHECKSUM |
//			RX_CTRL_IP_CHECKSUM |
//			RX_CTRL_FLOW_ENABLE |
//			RX_CTRL_PROMISCUOUS);

	ksz8851_reg_write_0(REG_RX_CTRL2,
			RX_CTRL_IPV6_UDP_NOCHECKSUM |
			RX_CTRL_UDP_LITE_CHECKSUM |
			RX_CTRL_ICMP_CHECKSUM |
			RX_CTRL_BURST_LEN_FRAME);


//#define   RXQ_TWOBYTE_OFFSET          (0x0200)    /* Enable adding 2-byte before frame header for IP aligned with DWORD */
#warning Remember to try the above option to get a 2-byte offset

	/* Init step11: configure QMU receive queue: trigger INT and auto-dequeue frame. */
	ksz8851_reg_write_0( REG_RXQ_CMD, RXQ_CMD_CNTL | RXQ_TWOBYTE_OFFSET );

	/* Init step12: adjust SPI data output delay. */
	ksz8851_reg_write_0(REG_BUS_CLOCK_CTRL, BUS_CLOCK_166 | BUS_CLOCK_DIVIDEDBY_1);

	/* Init step13: restart auto-negotiation. */
	ksz8851_reg_setbits(REG_PORT_CTRL, PORT_AUTO_NEG_RESTART);

	/* Init step13.1: force link in half duplex if auto-negotiation failed. */
	if ((ksz8851_reg_read(REG_PORT_CTRL) & PORT_AUTO_NEG_RESTART) != PORT_AUTO_NEG_RESTART)
	{
		ksz8851_reg_clrbits(REG_PORT_CTRL, PORT_FORCE_FULL_DUPLEX);
	}

	/* Init step14: clear interrupt status. */
	ksz8851_reg_write_0(REG_INT_STATUS, 0xFFFF);

	/* Init step15: set interrupt mask. */
	ksz8851_reg_write_0(REG_INT_MASK, INT_RX);

	/* Init step16: enable QMU Transmit. */
	ksz8851_reg_setbits(REG_TX_CTRL, TX_CTRL_ENABLE);

	/* Init step17: enable QMU Receive. */
	ksz8851_reg_setbits(REG_RX_CTRL1, RX_CTRL_ENABLE);
}

// Required
uint32_t ksz8851_reinit(void)
{
	uint32_t count = 10;
	uint16_t dev_id = 0;
	uint8_t id_ok = 0;

	/* Reset the Micrel in a proper state. */
	while( count-- )
	{
		/* Perform hardware reset with respect to the reset timing from the datasheet. */
		ksz8851_hard_reset();

		/* Init step1: read chip ID. */
		dev_id = ksz8851_reg_read(REG_CHIP_ID);
		if( ( dev_id & 0xFFF0 ) == CHIP_ID_8851_16 )
		{
			id_ok = 1;
			break;
		}
	}
	if( id_ok != 0 )
	{
		ksz8851_set_registers();
	}

	return id_ok ? 1 : -1;
}

void ksz8851_set_pm(uint8_t mode)
{
	uint8_t pmecr;

	pmecr = ksz8851_reg_read(REG_POWER_CNTL);
	pmecr &= ~POWER_STATE_MASK;
	pmecr |= mode;
	ksz8851_reg_write_0(REG_POWER_CNTL, pmecr);
}

// https://github.com/fabiobaltieri/avr-nrf/blob/master/firmware/ksz8851snl.c
void ksz8851_send_packet(uint8_t *buf, uint16_t length)
{
	ksz8851_reg_write_0(REG_RXQ_CMD, rxqcr | RXQ_START);
	ksz8851_fifo_write(buf, length);
	ksz8851_reg_write_0(REG_RXQ_CMD, rxqcr);
	ksz8851_reg_write_0(REG_TXQ_CMD, RXQ_CMD_FREE_PACKET);

//	led_net_tx();
}

uint16_t ksz8851_read_packet(uint8_t *buf, uint16_t limit)
{
	uint16_t rxlen;
	uint16_t rxfctr;
	uint16_t ret = 0;
	uint8_t i;

	rxfctr = ksz8851_reg_read(KS_RXFCTR);

	rxfctr = rxfctr >> 8;
	for (i = 0; i < rxfctr; i++)
	{
		rxlen = ksz8851_reg_read(REG_RX_FHR_BYTE_CNT);

		ksz8851_reg_write_0(REG_RX_ADDR_PTR, ADDR_PTR_AUTO_INC | 0x00);
		ksz8851_reg_write_0(REG_RXQ_CMD, rxqcr | RXQ_START | RXQ_AUTO_DEQUEUE);

		if (rxlen > 4)
		{
			if (i == 0 && rxlen <= limit)
			{
				rxlen -= rxlen % 4;
				ksz8851_fifo_read(buf, rxlen);
				ret = rxlen;
			}
			else
			{
				rxlen -= rxlen % 4;
				ksz8851_fifo_read(NULL, rxlen);
				ret = 0;
			}
		}

		ksz8851_reg_write_0(REG_RXQ_CMD, rxqcr);
	}

//	led_net_rx();

	return ret;
}

uint8_t ksz8851_has_data(void)
{
	uint16_t isr;

	isr = ksz8851_reg_read(REG_INT_STATUS);

	if (isr & IRQ_RXI)
	{
		ksz8851_reg_write_0(REG_INT_STATUS, isr);
		return 1;
	}
	else
	{
		ksz8851_reg_write_0(REG_INT_STATUS, isr);
		return 0;
	}
}

void ksz8851_irq(void)
{
	uint16_t isr;

	isr = ksz8851_reg_read(REG_INT_STATUS);

	if (isr & IRQ_LCI)
	{
		;
	}
	if (isr & IRQ_LDI)
	{
		;
	}
	if (isr & IRQ_RXPSI)
	{
		;
	}
	if (isr & IRQ_TXI)
	{
//		led_net_tx();
	}
	if (isr & IRQ_RXI)
	{
//		led_net_rx();
	}
	if (isr & IRQ_SPIBEI)
	{
		;
	}

	ksz8851_reg_write_0(REG_INT_STATUS, isr);

	if (isr & IRQ_RXI)
	{
		;
	}
}

/* ------------------- Exported functions -------------------- */
uint8_t ksz8851_get_spi_state(SPI_HandleTypeDef *hspi)
{
	HAL_SPI_StateTypeDef spiState;
	uint8_t spiIntCode;

	spiState = HAL_SPI_GetState(hspi);

	switch (spiState)
	{
	case HAL_SPI_STATE_READY:
	{
		spiIntCode = INT_SPI_READY;
		break;
	}
	case HAL_SPI_STATE_BUSY:
	case HAL_SPI_STATE_BUSY_TX:
	case HAL_SPI_STATE_BUSY_RX:
	case HAL_SPI_STATE_BUSY_TX_RX:
	{
		spiIntCode = INT_SPI_BUSY;
		break;
	}
	case HAL_SPI_STATE_ERROR:
	default:
	{
		//TODO: get and understand the error code
		//error_code = HAL_SPI_GetError(hspi);
		spiIntCode = INT_SPI_ERROR;
	}
	}

	return spiIntCode;
}

// Required
void ksz8851_fifo_read(uint8_t *buf, uint16_t len)
{
	uint8_t inbuf[9];
	uint8_t pad_bytes;

	//TODO: check len value

//	RESET_SPI_CS_PIN();
//	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
//	HAL_Delay(2);

	/* calculate number of dummy pad bytes to read a 32-bits aligned buffer */
	pad_bytes = ((len & 0x03) != 0) ? (4 - (len & 0x03)) : 0;

	inbuf[0] = FIFO_READ;

	/* Perform blocking SPI transfer. */
	(void) HAL_SPI_TransmitReceive(&SpiHandle, (uint8_t*) inbuf, (uint8_t*) inbuf, 9, 2000);

	/* update length to a 32-bits aligned value */
	len += pad_bytes;

	/* Perform non-blocking DMA SPI transfer. Discard function returned value! TODO: handle it? */
	(void) HAL_SPI_TransmitReceive_DMA(&SpiHandle, (uint8_t*) buf, (uint8_t*) buf, len);
	/* an interrupt will occur */

	/* Perform blocking SPI transfer. Discard function returned value! TODO: handle it? */
//	(void) HAL_SPI_TransmitReceive(&SpiHandle, (uint8_t*) buf, (uint8_t*) buf, len, 1000);
}

// Required
void ksz8851_fifo_write(uint8_t *buf, uint16_t len)
{
	uint8_t outbuf[5];
	uint8_t pad_bytes;
	static uint8_t frameID = 0;

//	RESET_SPI_CS_PIN();
//	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
//	HAL_Delay(2);

	//TODO: check len value

	/* length is 11 bits long */
	len &= 0x07FF;
	/* calculate number of dummy pad bytes to send a 32-bits aligned buffer */
	pad_bytes = ((len & 0x03) != 0) ? (4 - (len & 0x03)) : 0;

	/* Prepare control word and byte count. */
	outbuf[0] = FIFO_WRITE;
	outbuf[1] = frameID++ & 0x3f;
	outbuf[2] = 0;
	outbuf[3] = len & 0xff;
	outbuf[4] = len >> 8;

	/* Perform blocking SPI transfer. */
	(void) HAL_SPI_Transmit(&SpiHandle, (uint8_t*) outbuf, 5, 2000);

	/* update length to a 32-bits aligned value */
	len += pad_bytes;

	/* ATTENTION: pad bytes are the bytes beyond buffer length (can be any rubbish value) */
	/* Perform non-blocking DMA SPI transfer. Discard function returned value! TODO: handle it? */
	(void) HAL_SPI_TransmitReceive_DMA(&SpiHandle, (uint8_t*) buf, (uint8_t*) buf, len);
	/* an interrupt will occur */

	/* Perform blocking SPI transfer. Discard function returned value! TODO: handle it? */
//	(void) HAL_SPI_TransmitReceive(&SpiHandle, (uint8_t*) buf, (uint8_t*) buf, len, 1000);
}

// Required
uint16_t ksz8851_reg_read(uint16_t reg)
{
	uint8_t inbuf[4];
	uint8_t outbuf[4];
	uint16_t cmd = 0;
	uint16_t res = 0;

//	RESET_SPI_CS_PIN();
//	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
//	HAL_Delay(2);

	/* Move register address to cmd bits 9-2, make 32-bit address. */
	cmd = (reg << 2) & REG_ADDR_MASK;

	/* Last 2 bits still under "don't care bits" handled with byte enable. */
	/* Select byte enable for command. */
	if (reg & 2)
	{
		/* Odd word address writes bytes 2 and 3 */
		cmd |= (0xc << 10);
	}
	else
	{
		/* Even word address write bytes 0 and 1 */
		cmd |= (0x3 << 10);
	}

	/* Add command read code. */
	cmd |= CMD_READ;
	outbuf[0] = cmd >> 8;
	outbuf[1] = cmd & 0xff;
	outbuf[2] = 0xff;
	outbuf[3] = 0xff;

	/* Perform blocking SPI transfer. Discard function returned value! TODO: handle it? */
	(void) HAL_SPI_TransmitReceive(&SpiHandle, (uint8_t*) outbuf, (uint8_t *) inbuf, 4, 2000);

//	SET_SPI_CS_PIN();
//	HAL_Delay(2);
//	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);

	res = (inbuf[3] << 8) | inbuf[2];
	return res;
}

// Required
void ksz8851_reg_write_0(uint16_t reg, uint16_t wrdata)
{
	uint8_t outbuf[4];
	uint16_t cmd = 0;

//	RESET_SPI_CS_PIN();
//	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
//	HAL_Delay(2);

	/* Move register address to cmd bits 9-2, make 32-bit address. */
	cmd = (reg << 2) & REG_ADDR_MASK;

	/* Last 2 bits still under "don't care bits" handled with byte enable. */
	/* Select byte enable for command. */
	if (reg & 2)
	{
		/* Odd word address writes bytes 2 and 3 */
		cmd |= (0xc << 10);
	}
	else
	{
		/* Even word address write bytes 0 and 1 */
		cmd |= (0x3 << 10);
	}

	/* Add command write code. */
	cmd |= CMD_WRITE;
	outbuf[0] = cmd >> 8;
	outbuf[1] = cmd & 0xff;
	outbuf[2] = wrdata & 0xff;
	outbuf[3] = wrdata >> 8;

	/* Perform blocking SPI transfer. Discard function returned value! TODO: handle it? */
	(void) HAL_SPI_Transmit(&SpiHandle, (uint8_t*) outbuf, 4, 2000);

//	SET_SPI_CS_PIN();
//	HAL_Delay(2);
//	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
}

// In order to set a bit in a register, such as during initialization,
// read the register first, modify the target bit only and write it back.
/**
 * \brief Read register content, set bitmask and write back to register.
 *
 * \param reg the register address to modify.
 * \param bits_to_set bitmask to apply.
 */
// Required
void ksz8851_reg_setbits(uint16_t reg, uint16_t bits_to_set)
{
	uint16_t temp;

	temp = ksz8851_reg_read(reg);
	temp |= bits_to_set;
	ksz8851_reg_write_0(reg, temp);
}

/**
 * \brief Read register content, clear bitmask and write back to register.
 *
 * \param reg the register address to modify.
 * \param bits_to_set bitmask to apply.
 */
// Required
void ksz8851_reg_clrbits(uint16_t reg, uint16_t bits_to_clr)
{
	uint16_t temp;

	temp = ksz8851_reg_read(reg);
	temp &= ~(uint32_t) bits_to_clr;
	ksz8851_reg_write_0(reg, temp);
}

// Required
uint8_t ksz8851_init(void)
{
	uint32_t count = 0;
	uint16_t dev_id = 0;
	bool success = true;

	dmc_puts("ksz8851_init\n");

	/* Initialize the SPI interface. */
	if ( true == ksz8851_interface_init())
	{
		dmc_puts("Reset the ksz8851 in a proper state.\n");
		/* Reset the Micrel in a proper state. */
		do
		{
			ksz8851_hard_reset();

			/* Init step1: read chip ID. */
			dev_id = ksz8851_reg_read(REG_CHIP_ID);
			dmc_puts("dev_id: ");
			dmc_puthex2(dev_id);
			dmc_puts(" (");
			dmc_puthex2(dev_id & 0xFFF0);
			dmc_puts(")\n");
			if (++count > 10)
			{
				return 1;
			}
		}
		while ((dev_id & 0xFFF0) != CHIP_ID_8851_16);

		/* Init step2-4: write QMU MAC address (low, middle then high). */
		ksz8851_reg_write_0(REG_MAC_ADDR_0, (ETHERNET_CONF_ETHADDR4 << 8) | ETHERNET_CONF_ETHADDR5); /* MARL */
		ksz8851_reg_write_0(REG_MAC_ADDR_2, (ETHERNET_CONF_ETHADDR2 << 8) | ETHERNET_CONF_ETHADDR3); /* MARM */
		ksz8851_reg_write_0(REG_MAC_ADDR_4, (ETHERNET_CONF_ETHADDR0 << 8) | ETHERNET_CONF_ETHADDR1); /* MARH */

		uint16_t addr_0 = ksz8851_reg_read(REG_MAC_ADDR_0);
		uint16_t addr_2 = ksz8851_reg_read(REG_MAC_ADDR_2);
		uint16_t addr_4 = ksz8851_reg_read(REG_MAC_ADDR_4);
		dmc_puts("MAC_ADDR: ");
		dmc_puthex2(addr_4 >> 8);
		dmc_puts(".");
		dmc_puthex2(addr_4 & 0xff);
		dmc_puts(".");
		dmc_puthex2(addr_2 >> 8);
		dmc_puts(".");
		dmc_puthex2(addr_2 & 0xff);
		dmc_puts(".");
		dmc_puthex2(addr_0 >> 8);
		dmc_puts(".");
		dmc_puthex2cr(addr_0 & 0xff);

		/* Init step5: enable QMU Transmit Frame Data Pointer Auto Increment. */
		ksz8851_reg_write_0(REG_TX_ADDR_PTR, ADDR_PTR_AUTO_INC);
		uint16_t TX_ADDR_PTR = ksz8851_reg_read(REG_TX_ADDR_PTR);
		dmc_puts("TX_ADDR_PTR: ");
		dmc_puthex4cr(TX_ADDR_PTR);

		/* Init step6: configure QMU transmit control register. */
		ksz8851_reg_write_0(REG_TX_CTRL,
				TX_CTRL_ICMP_CHECKSUM |
				TX_CTRL_UDP_CHECKSUM |
				TX_CTRL_TCP_CHECKSUM |
				TX_CTRL_IP_CHECKSUM |
				TX_CTRL_FLOW_ENABLE |
				TX_CTRL_PAD_ENABLE |
				TX_CTRL_CRC_ENABLE);

		/* Init step7: enable QMU Receive Frame Data Pointer Auto Increment. */
		ksz8851_reg_write_0(REG_RX_ADDR_PTR, ADDR_PTR_AUTO_INC);
		uint16_t RX_ADDR_PTR = ksz8851_reg_read(REG_RX_ADDR_PTR);
		dmc_puts("RX_ADDR_PTR: ");
		dmc_puthex4cr(RX_ADDR_PTR);

		/* Init step8: configure QMU Receive Frame Threshold for one frame. */
		ksz8851_reg_write_0(REG_RX_FRAME_CNT_THRES, 1);

		/* Init step9: configure QMU receive control register1. */
		ksz8851_reg_write_0(REG_RX_CTRL1,
				RX_CTRL_UDP_CHECKSUM |
				RX_CTRL_TCP_CHECKSUM |
				RX_CTRL_IP_CHECKSUM |
				RX_CTRL_MAC_FILTER |
				RX_CTRL_FLOW_ENABLE |
				RX_CTRL_BROADCAST |
				RX_CTRL_ALL_MULTICAST |
				RX_CTRL_UNICAST |
				RX_CTRL_PROMISCUOUS);

		/* Init step10: configure QMU receive control register2. */
		ksz8851_reg_write_0(REG_RX_CTRL2,
				RX_CTRL_IPV6_UDP_NOCHECKSUM |
				RX_CTRL_UDP_LITE_CHECKSUM |
				RX_CTRL_ICMP_CHECKSUM |
				RX_CTRL_BURST_LEN_FRAME);

		/* Init step11: configure QMU receive queue: trigger INT and auto-dequeue frame. */
		ksz8851_reg_write_0(REG_RXQ_CMD, RXQ_CMD_CNTL | RXQ_TWOBYTE_OFFSET);

		/* Init step12: adjust SPI data output delay. */
		ksz8851_reg_write_0(REG_BUS_CLOCK_CTRL, BUS_CLOCK_166 | BUS_CLOCK_DIVIDEDBY_1);

		/* Init step13: restart auto-negotiation. */
		ksz8851_reg_setbits(REG_PORT_CTRL, PORT_AUTO_NEG_RESTART);

		/* Init step13.1: force link in half duplex if auto-negotiation failed. */
		if ((ksz8851_reg_read(REG_PORT_CTRL) & PORT_AUTO_NEG_RESTART) != PORT_AUTO_NEG_RESTART)
		{
			ksz8851_reg_clrbits(REG_PORT_CTRL, PORT_FORCE_FULL_DUPLEX);
			dmc_puts("Full Duplex\n");
		}
		else
		{
			dmc_puts("Half Duplex\n");
		}

		/* Init step14: clear interrupt status. */
		ksz8851_reg_write_0(REG_INT_STATUS, 0xFFFF);

		/* Init step15: set interrupt mask. */
		ksz8851_reg_write_0(REG_INT_MASK, INT_RX);

		/* Init step16: enable QMU Transmit. */
		ksz8851_reg_setbits(REG_TX_CTRL, TX_CTRL_ENABLE);

		/* Init step17: enable QMU Receive. */
		ksz8851_reg_setbits(REG_RX_CTRL1, RX_CTRL_ENABLE);
		dmc_puts("OK\n");
	}
	else
	{
		dmc_puts("FAIL\n");
		success = false;
	}

	return success;
}

void ksz8851_init_alt(void)
{
	fid = 0;

//	eth_cs_h();

	ksz8851_soft_reset(0);

	/* set MAC address */
	ksz8851_reg_write_0(REG_MAC_ADDR_4, 0x0021);
	ksz8851_reg_write_0(REG_MAC_ADDR_2, 0xf300);
	ksz8851_reg_write_0(REG_MAC_ADDR_0, 0x8851);

	/* PM NORMAL */
	ksz8851_set_pm(POWER_STATE_D0);

	/* QMU reset */
	ksz8851_soft_reset(1);

	/* TX parameters */
	ksz8851_reg_write_0(REG_TX_CTRL, (TX_CTRL_ENABLE | /* enable transmit process */
	TX_CTRL_PAD_ENABLE | /* pad to min length */
	TX_CTRL_CRC_ENABLE | /* add CRC */
	TX_CTRL_FLOW_ENABLE)); /* enable flow control */

	/* auto-increment tx data, reset tx pointer */
	ksz8851_reg_write_0(REG_TX_ADDR_PTR, TXFDPR_TXFPAI);

	/* RX parameters */
	ksz8851_reg_write_0(REG_RX_CTRL1, (RX_CTRL_MAC_FILTER | /*  from mac filter */
	RX_CTRL_FLOW_ENABLE | /* enable flow control */
	RX_CTRL_BROADCAST | /* broadcast enable */
	RX_CTRL_UNICAST | /* unicast enable */
	RX_CTRL_ENABLE)); /* enable rx block */

	/* transfer entire frames out in one go */
	ksz8851_reg_write_0(REG_RX_CTRL2, RXCR2_SRDBL_FRAME);

	/* set receive counter timeouts */
	ksz8851_reg_write_0(REG_RX_TIME_THRES, 1000); /* 1ms after first frame to IRQ */
	ksz8851_reg_write_0(REG_RX_BYTE_CNT_THRES, 4096); /* >4Kbytes in buffer to IRQ */
	ksz8851_reg_write_0(REG_RX_FRAME_CNT_THRES, 10); /* 10 frames to IRQ */

	rxqcr = RXQ_FRAME_CNT_INT | /* IRQ on frame count exceeded */
	RXQ_BYTE_CNT_INT | /* IRQ on byte count exceeded */
	RXQ_TIME_INT; /* IRQ on time exceeded */

	ksz8851_reg_write_0(REG_RXQ_CMD, rxqcr);

	ksz8851_reg_write_0(REG_INT_STATUS, (IRQ_LCI | /* Link Change */
	IRQ_TXI | /* TX done */
	IRQ_RXI | /* RX done */
	IRQ_SPIBEI | /* SPI bus error */
	IRQ_TXPSI | /* TX process stop */
	IRQ_RXPSI));

	ksz8851_reg_write_0(REG_INT_MASK, (IRQ_LCI | /* Link Change */
	IRQ_TXI | /* TX done */
	IRQ_RXI | /* RX done */
	IRQ_SPIBEI | /* SPI bus error */
	IRQ_TXPSI | /* TX process stop */
	IRQ_RXPSI));
}

/***************************************************************************//**
 * @brief Initialize the registers of the ethernet controller.
 *****************************************************************************/
void ksz8851_init_alt2(void)
{
  uint16_t data;

  /* Initialize SPI Interface */
//  ETHSPI_Init();

  /* Reset Soft (clear registers of PHY, MAC, QMU, DMA) */
  data = GLOBAL_SOFT_RESET;
  ksz8851_reg_write_0(GLOBAL_RESET_REG, data);
  data = ksz8851_reg_read(GLOBAL_RESET_REG);
  data &= ~GLOBAL_SOFT_RESET;
  ksz8851_reg_write_0(GLOBAL_RESET_REG, data);

  /* Reset QMU Modules(flush out TXQ and RXQ) */
  data = QMU_MODULE_SOFT_RESET;
  ksz8851_reg_write_0(GLOBAL_RESET_REG, data);
  data = ksz8851_reg_read(GLOBAL_RESET_REG);
  data &= ~QMU_MODULE_SOFT_RESET;
  ksz8851_reg_write_0(GLOBAL_RESET_REG, data);

#ifdef DIGITAL_PHY_LOOPBACK
  KSZ8851SNL_SetDigitalLoopbackMode();
#endif /* DIGITAL_PHY_LOOPBACK */

  /* Read the chip ID and check if that is correct */
  data = ksz8851_reg_read(CIDER_REG);

  /* The CIDER lower bits [3..1] are defined as revision number,
   *   thus a mask needs to be applied
   */
  if ((data & CHIP_ID_MASK) != KSZ8851SNL_CHIP_ID)
  {
//    KSZ8851SNL_ExceptionHandler(ERROR, "ETH: Incorrect Device ID");
  }

  /* Write the Queue Management Unit MAC Address */
//  KSZ8851SNL_GetMacAddress(macAddress);
  /* Write the appropriate KSZ8851SNL MAC registers
   *   starting from the HIGH part towards the lower one
   *   going with a step of 2
   */
  int i;
  for (i = 0; (i < 6); i += 2)
  {
    data = (macAddress[i] << MSB_POS) | macAddress[i + 1];
    ksz8851_reg_write_0(HIGH_QMU_MAC_REG - i, data);
  }

  /* Enable QMU Transmit Frame Data Pointer Auto Increment */
  data = FD_PTR_AUTO_INC;
  ksz8851_reg_write_0(TX_FD_PTR_REG, data);

  /* FLUSH TX queue */
  data |= TX_FLOW_CTRL_FLUSH_QUEUE;
  ksz8851_reg_write_0(TX_FLOW_CTRL_REG, data);

  /* Enable QMU Transmit:
   *  flow control,
   *  padding,
   *  CRC,
   *  IP/TCP/UDP/ICMP checksum generation.
   */
  data = TX_FLOW_CTRL_CONFIG;
  ksz8851_reg_write_0(TX_FLOW_CTRL_REG, data);

  /* Enable QMU Receive Frame Data Pointer Auto Increment */
  data = FD_PTR_AUTO_INC;
  ksz8851_reg_write_0(RX_FD_PTR_REG, data);

  /* Configure Receive Frame Threshold for one frame */
  data = ONE_FRAME_THRES;
  ksz8851_reg_write_0(RX_FRAME_THRES_REG, data);

  /* Enable QMU Receive:
   *  flow control,
   *  receive all broadcast frame,
   *  receive unicast frame,
   *  IP/TCP/UDP/ICMP checksum generation.
   */
  data = RX_FLOW_CTRL1_CONFIG;
  ksz8851_reg_write_0(RX_FLOW_CTRL1_REG, data);

  /* Enable QMU Receive:
   *  ICMP/UDP Lite frame checksum verification,
   *  UDP Lite frame checksum generation,
   *  IPv6 UDP fragment frame pass.
   */
  data = RX_FLOW_CTRL2_CONFIG;
  ksz8851_reg_write_0(RX_FLOW_CTRL2_REG, data);

  /* Enable QMU Receive:
   *  IP Header Two-Byte Offset,
   *  Receive Frame Count Threshold,
   *  RXQ Auto-Dequeue frame.
   */
  data = RXQ_CMD_CONFIG;
  ksz8851_reg_write_0(RXQ_CMD_REG, data);

  /* Restart Port 1 auto-negotiation */
  data = ksz8851_reg_read(PORT1_CTRL_REG);
  data |= PORT1_AUTO_NEG_RESTART;
  ksz8851_reg_write_0(PORT1_CTRL_REG, data);

  /* Force link in half duplex if auto-negotiation failed  */
  data = ksz8851_reg_read(PORT1_CTRL_REG);
  if ((data & PORT1_AUTO_NEG_RESTART) != PORT1_AUTO_NEG_RESTART)
  {
    data &= ~PORT1_FORCE_FULL_DUPLEX;
    ksz8851_reg_write_0(PORT1_CTRL_REG, data);
  }
  /* Configure Low Watermark to 6KByte available buffer space out of 12KByte */
  data = WATERMARK_6KB;
  ksz8851_reg_write_0(FLOW_CTRL_LOW_WATERMARK, data);

  /* Configure High Watermark to 4KByte available buffer space out of 12KByte */
  data = WATERMARK_4KB;
  ksz8851_reg_write_0(FLOW_CTRL_HIGH_WATERMARK, data);

  /* Clear the interrupts status */
  data = CLEAR_INT;
  ksz8851_reg_write_0(INT_STATUS_REG, data);

  /* Enable Interrupts on:
   *  Link Change
   *  Transmit
   *  Receive
   *  Receive Overrun
   *  Transmit Process Stop
   *  Receive Process Stop
   */
  data = INT_MASK_EXAMPLE;
  ksz8851_reg_write_0(INT_ENABLE_REG, data);

  /* Enable QMU Transmit */
  data = ksz8851_reg_read(TX_FLOW_CTRL_REG);
  data |= TX_FLOW_CTRL_ENABLE;
  ksz8851_reg_write_0(TX_FLOW_CTRL_REG, data);

  /* Enable QMU Receive */
  data = ksz8851_reg_read(RX_FLOW_CTRL1_REG);
  data |= RX_FLOW_CTRL_RX_ENABLE;
  ksz8851_reg_write_0(RX_FLOW_CTRL1_REG, data);

//  KSZ8851SNL_ExceptionHandler(INFO, "ETH: Initialization complete");
}

uint32_t ksz8851_MIBCountersRead(uint16_t offset)
{
	uint16_t data;
	uint32_t counter;

	data = MIB_MASK | offset;
	ksz8851_reg_write_0(REG_IND_IACR, data);
	counter = ksz8851_reg_read(REG_IND_DATA_LOW);
	counter |= ksz8851_reg_read(REG_IND_DATA_HIGH) << 16;

	return counter;
}

/***************************************************************************//**
 * @brief Dumps the Management Information Base Counters
 * @note  Support method used for debugging.
 *
 * @param param
 *     the string representing the moment of the register dump
 *
 *****************************************************************************/
void KSZ8851SNL_ReadMIBCounters(char* param)
{
	EFM_ASSERT(param != NULL);
	uint16_t data;
	uint32_t counter;
	dmc_puts("Dumping MIB Counters values @");
	dmc_puts(param);
	dmc_puts("\n");
	int i;
	for (i = 0; i < 0x20; i++)
	{
		data = MIB_MASK | i;
		ksz8851_reg_write_0(REG_IND_IACR, data);
		counter = ksz8851_reg_read(REG_IND_DATA_LOW);
		counter |= ksz8851_reg_read(REG_IND_DATA_HIGH) << 16;
		dmc_puts("MIB_REG[");
		dmc_puthex2(i);
		dmc_puts("] contains ");
		dmc_puthex4(counter);
		dmc_puts("\n");
	}
}

void ksz8851_AllRegistersDump(void)
{
	dmc_puts("###################### ALL REGISTER DUMP ########################\n");
	int i;
	for (i = 0x00; i < 0xFF; i += 0x02)
	{
		if ((i % 8 == 0) && (i > 0))
		{
			dmc_puts("\n");
		}
		dmc_puts("REG[0x");
		dmc_puthex2(i);
		dmc_puts("]=0x");
		dmc_puthex4cr(ksz8851_reg_read(i));
	}
	dmc_puts("\n");
	dmc_puts("#################################################################\n");
}

void ksz8851_RegistersDump(void)
{
	dmc_puts("##################### SPECIAL REGISTER DUMP ######################\n");
//	printf("MARL  [0x%02X]=0x%04X\n", REG_MAC_ADDR_0, ksz8851_reg_read(REG_MAC_ADDR_0));
	dmc_puts("MARL  [0x");
	dmc_puthex2(REG_MAC_ADDR_0);
	dmc_puts("]=0x");
	dmc_puthex4cr(ksz8851_reg_read(REG_MAC_ADDR_0));
//	printf("MARM  [0x%02X]=0x%04X\n", REG_MAC_ADDR_2, ksz8851_reg_read(REG_MAC_ADDR_2));
	dmc_puts("MARM  [0x");
	dmc_puthex2(REG_MAC_ADDR_2);
	dmc_puts("]=0x");
	dmc_puthex4cr(ksz8851_reg_read(REG_MAC_ADDR_2));
//	printf("MARH  [0x%02X]=0x%04X\n", REG_MAC_ADDR_4, ksz8851_reg_read(REG_MAC_ADDR_4));
	dmc_puts("MARH  [0x");
	dmc_puthex2(REG_MAC_ADDR_4);
	dmc_puts("]=0x");
	dmc_puthex4cr(ksz8851_reg_read(REG_MAC_ADDR_4));
//	printf("OBCR  [0x%02X]=0x%04X\n", REG_BUS_CLOCK_CTRL, ksz8851_reg_read(REG_BUS_CLOCK_CTRL));
	dmc_puts("OBCR  [0x");
	dmc_puthex2(REG_BUS_CLOCK_CTRL);
	dmc_puts("]=0x");
	dmc_puthex4cr(ksz8851_reg_read(REG_BUS_CLOCK_CTRL));
//	printf("GRR   [0x%02X]=0x%04X\n", REG_RESET_CTRL, ksz8851_reg_read(REG_RESET_CTRL));
	dmc_puts("GRR   [0x");
	dmc_puthex2(REG_RESET_CTRL);
	dmc_puts("]=0x");
	dmc_puthex4cr(ksz8851_reg_read(REG_RESET_CTRL));
//	printf("TXCR  [0x%02X]=0x%04X\n", REG_TX_CTRL, ksz8851_reg_read(REG_TX_CTRL));
	dmc_puts("TXCR  [0x");
	dmc_puthex2(REG_TX_CTRL);
	dmc_puts("]=0x");
	dmc_puthex4cr(ksz8851_reg_read(REG_TX_CTRL));
//	printf("RXCR1 [0x%02X]=0x%04X\n", REG_RX_CTRL1, ksz8851_reg_read(REG_RX_CTRL1));
	dmc_puts("RXCR1 [0x");
	dmc_puthex2(REG_RX_CTRL1);
	dmc_puts("]=0x");
	dmc_puthex4cr(ksz8851_reg_read(REG_RX_CTRL1));
//	printf("RXCR2 [0x%02X]=0x%04X\n", REG_RX_CTRL2, ksz8851_reg_read(REG_RX_CTRL2));
	dmc_puts("RXCR2 [0x");
	dmc_puthex2(REG_RX_CTRL2);
	dmc_puts("]=0x");
	dmc_puthex4cr(ksz8851_reg_read(REG_RX_CTRL2));
//	printf("TXMIR [0x%02X]=0x%04X\n", REG_TX_MEM_INFO, ksz8851_reg_read(REG_TX_MEM_INFO));
	dmc_puts("TXMIR [0x");
	dmc_puthex2(REG_TX_MEM_INFO);
	dmc_puts("]=0x");
	dmc_puthex4cr(ksz8851_reg_read(REG_TX_MEM_INFO));
#if (READ_UNSAFE_REGISTERS)
//	printf("RXFHSR[0x%02X]=0x%04X\n", REG_RX_FHR_STATUS, ksz8851_reg_read(REG_RX_FHR_STATUS));
	dmc_puts("RXFHSR[0x");
	dmc_puthex2(REG_RX_FHR_STATUS);
	dmc_puts("]=0x");
	dmc_puthex4cr(ksz8851_reg_read(REG_RX_FHR_STATUS));
#endif
//	printf("TXQCR [0x%02X]=0x%04X\n", REG_TXQ_CMD, ksz8851_reg_read(REG_TXQ_CMD));
	dmc_puts("TXQCR [0x");
	dmc_puthex2(REG_TXQ_CMD);
	dmc_puts("]=0x");
	dmc_puthex4cr(ksz8851_reg_read(REG_TXQ_CMD));
//	printf("RXQCR [0x%02X]=0x%04X\n", REG_RXQ_CMD, ksz8851_reg_read(REG_RXQ_CMD));
	dmc_puts("RXQCR [0x");
	dmc_puthex2(REG_RXQ_CMD);
	dmc_puts("]=0x");
	dmc_puthex4cr(ksz8851_reg_read(REG_RXQ_CMD));
//	printf("TXFDPR[0x%02X]=0x%04X\n", REG_TX_ADDR_PTR, ksz8851_reg_read(REG_TX_ADDR_PTR));
	dmc_puts("TXFDPR[0x");
	dmc_puthex2(REG_TX_ADDR_PTR);
	dmc_puts("]=0x");
	dmc_puthex4cr(ksz8851_reg_read(REG_TX_ADDR_PTR));
//	printf("RXFDPR[0x%02X]=0x%04X\n", REG_RX_ADDR_PTR, ksz8851_reg_read(REG_RX_ADDR_PTR));
	dmc_puts("RXFDPR[0x");
	dmc_puthex2(REG_RX_ADDR_PTR);
	dmc_puts("]=0x");
	dmc_puthex4cr(ksz8851_reg_read(REG_RX_ADDR_PTR));
//	printf("IER   [0x%02X]=0x%04X\n", REG_INT_MASK, ksz8851_reg_read(REG_INT_MASK));
	dmc_puts("IER   [0x");
	dmc_puthex2(REG_INT_MASK);
	dmc_puts("]=0x");
	dmc_puthex4cr(ksz8851_reg_read(REG_INT_MASK));
//	printf("ISR   [0x%02X]=0x%04X\n", REG_INT_STATUS, ksz8851_reg_read(REG_INT_STATUS));
	dmc_puts("ISR   [0x");
	dmc_puthex2(REG_INT_STATUS);
	dmc_puts("]=0x");
	dmc_puthex4cr(ksz8851_reg_read(REG_INT_STATUS));
//	printf("RXFCTR[0x%02X]=0x%04X\n", KS_RXFCTR, ksz8851_reg_read(KS_RXFCTR));
	dmc_puts("RXFCTR[0x");
	dmc_puthex2(KS_RXFCTR);
	dmc_puts("]=0x");
	dmc_puthex4cr(ksz8851_reg_read(KS_RXFCTR));
#if (READ_UNSAFE_REGISTERS)
//	printf("TXNTFSR[0x%02X]=0x%04X\n", TXNTFSR, ksz8851_reg_read(TXNTFSR));
	dmc_puts("TXNTFSR[0x");
	dmc_puthex2(REG_TX_MEM_INFO);
	dmc_puts("]=0x");
	dmc_puthex4cr(ksz8851_reg_read(REG_TX_MEM_INFO));
#endif
	printf("CIDER [0x%02X]=0x%04X\n", REG_CHIP_ID, ksz8851_reg_read(REG_CHIP_ID));
	dmc_puts("CIDER [0x");
	dmc_puthex2(REG_CHIP_ID);
	dmc_puts("]=0x");
	dmc_puthex4cr(ksz8851_reg_read(REG_CHIP_ID));
//	printf("PHYRR [0x%02X]=0x%04X\n", REG_PHY_RESET, ksz8851_reg_read(REG_PHY_RESET));
	dmc_puts("PHYRR [0x");
	dmc_puthex2(REG_PHY_RESET);
	dmc_puts("]=0x");
	dmc_puthex4cr(ksz8851_reg_read(REG_PHY_RESET));
//	printf("P1MBCR[0x%02X]=0x%04X\n", REG_PHY_CNTL, ksz8851_reg_read(REG_PHY_CNTL));
	dmc_puts("P1MBCR[0x");
	dmc_puthex2(REG_PHY_CNTL);
	dmc_puts("]=0x");
	dmc_puthex4cr(ksz8851_reg_read(REG_PHY_CNTL));
//	printf("P1CR  [0x%02X]=0x%04X\n", REG_PORT_CTRL, ksz8851_reg_read(REG_PORT_CTRL));
	dmc_puts("P1CR  [0x");
	dmc_puthex2(REG_PORT_CTRL);
	dmc_puts("]=0x");
	dmc_puthex4cr(ksz8851_reg_read(REG_PORT_CTRL));
	dmc_puts("#################################################################\n");
}

void ksz8851_IntEnable(void)
{
//	if (interruptCnt)
//	{
//		interruptCnt--;
//	}
//	if (interruptCnt == 0)
//	{
	/* Enable interrupts */
	ksz8851_reg_write_0(REG_INT_MASK, (INT_RX | /* Enable receive interrupt */
	INT_RX_STOPPED | /* Enable receive process stopped interrupt */
	INT_TX_STOPPED | /* Enable transmit process stopped interrupt */
	INT_PHY | /* Enable link change interrupt */
	INT_RX_SPI_ERROR)); /* Enable receive SPI bus error interrupt */
//    }
}

void ksz8851_IntDisable(void)
{
	ksz8851_reg_write_0(REG_INT_MASK, INT_NO_INT);
//	interruptCnt++;
}

void ksz8851_IntClear(uint16_t flags)
{
	ksz8851_reg_write_0(REG_INT_STATUS, flags);
}

uint16_t ksz8851_IntGet(void)
{
	return ksz8851_reg_read(REG_INT_STATUS);
}

void ksz8851_PMECRStatusClear(uint16_t flags)
{
	uint16_t status = ksz8851_reg_read(REG_POWER_CNTL) | flags;
	ksz8851_reg_write_0(REG_POWER_CNTL, status);
}

uint16_t ksz8851_RXQCRGet(void)
{
	return ksz8851_reg_read(REG_RXQ_CMD);
}

uint16_t ksz8851_FrameCounterSet(void)
{
	/* Read the frame count and threshold register */
	uint16_t rxftr = ksz8851_reg_read(KS_RXFCTR);
	/* Extract the actual number of frames from RX_FRAME_THRES_REG */
	rxFrameCount = rxftr >> MSB_POS;
	return rxftr;
}

void ksz8851_TxQueueReset(void)
{
	uint16_t data;

	data = ksz8851_reg_read(REG_TX_CTRL);
	/* Disable TX */
	data &= ~(TX_FLOW_CTRL_ENABLE | TX_FLOW_CTRL_FLUSH_QUEUE);
	ksz8851_reg_write_0(REG_TX_CTRL, data);
	HAL_Delay(2);
	/* Flush */
	data |= TX_FLOW_CTRL_FLUSH_QUEUE;
	ksz8851_reg_write_0(REG_TX_CTRL, data);
	HAL_Delay(1);
	/* normal op - no flush */
	data &= ~TX_FLOW_CTRL_FLUSH_QUEUE;
	ksz8851_reg_write_0(REG_TX_CTRL, data);
	HAL_Delay(1);
	/* Enable TX */
	data |= TX_FLOW_CTRL_ENABLE;
	ksz8851_reg_write_0(REG_TX_CTRL, data);
	HAL_Delay(1);
}

void ksz8851_RxQueueReset(void)
{
	uint16_t data;

	data = ksz8851_reg_read(REG_RX_CTRL1);
	/* Disable RX */
	data &= ~(RX_FLOW_CTRL_RX_ENABLE | RX_FLOW_CTRL_FLUSH_QUEUE);
	ksz8851_reg_write_0(REG_RX_CTRL1, data);
	HAL_Delay(2);
	/* Flush */
	ksz8851_reg_write_0(REG_RX_CTRL1, data);
	/* Clear flush */
	data &= ~RX_FLOW_CTRL_FLUSH_QUEUE;
	ksz8851_reg_write_0(REG_RX_CTRL1, data);
	HAL_Delay(1);
	/* Write default config with enable set */
	data |= RX_FLOW_CTRL_RX_ENABLE;
	ksz8851_reg_write_0(REG_RX_CTRL1, data);
	HAL_Delay(1);
}

// Required
uint32_t ksz8851snl_reset_rx( void )
{
	uint16_t usValue;

	usValue = ksz8851_reg_read(REG_RX_CTRL1);
	usValue &= ~( ( uint16_t ) RX_CTRL_ENABLE | RX_CTRL_FLUSH_QUEUE );
	ksz8851_reg_write_0( REG_RX_CTRL1, usValue );
	HAL_Delay( 2 );
	ksz8851_reg_write_0( REG_RX_CTRL1, usValue | RX_CTRL_FLUSH_QUEUE );
	HAL_Delay( 1 );
	ksz8851_reg_write_0( REG_RX_CTRL1, usValue );
	HAL_Delay( 1 );
	ksz8851_reg_write_0( REG_RX_CTRL1, usValue | RX_CTRL_ENABLE );
	HAL_Delay( 1 );

	return ( uint32_t )usValue;
}

// Required
uint32_t ksz8851snl_reset_tx( void )
{
	uint16_t usValue;

	usValue = ksz8851_reg_read( REG_TX_CTRL );
	usValue &= ~( ( uint16_t ) TX_CTRL_ENABLE | TX_CTRL_FLUSH_QUEUE );
	ksz8851_reg_write_0( REG_TX_CTRL, usValue );
	HAL_Delay( 2 );
	ksz8851_reg_write_0( REG_TX_CTRL, usValue | TX_CTRL_FLUSH_QUEUE );
	HAL_Delay( 1 );
	ksz8851_reg_write_0( REG_TX_CTRL, usValue );
	HAL_Delay( 1 );
	ksz8851_reg_write_0( REG_TX_CTRL, usValue | TX_CTRL_ENABLE );
	HAL_Delay( 1 );

	return ( uint32_t )usValue;
}

uint16_t ksz8851_FrameCounterGet(void)
{
	return rxFrameCount;
}

void ksz8851_Enable(void)
{
	uint16_t data;

	ksz8851_reg_write_0(REG_INT_MASK, KSZ8851SNL_INT_ENABLE_MASK);

	/* Enable QMU Transmit */
	data = ksz8851_reg_read(REG_TX_CTRL);
	data |= TX_FLOW_CTRL_ENABLE;
	ksz8851_reg_write_0(REG_TX_CTRL, data);

	/* Enable QMU Receive */
	data = ksz8851_reg_read(REG_RX_CTRL1);
	data |= RX_FLOW_CTRL_RX_ENABLE;
	ksz8851_reg_write_0(REG_RX_CTRL1, data);
}

bool ksz8851_TransmitBegin(uint16_t length)
{
	uint16_t txmir;
	uint16_t data, reqSize;
	uint8_t outbuf[4];

	/* Wait for previous frame to finish before setting up a new one */
	while (ksz8851_reg_read(REG_TXQ_CMD) & TXQ_ENQUEUE)
	{
		;
	}

	/* Make sure there is room for
	 *
	 * 4 bytes Control Word
	 * 4 bytes Byte Count
	 * n bytes Packet
	 * 4 bytes CRC
	 */
	reqSize = length + 12;
	txmir = ksz8851_reg_read(REG_TX_MEM_INFO) & TX_MEM_AVAIL_MASK;
//	LWIP_DEBUGF(NETIF_DEBUG, ("KSZ8851SNL_LongTransmitInit: txmir =%hu  reqSize = %hu \n", txmir, reqSize));

	if (txmir < reqSize)
	{
		/* TXQ is out of memory */
//		LWIP_DEBUGF(NETIF_DEBUG | LWIP_DBG_LEVEL_WARNING,
//				("Not enough TXQ Memory, available=%u required=%u\n", txmir, reqSize));
		return false;
	}

//	LWIP_DEBUGF(NETIF_DEBUG,
//			("KSZ8851SNL_LongTransmitInit: Memory available >  txmir =%hu  reqSize = %hu \n", txmir, reqSize));
	/* Enable TXQ write access */
	data = ksz8851_reg_read(REG_RXQ_CMD) | RXQ_START_DMA;
	ksz8851_reg_write_0(REG_RXQ_CMD, data);

	/* Write frame ID, control word and byte count */
	outbuf[0] = (frameId++ & FRAME_ID_MASK);
	outbuf[1] = 0x80; /*  TX_INT_on_COMPLETION */
	outbuf[2] = length & LSB_MASK;
	outbuf[3] = length >> MSB_POS;

	/* Start the SPI Transfer and send frame header */
	ksz8851_fifo_write(outbuf, 4);
//	KSZ8851SNL_SPI_WriteFifoBegin();
//	KSZ8851SNL_SPI_WriteFifo(4, outbuf);

	return true;
}

void ksz8851_Transmit(uint16_t length, const uint8_t *buffer)
{
//	EFM_ASSERT(buffer != NULL);
	ksz8851_fifo_write(buffer, length);
}

/***************************************************************************/
void ksz8851_TransmitEnd(uint16_t length)
{
	uint16_t data;
	uint16_t padding;
	uint8_t dummy[4] =
	{ 0x00 };

	/* Padding packet to 4 byte boundary */
	if (length % 4)
	{
		padding = 4 - (length % 4);
		ksz8851_fifo_write(dummy, padding);
	}

	/* Stop the SPI Transfer */
//	KSZ8851SNL_SPI_WriteFifoEnd();
	/* Disable TXQ write access */
	data = ksz8851_reg_read(RXQCR) & (~RXQ_START_DMA);
	ksz8851_reg_write_0(RXQCR, data);

	/* Start TXQ Manual enqueue */
	data = ksz8851_reg_read(TXQCR) | TXQ_ENQUEUE;
	ksz8851_reg_write_0(TXQCR, data);
}

static void ksz8851_ReleaseIncosistentFrame(void)
{
	uint16_t data;
	/* Issue the Release error frame command */
	data = ksz8851_reg_read(REG_RXQ_CMD);
	data |= RXQ_RELEASE_ERROR_FRAME;
	ksz8851_reg_write_0(REG_RXQ_CMD, data);

	/* Wait for PHY to clear the command/flag */
	while (ksz8851_reg_read(REG_RXQ_CMD) & RXQ_RELEASE_ERROR_FRAME)
	{
		;
	}
}

/***************************************************************************//**
 * @brief Performs the actual transmit of a raw frame over the network.
 *
 * @param pTXLength
 *     the length of the transmitted frame
 * @param pTXData
 *     the data of the transmitted frame
 *****************************************************************************/
void ksz8851_Send(uint16_t pTXLength, uint8_t *pTXData)
{
//  EFM_ASSERT(pTXData != NULL);

	uint16_t txmir;
	uint16_t data, reqSize;
	uint8_t outbuf[4];

	/* Check if TXQ has enough memory for the transmission of the package */
	data = ksz8851_reg_read(REG_TX_MEM_INFO);
	txmir = data & TX_MEM_AVAIL_MASK;

	reqSize = pTXLength + EXTRA_SIZE;
	if (txmir < reqSize)
	{
//		KSZ8851SNL_ExceptionHandler(INFO, "I will wait until mem is available\n");
		/* Enable TX memory available monitor */
		ksz8851_reg_write_0(REG_TX_TOTAL_FRAME_SIZE, reqSize);
		data = ksz8851_reg_read(REG_TXQ_CMD);
		data |= TXQ_MEM_AVAILABLE_INT;
		ksz8851_reg_write_0(REG_TXQ_CMD, data);

		/* Wait until enough space is available */
		while (1)
		{
			data = ksz8851_reg_read(REG_INT_STATUS);
			if ((data & INT_TX_SPACE) == INT_TX_SPACE)
			{
				break;
			}
		}
//		KSZ8851SNL_ExceptionHandler(INFO, "Done\n");

		/* Clear flag */
		data = ksz8851_reg_read(REG_INT_STATUS);
		data &= ~INT_TX_SPACE;
		ksz8851_reg_write_0(REG_INT_STATUS, data);
	}

	/* Disable interupts on KSZ8851SNL */
	data = NO_INT;
	ksz8851_reg_write_0(REG_INT_MASK, data);

	/* Enable TXQ write access */
	data = ksz8851_reg_read(REG_RXQ_CMD);
	data |= RXQ_START;
	ksz8851_reg_write_0(REG_RXQ_CMD, data);

	/* Write frame ID, control word and byte count */
	outbuf[0] = (frameId++ & FRAME_ID_MASK) | TX_INT_on_COMPLETION;
	outbuf[1] = 0;
	outbuf[2] = pTXLength & LSB_MASK;
	outbuf[3] = pTXLength >> MSB_POS;

	/* Start the SPI Transfer */
//	ETHSPI_StartWriteFIFO();
	/* Send the frame header info */
//	ETHSPI_WriteFifoContinue(4, outbuf);

	/* Send the actual data */
//	ETHSPI_WriteFifoContinue(pTXLength, pTXData);

	/* Send dummy bytes to align data to DWORD */
//	ETHSPI_WriteFifoContinue(KSZ8851SNL_DwordAllignDiff(pTXLength), pTXData);

	/* Stop the SPI Transfer */
//	ETHSPI_StopFIFO();

	/* Disable TXQ write access */
	data = ksz8851_reg_read(REG_RXQ_CMD);
	data &= ~RXQ_START;
	ksz8851_reg_write_0(REG_RXQ_CMD, data);

	/* Start TXQ Manual Engue */
	data = ksz8851_reg_read(REG_TXQ_CMD);
	data |= TXQ_ENQUEUE;
	ksz8851_reg_write_0(REG_TXQ_CMD, data);

	/* Wait until transmit command clears */
	while (1)
	{
		data = ksz8851_reg_read(REG_TXQ_CMD);
		if (!(data & TXQ_ENQUEUE))
			break;
	}

	/* Enable interupts */
	data = INT_MASK_EXAMPLE;
	ksz8851_reg_write_0(REG_INT_MASK, data);
}

/***************************************************************************//**
 * @brief Performs the actual receive of a raw frame over the network.
 *
 * @param pRXLength
 *     the length of the received frame
 * @param pRXData
 *     the data of the received frame
 *
 * @return
 *     received packet length, 0 in case of failure
 *****************************************************************************/
uint16_t ksz8851_Receive(uint8_t *pRXData, uint16_t pRXLength)
{
	uint16_t data;
	uint16_t rxFrameCount, rxftr;
	uint16_t rxStatus;
	uint16_t rxPacketLength;
	uint16_t frameLength;
	uint16_t bytesToRead;

//	EFM_ASSERT(buffer != NULL);
//	EFM_ASSERT(pRXData != NULL);
//	EFM_ASSERT(pRXLength != NULL);

	/* Read the frame count and threshold register */
	rxftr = ksz8851_reg_read(REG_RX_FRAME_CNT_THRES);
	/* Extract the actual number of frames from RX_FRAME_THRES_REG*/
	rxFrameCount = rxftr >> MSB_POS;

	while (rxFrameCount > 0)
	{
		rxFrameCount--;
		/* Read the received frame status */
		rxStatus = ksz8851_reg_read(REG_RX_FHR_STATUS);

		/* Check the consistency of the frame */
		if ((!(rxStatus & VALID_FRAME_MASK)) || (rxStatus & CHECKSUM_VALID_FRAME_MASK))
		{
			/* Issue the Release error frame command */
			ksz8851_ReleaseIncosistentFrame();
			/* continue to next frame */
			continue;
		}
		else
		{
			/* Read the byte size of the received frame */
			rxPacketLength = ksz8851_reg_read(RXFHBCR) & RX_BYTE_CNT_MASK;

			/* round to dword boundary */
			bytesToRead = 4 * ((rxPacketLength + 3) >> 2);
//			LWIP_DEBUGF(NETIF_DEBUG,
//					("KSZ8851SNL_Receive: rxPacketLength=%u, bytesToRead=%u \n", rxPacketLength, bytesToRead));
			if ((bytesToRead > pRXLength) || (rxPacketLength <= 4))
			{
				/* Issue the Release error frame command */
				ksz8851_ReleaseIncosistentFrame();
				/* continue to next frame */
				continue;
			}

			/* Set QMU RXQ frame pointer to start of packet data. Note
			 * that we skip the status word and byte count since we
			 * already know this from RXFHSR and RXFHBCR.
			 */
			ksz8851_reg_write_0(REG_RX_ADDR_PTR, 0x0004 | FD_PTR_AUTO_INC);

			/* Start QMU DMA transfer */
			data = ksz8851_reg_read(REG_RXQ_CMD);
			data |= RXQ_START_DMA;
			ksz8851_reg_write_0(REG_RXQ_CMD, data);

			/* Read the whole ethernet frame */
			ksz8851_fifo_read(pRXData, bytesToRead);

			/* Stop QMU DMA transfer */
			data = ksz8851_reg_read(REG_RXQ_CMD);
			data &= ~RXQ_START_DMA;
			ksz8851_reg_write_0(REG_RXQ_CMD, data);

			/* Enable interupts */
			data = INT_MASK_EXAMPLE;
			ksz8851_reg_write_0(REG_INT_MASK, data);

			/* Return frame length without CRC */
			frameLength = rxPacketLength - 4;
			return frameLength;
		}
		/* Enable interupts */
		data = INT_MASK_EXAMPLE;
		ksz8851_reg_write_0(REG_INT_MASK, data);
	}
	return 0;
}

/***************************************************************************//**
 * @brief Get the MAC address of the current board.
 * @note  Support method used for minimizing the code size.
 * @param[out] macAddress
 *     data buffer to store the macAddress
 *****************************************************************************/
void ksz8851_GetMacAddress(uint8_t *macAddress)
{
	/* TODO:  Get MAC based on actual MAC and not on the CMU unique ID. */

//	int i;
//	EFM_ASSERT(macAddress != NULL);
	/* set the first 3 bytes given by the EM MAC Address space */
//	macAddress[0] = HIGH_QMU_MAC_H;
//	macAddress[1] = HIGH_QMU_MAC_L;
//	macAddress[2] = MID_QMU_MAC_H;
	/* set the next 3 bytes given by the CMU unique ID */

//	for (i = 0; i < 3; i++)
//	{
//		macAddress[5 - i] = (DEVINFO->UNIQUEL & (BYTE_MASK << i * BYTE_SIZE)) >> i * BYTE_SIZE;
//	}
}

uint16_t ksz8851_PHYStatusGet(void)
{
	return ksz8851_reg_read(P1SR);
}

static void ksz8851_SetDigitalLoopbackMode(void)
{
	uint16_t data;
	/* Reset PHY. */
	data = PHY_RESET;
	ksz8851_reg_write_0(REG_PHY_RESET, data);
	/* Disable Auto-negotiation.1. Reset PHY. */
	/* Set Speed to either 100Base-TX or 10Base-T. */
	/* Set Duplex to full-duplex. */
	/* Set PHY register 0.14 to 1 to enable Local Loop-back. */
	data = DIGITAL_LOOPBACK | FORCE_FULL_DUPLEX | FORCE_100;
	data &= ~AUTO_NEG;
	ksz8851_reg_write_0(REG_PHY_CNTL, data);

//  KSZ8851SNL_ExceptionHandler(INFO, "Loopback mode initiated");
}

/**************************************************************************//**
 * @brief enables the chip interrupts
 *****************************************************************************/
void ksz8851_EnableInterupts(void)
{
	/* Enable interupts */
	ksz8851_reg_write_0(INT_ENABLE_REG, INT_MASK_EXAMPLE);
}

// https://github.com/EnergyMicro/kit_common/blob/master/drivers/ksz8851snl.c
/***************************************************************************//**
 * @brief Checks for any interrrupts and if found, clears their status
 *        and prepair for interrupt handler routines
 * @note  Programmer needs to re-enable the KSZ8851SNL interrupts after
 *        calling this function
 *
 * @return
 *     found interrupts
 *
 *****************************************************************************/
uint16_t ksz8851_CheckIrqStat(void)
{
	uint16_t data, ISR_stat, found_INT;
	found_INT = 0;
	ISR_stat = ksz8851_reg_read(REG_INT_STATUS);

	/* Disable interupts on KSZ8851SNL */
	data = NO_INT;
	ksz8851_reg_write_0(REG_INT_MASK, data);

	/* Resolve the RX completion interrupt */
	if (ISR_stat & INT_RX_DONE)
	{
		/* Clear RX Interrupt flag */
		data = ksz8851_reg_read(REG_INT_STATUS);
		data = INT_RX_DONE;
		ksz8851_reg_write_0(REG_INT_STATUS, data);
		found_INT |= INT_RX_DONE;
	}
	/* Resolve the Link change interrupt */
	if (ISR_stat & INT_LINK_CHANGE)
	{
		/* Clear Link change Interrupt flag */
		data = ksz8851_reg_read(REG_INT_STATUS);
		data = INT_LINK_CHANGE;
		ksz8851_reg_write_0(REG_INT_STATUS, data);
		found_INT |= INT_LINK_CHANGE;
	}
	/* Resolve the RX overrun interrupt */
	if (ISR_stat & INT_RX_OVERRUN)
	{
		/* Clear RX overrun Interrupt flag */
		data = ksz8851_reg_read(REG_INT_STATUS);
		data = INT_RX_OVERRUN;
		ksz8851_reg_write_0(REG_INT_STATUS, data);
		found_INT |= INT_RX_OVERRUN;
	}
	/* Resolve the TX stopped interrupt */
	if (ISR_stat & INT_TX_STOPPED)
	{
		/* Clear TX stopped Interrupt flag */
		data = ksz8851_reg_read(REG_INT_STATUS);
		data = INT_TX_STOPPED;
		ksz8851_reg_write_0(REG_INT_STATUS, data);
		found_INT |= INT_TX_STOPPED;
	}
	/* Resolve the RX stopped interrupt */
	if (ISR_stat & INT_RX_STOPPED)
	{
		/* Clear RX stopped Interrupt flag */
		data = ksz8851_reg_read(REG_INT_STATUS);
		data = INT_RX_STOPPED;
		ksz8851_reg_write_0(REG_INT_STATUS, data);
		found_INT |= INT_RX_STOPPED;
	}
	/* Resolve the RX of a WakeOnLan frame interrupt */
	if (ISR_stat & INT_RX_WOL_FRAME)
	{
		/* Clear RX of a WakeOnLan Interrupt flag */
		data = ksz8851_reg_read(REG_INT_STATUS);
		data = INT_RX_WOL_FRAME;
		ksz8851_reg_write_0(REG_INT_STATUS, data);
		found_INT |= INT_RX_WOL_FRAME;
	}
	/* Resolve the RX of a magic frame interrupt */
	if (ISR_stat & INT_MAGIC)
	{
		/* Clear RX of a magic frame Interrupt flag */
		data = ksz8851_reg_read(REG_INT_STATUS);
		data = INT_MAGIC;
		ksz8851_reg_write_0(REG_INT_STATUS, data);
		found_INT |= INT_MAGIC;
	}
	/* Resolve the RX of a LINKUP interrupt */
	if (ISR_stat & INT_LINKUP)
	{
		/* Clear RX of a LINKUP Interrupt flag */
		data = ksz8851_reg_read(REG_INT_STATUS);
		data = INT_LINKUP;
		ksz8851_reg_write_0(REG_INT_STATUS, data);
		found_INT |= INT_LINKUP;
	}
	/* Resolve the RX of a Energy interrupt */
	if (ISR_stat & INT_ENERGY)
	{
		/* Clear RX of a Energy Interrupt flag */
		data = ksz8851_reg_read(REG_INT_STATUS);
		data = INT_ENERGY;
		ksz8851_reg_write_0(REG_INT_STATUS, data);
		found_INT |= INT_ENERGY;
	}
	/* Resolve the SPI Error interrupt */
	if (ISR_stat & INT_SPI_ERROR)
	{
		/* Clear SPI Error Interrupt flag */
		data = ksz8851_reg_read(REG_INT_STATUS);
		data = INT_SPI_ERROR;
		ksz8851_reg_write_0(REG_INT_STATUS, data);
		found_INT |= INT_SPI_ERROR;
	}
	/* Resolve the TX space interrupt */
	if (ISR_stat & INT_TX_SPACE)
	{
		/* Clear TX space Interrupt flag */
		data = ksz8851_reg_read(REG_INT_STATUS);
		data = INT_TX_SPACE;
		ksz8851_reg_write_0(REG_INT_STATUS, data);
		found_INT |= INT_TX_SPACE;
	}
	return found_INT;
}

// https://github.com/EnergyMicro/kit_common/blob/master/drivers/ksz8851snl.c
/***************************************************************************//**
 * @brief Returns the size of the currently received frame
 *
 * @return
 *     the printed string
 *
 *****************************************************************************/
uint16_t ksz8851_CurrFrameSize(void)
{
	uint16_t data;

	/* Read the byte size of the received frame */
	data = ksz8851_reg_read(REG_RX_FHR_BYTE_CNT);

	data &= RX_BYTE_CNT_MASK;

	return data;
}

/***************************************************************************//**
 * @brief Returns the difference in bytes to be DWORD aligned
 *
 * @param val
 *     value that needs to be aligned
 * @return
 *     the number of bytes needed to be added so that the value is aligned
 *
 *****************************************************************************/
static uint8_t ksz8851_DwordAllignDiff(uint8_t val)
{
	if (val % 4 == 0)
	{
		return 0;
	}
	else
	{
		return val % 4;
	}
}

/**
 * @brief CRC calculation
 * @param[in] data Pointer to the data over which to calculate the CRC
 * @param[in] length Number of bytes to process
 * @return Resulting CRC value
 **/
uint32_t ksz8851_CalcCrc(const void *data, size_t length)
{
	uint32_t i;
	uint32_t j;

	//Point to the data over which to calculate the CRC
	const uint8_t *p = (uint8_t *) data;
	//CRC preset value
	uint32_t crc = 0xFFFFFFFF;

	//Loop through data
	for (i = 0; i < length; i++)
	{
		//The message is processed bit by bit
		for (j = 0; j < 8; j++)
		{
			//Update CRC value
			if (((crc >> 31) ^ (p[i] >> j)) & 0x01)
				crc = (crc << 1) ^ 0x04C11DB7;
			else
				crc = crc << 1;
		}
	}

	//Return CRC value
	return crc;
}

// https://www.oryx-embedded.com/doc/ksz8851_8c_source.html
// Doc only
/**
 * @brief Send a packet
 * @param[in] interface Underlying network interface
 * @param[in] buffer Multi-part buffer containing the data to send
 * @param[in] offset Offset to the first data byte
 * @return Error code
 **/
//error_t ksz8851_SendPacket(NetInterface *interface, const NetBuffer *buffer, size_t offset)
//{
//   size_t n;
//   size_t length;
//   Ksz8851TxHeader header;
//   Ksz8851Context *context;
//
//   //Point to the driver context
//   context = (Ksz8851Context *) interface->nicContext;
//
//   //Retrieve the length of the packet
//   length = netBufferGetLength(buffer) - offset;
//
//   //Check the frame length
//   if(length > ETH_MAX_FRAME_SIZE)
//   {
//      //The transmitter can accept another packet
//      osSetEvent(&interface->nicTxEvent);
//      //Report an error
//      return ERROR_INVALID_LENGTH;
//   }
//
//   //Get the amount of free memory available in the TX FIFO
//   n = ksz8851ReadReg(interface, KSZ8851_REG_TXMIR) & TXMIR_TXMA_MASK;
//
//   //Make sure enough memory is available
//   if((length + 8) > n)
//      return ERROR_FAILURE;
//
//   //Copy user data
//   netBufferRead(context->txBuffer, buffer, offset, length);
//
//   //Format control word
//   header.controlWord = TX_CTRL_TXIC | (context->frameId++ & TX_CTRL_TXFID);
//   //Total number of bytes to be transmitted
//   header.byteCount = length;
//
//   //Enable TXQ write access
//   ksz8851SetBit(interface, KSZ8851_REG_RXQCR, RXQCR_SDA);
//   //Write TX packet header
//   ksz8851WriteFifo(interface, (uint8_t *) &header, sizeof(Ksz8851TxHeader));
//   //Write data
//   ksz8851WriteFifo(interface, context->txBuffer, length);
//   //End TXQ write access
//   ksz8851ClearBit(interface, KSZ8851_REG_RXQCR, RXQCR_SDA);
//
//   //Start transmission
//   ksz8851SetBit(interface, KSZ8851_REG_TXQCR, TXQCR_METFE);
//
//   //Successful processing
//   return NO_ERROR;
//}


/**
 * @brief Receive a packet
 * @param[in] interface Underlying network interface
 * @return Error code
 **/
//error_t ksz8851_ReceivePacket(NetInterface *interface)
//{
//   size_t n;
//   uint16_t status;
//   Ksz8851Context *context;
//
//   //Point to the driver context
//   context = (Ksz8851Context *) interface->nicContext;
//
//   //Read received frame status from RXFHSR
//   status = ksz8851ReadReg(interface, KSZ8851_REG_RXFHSR);
//
//   //Make sure the frame is valid
//   if(status & RXFHSR_RXFV)
//   {
//      //Check error flags
//      if(!(status & (RXFHSR_RXMR | RXFHSR_RXFTL | RXFHSR_RXRF | RXFHSR_RXCE)))
//      {
//         //Read received frame byte size from RXFHBCR
//         n = ksz8851ReadReg(interface, KSZ8851_REG_RXFHBCR) & RXFHBCR_RXBC_MASK;
//
//         //Ensure the frame size is acceptable
//         if(n > 0 && n <= ETH_MAX_FRAME_SIZE)
//         {
//            //Reset QMU RXQ frame pointer to zero
//            ksz8851WriteReg(interface, KSZ8851_REG_RXFDPR, RXFDPR_RXFPAI);
//            //Enable RXQ read access
//            ksz8851SetBit(interface, KSZ8851_REG_RXQCR, RXQCR_SDA);
//            //Read data
//            ksz8851ReadFifo(interface, context->rxBuffer, n);
//            //End RXQ read access
//            ksz8851ClearBit(interface, KSZ8851_REG_RXQCR, RXQCR_SDA);
//
//            //Pass the packet to the upper layer
//            nicProcessPacket(interface, context->rxBuffer, n);
//            //Valid packet received
//            return NO_ERROR;
//         }
//      }
//   }
//
//   //Release the current error frame from RXQ
//   ksz8851SetBit(interface, KSZ8851_REG_RXQCR, RXQCR_RRXEF);
//   //Report an error
//   return ERROR_INVALID_PACKET;
//}

