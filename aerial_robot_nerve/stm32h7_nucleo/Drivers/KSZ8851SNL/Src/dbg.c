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

//#include <stdio.h>
//#include <stdlib.h>
//#include <gpio.h>
//#include "string.h"
//#include "cmsis_os.h"
//#include "stm32h7xx_hal.h"
//#include <stm32h7xx_hal_uart.h>
#include <usart.h>
#include "main.h"
#include "dbg.h"

//
//#define LED_3			            	GPIO_PIN_13
//#define LED_4			            	GPIO_PIN_12
//#define LED_5			            	GPIO_PIN_14
//#define LED_6			            	GPIO_PIN_15
//#define LED_PORT		            	GPIOD
//
#define UART_BUFFER_DEPTH				5
#define UART_BUFFER_SIZE				DBG_UART_BUFFER_MSG_LENGTH
#define UART_HAL_TIMEOUT_MS			200

#define UART_TASK_PERIOD_MS			(UART_HAL_TIMEOUT_MS * UART_BUFFER_DEPTH)

static UART_HandleTypeDef UartHandle;

static char uartBuffer[UART_BUFFER_DEPTH][UART_BUFFER_SIZE];

static uint8_t uartBufferHeadIndex = 0;

static uint8_t uartBufferTailIndex = 0;

void DBG_sendString( char * pString, uint8_t length )
{
	/* limit length */
	if( length > UART_BUFFER_SIZE )
	{
		length = UART_BUFFER_SIZE;
	}
	HAL_UART_Transmit(&huart3, (uint8_t *) pString, strlen(length), 10);	// Jack

	/* copy message in the free location */
	memcpy(uartBuffer[uartBufferHeadIndex], pString, length);

	/* next index; do not wrap around */
	if( uartBufferHeadIndex < UART_BUFFER_DEPTH )
	{
		uartBufferHeadIndex++;
	}
	else
	{
		/* next message will overwrite last location */
	}
}

void DBG_toggleDbgLED( void )
{
	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);	// Blue
//	HAL_GPIO_TogglePin(LED_PORT, LED_3);
}
