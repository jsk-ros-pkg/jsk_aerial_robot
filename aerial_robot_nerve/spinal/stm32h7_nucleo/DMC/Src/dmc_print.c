#include "dmc_print.h"
#include "dmc_convert.h"
#include <stdarg.h>
#include <string.h>
#include "usart.h"
//#include "stm32f7xx_hal_uart.h"

void debugPrint(UART_HandleTypeDef *huart, char _out[])
{
	HAL_UART_Transmit(huart, (uint8_t *) _out, strlen(_out), 10);
}

void debugPrintln(UART_HandleTypeDef *huart, char _out[])
{
	HAL_UART_Transmit(huart, (uint8_t *) _out, strlen(_out), 10);
	char newline[] = "\n";
	HAL_UART_Transmit(huart, (uint8_t *) newline, 2, 10);
}

void VprintUart3(const char *fmt, va_list argp)
{
	char string[200];
	if (0 < vsprintf(string, fmt, argp)) // build string
	{
		HAL_UART_Transmit(&huart3, (uint8_t*) string, (uint16_t) strlen(string),
				(uint32_t) 0xffffff); // send message via UART
	}
}

void printfUart2(const char *fmt, ...) // custom printf() function
{
	va_list argp;
	va_start(argp, fmt);
	VprintUart2(fmt, argp);
	va_end(argp);
}

void printfUart3(const char *fmt, ...) // custom printf() function
{
	va_list argp;
	va_start(argp, fmt);
	VprintUart3(fmt, argp);
	va_end(argp);
}

int __io_putchar(int ch)
{
	uint8_t c[1];
	c[0] = ch & 0x00FF;
	HAL_UART_Transmit(&huart3, &*c, 1, 10);
	return ch;
}

int __io_getchar(void)
{
	HAL_StatusTypeDef Status = HAL_BUSY;
	uint8_t ch;
	// Remember RS485 TX_Enable
	while (Status != HAL_OK)
	{
		Status = HAL_UART_Receive(&huart3, &ch, 1, 10);
	}
	return (ch);
}

//int _write(int file, char *ptr, int len)
//{
//	int DataIdx;
//
//	for (DataIdx = 0; DataIdx < len; DataIdx++)
//	{
//	   __io_putchar( *ptr++ );
//	}
//	return len;
//}
//
//int _read(int file, char *ptr, int len)
//{
//	int DataIdx;
//
//	for (DataIdx = 0; DataIdx < len; DataIdx++)
//	{
//	  *ptr++ = __io_getchar();
//	}
//
//   return len;
//}

void enableFlushAfterPrintf(void)
{
	setvbuf(stdout, 0, _IONBF, 0);
	setvbuf(stdin, 0, _IONBF, 0);
}

void dmc_putint(int value)
{
	char myStr[30];
	dmc_itoa(value, myStr, 10);
	dmc_puts(myStr);
}

void dmc_putintcr(int value)
{
	char myStr[30];
	dmc_itoa(value, myStr, 10);
	dmc_putscr(myStr);
}

void dmc_putint2(int value, char c)
{
	char myStr[30];
	dmc_itoa(value, myStr, 10);
	if (strlen(myStr) < 2)
	{
		__io_putchar(c);
	}
	dmc_puts(myStr);
}

void dmc_putint2cr(int value, char c)
{
	char myStr[30];
	dmc_itoa(value, myStr, 10);
	if (strlen(myStr) < 2)
	{
		__io_putchar(c);
	}
	dmc_putscr(myStr);
}

void dmc_putint4(int value, char c)
{
	char myStr[30];
	dmc_itoa(value, myStr, 10);
	uint8_t len = strlen(myStr);
	for (; len < 4; len++)
	{
		__io_putchar(c);
	}
	dmc_puts(myStr);
}

void dmc_putint4cr(int value, char c)
{
	char myStr[30];
	dmc_itoa(value, myStr, 10);
	uint8_t len = strlen(myStr);
	for (; len < 4; len++)
	{
		__io_putchar(c);
	}
	dmc_putscr(myStr);
}

void dmc_putint6(int value, char c)
{
	char myStr[30];
	dmc_itoa(value, myStr, 10);
	uint8_t len = strlen(myStr);
	for (; len < 6; len++)
	{
		__io_putchar(c);
	}
	dmc_puts(myStr);
}

void dmc_putint6cr(int value, char c)
{
	char myStr[30];
	dmc_itoa(value, myStr, 10);
	uint8_t len = strlen(myStr);
	for (; len < 6; len++)
	{
		__io_putchar(c);
	}
	dmc_putscr(myStr);
}

void dmc_putint8(int value, char c)
{
	char myStr[30];
	dmc_itoa(value, myStr, 10);
	uint8_t len = strlen(myStr);
	for (; len < 8; len++)
	{
		__io_putchar(c);
	}
	dmc_puts(myStr);
}

void dmc_putint8cr(int value, char c)
{
	char myStr[30];
	dmc_itoa(value, myStr, 10);
	uint8_t len = strlen(myStr);
	for (; len < 8; len++)
	{
		__io_putchar(c);
	}
	dmc_putscr(myStr);
}

void dmc_putintstr(int value, char* str)
{
	char myStr[30];
	dmc_itoa(value, myStr, 10);
	dmc_puts(myStr);
	dmc_puts(str);
}

void dmc_putintstrcr(int value, char* str)
{
	char myStr[30];
	dmc_itoa(value, myStr, 10);
	dmc_puts(myStr);
	dmc_putscr(str);
}

void dmc_putstrint(char* str, int value)
{
	char myStr[30];
	dmc_itoa(value, myStr, 10);
	dmc_puts2(str, myStr);
}

void dmc_putstrintcr(char* str, int value)
{
	char myStr[30];
	dmc_itoa(value, myStr, 10);
	dmc_puts2cr(str, myStr);
}

void dmc_putstrintstr(char* str1, int value, char* str2)
{
	char myStr[30];
	dmc_itoa(value, myStr, 10);
	dmc_puts3(str1, myStr, str2);
}

void dmc_putbin8cr(uint32_t value)
{
	char myStr[30];
	dmc_itoa(value, myStr, 2);
	uint8_t len = strlen(myStr);
	for (; len < 8; len++)
	{
		__io_putchar('0');
	}
	dmc_puts(myStr);
	__io_putchar('\n');
}

void dmc_putbin16cr(uint32_t value)
{
	char myStr[30];
	dmc_itoa(value, myStr, 2);
	uint8_t len = strlen(myStr);
	for (; len < 16; len++)
	{
		__io_putchar('0');
	}
	dmc_puts(myStr);
	__io_putchar('\n');
}

void dmc_putbin32cr(uint32_t value)
{
	char myStr[30];
	dmc_itoa(value, myStr, 2);
	uint8_t len = strlen(myStr);
	for (; len < 32; len++)
	{
		__io_putchar('0');
	}
	dmc_puts(myStr);
	__io_putchar('\n');
}

void dmc_putbin8str(uint32_t value, char* str)
{
	char myStr[30];
	dmc_itoa(value, myStr, 2);
	uint8_t len = strlen(myStr);
	for (; len < 8; len++)
	{
		__io_putchar('0');
	}
	dmc_puts2(myStr, str);
}

void dmc_putbin16str(uint32_t value, char* str)
{
	char myStr[30];
	dmc_itoa(value, myStr, 2);
	uint8_t len = strlen(myStr);
	for (; len < 16; len++)
	{
		__io_putchar('0');
	}
	dmc_puts2(myStr, str);
}

void dmc_putbin32str(uint32_t value, char* str)
{
	char myStr[30];
	dmc_itoa(value, myStr, 2);
	uint8_t len = strlen(myStr);
	for (; len < 32; len++)
	{
		__io_putchar('0');
	}
	dmc_puts2(myStr, str);
}

void dmc_putstrbin8str(char* str1, uint32_t value, char* str2)
{
	char myStr[30];
	dmc_itoa(value, myStr, 2);
	dmc_puts(str1);
	uint8_t len = strlen(myStr);
	for (; len < 8; len++)
	{
		__io_putchar('0');
	}
	dmc_puts2(myStr, str2);
}

void dmc_putstrbin16str(char* str1, uint32_t value, char* str2)
{
	char myStr[30];
	dmc_itoa(value, myStr, 2);
	dmc_puts(str1);
	uint8_t len = strlen(myStr);
	for (; len < 16; len++)
	{
		__io_putchar('0');
	}
	dmc_puts2(myStr, str2);
}

void dmc_putstrbin32str(char* str1, uint32_t value, char* str2)
{
	char myStr[30];
	dmc_itoa(value, myStr, 2);
	dmc_puts(str1);
	uint8_t len = strlen(myStr);
	for (; len < 32; len++)
	{
		__io_putchar('0');
	}
	dmc_puts2(myStr, str2);
}

void dmc_puthex(uint32_t value)
{
	char myStr[30];
	dmc_itoa(value, myStr, 16);
	if (strlen(myStr) == 1)
		__io_putchar('0');
	if (strlen(myStr) == 3)
		__io_putchar('0');
	if (strlen(myStr) == 5)
		dmc_puts("000");
	if (strlen(myStr) == 6)
		dmc_puts("00");
	if (strlen(myStr) == 7)
		__io_putchar('0');
	dmc_puts(myStr);
}

void dmc_puthex2(uint32_t value)
{
	char myStr[30];
	dmc_itoa(value, myStr, 16);
	if (strlen(myStr) < 2)
	{
		__io_putchar('0');
	}
	dmc_puts(myStr);
}

void dmc_puthex4(uint32_t value)
{
	char myStr[30];
	dmc_itoa(value, myStr, 16);
	uint8_t len = strlen(myStr);
	for (; len < 4; len++)
	{
		__io_putchar('0');
	}
	dmc_puts(myStr);
}

void dmc_puthex8(uint32_t value)
{
	char myStr[30];
	dmc_itoa(value, myStr, 16);
	uint8_t len = strlen(myStr);
	for (; len < 8; len++)
	{
		__io_putchar('0');
	}
	dmc_puts(myStr);
}

void dmc_puthexcr(uint32_t value)
{
	dmc_puthex(value);
	__io_putchar('\n');
}

void dmc_puthex2cr(uint32_t value)
{
	dmc_puthex2(value);
	__io_putchar('\n');
}

void dmc_puthex4cr(uint32_t value)
{
	dmc_puthex4(value);
	__io_putchar('\n');
}

void dmc_puthex8cr(uint32_t value)
{
	dmc_puthex8(value);
	__io_putchar('\n');
}

void dmc_puthexstr(uint32_t value, char* str)
{
	char myStr[30];
	dmc_itoa(value, myStr, 16);
	if (strlen(myStr) == 1)
		__io_putchar('0');
	if (strlen(myStr) == 3)
		__io_putchar('0');
	if (strlen(myStr) == 5)
		dmc_puts("000");
	if (strlen(myStr) == 6)
		dmc_puts("00");
	if (strlen(myStr) == 7)
		__io_putchar('0');
	dmc_puts2(myStr, str);
}

void dmc_puthex2str(uint32_t value, char* str)
{
	char myStr[30];
	dmc_itoa(value, myStr, 16);
	if (strlen(myStr) < 2)
	{
		__io_putchar('0');
	}
	dmc_puts2(myStr, str);
}

void dmc_puthex4str(uint32_t value, char* str)
{
	char myStr[30];
	dmc_itoa(value, myStr, 16);
	uint8_t len = strlen(myStr);
	for (; len < 4; len++)
	{
		__io_putchar('0');
	}
	dmc_puts2(myStr, str);
}

void dmc_puthex8str(uint32_t value, char* str)
{
	char myStr[30];
	dmc_itoa(value, myStr, 16);
	uint8_t len = strlen(myStr);
	for (; len < 8; len++)
	{
		__io_putchar('0');
	}
	dmc_puts2(myStr, str);
}

void dmc_putstrhexstr(char* str1, uint32_t value, char* str2)
{
	char myStr[30];
	dmc_itoa(value, myStr, 16);
	dmc_puts(str1);
	if (strlen(myStr) == 1)
		__io_putchar('0');
	if (strlen(myStr) == 3)
		__io_putchar('0');
	if (strlen(myStr) == 5)
		dmc_puts("000");
	if (strlen(myStr) == 6)
		dmc_puts("00");
	if (strlen(myStr) == 7)
		__io_putchar('0');
	dmc_puts2(myStr, str2);
}

void dmc_putstrhex2str(char* str1, uint32_t value, char* str2)
{
	char myStr[30];
	dmc_itoa(value, myStr, 16);
	dmc_puts(str1);
	if (strlen(myStr) < 2)
		__io_putchar('0');
	dmc_puts2(myStr, str2);
}

void dmc_putstrhex4str(char* str1, uint32_t value, char* str2)
{
	char myStr[30];
	dmc_itoa(value, myStr, 16);
	dmc_puts(str1);
	uint8_t len = strlen(myStr);
	for (; len < 4; len++)
	{
		__io_putchar('0');
	}
	dmc_puts2(myStr, str2);
}

void dmc_putstrhex8str(char* str1, uint32_t value, char* str2)
{
	char myStr[30];
	dmc_itoa(value, myStr, 16);
	dmc_puts(str1);
	uint8_t len = strlen(myStr);
	for (; len < 8; len++)
	{
		__io_putchar('0');
	}
	dmc_puts2(myStr, str2);
}

void dmc_puta(uint8_t v)
{
	__io_putchar(v);
}

void dmc_put2a(uint16_t v)
{
	__io_putchar(v >> 8);
	__io_putchar(v & 0xff);
}

void dmc_put2a2(uint16_t v1, uint16_t v2)
{
	__io_putchar(v1 >> 8);
	__io_putchar(v1 & 0xff);
	__io_putchar(v2 >> 8);
	__io_putchar(v2 & 0xff);
}

void dmc_put4a(uint32_t v)
{
	__io_putchar(v >> 24);
	__io_putchar((v >> 16) & 0xff);
	__io_putchar((v >> 8) & 0xff);
	__io_putchar(v & 0xff);
}

void dmc_putc(char c)
{
	__io_putchar(c);
}

void dmc_putccr(char c)
{
	__io_putchar(c);
	__io_putchar('\n');
}

void dmc_putcr(void)
{
	__io_putchar('\n');
}

void dmc_puts(char* str)
{
	while (*str)
	{
		__io_putchar(*str++);
	}
}

void dmc_putslen(char* str, uint16_t len)
{
	for (uint16_t i = 0; i < len; i++)
	{
		__io_putchar(*str++);
	}
}

void dmc_putscr(char* str)
{
	dmc_puts(str);
	__io_putchar('\n');
}

void dmc_puts2(char* str1, char* str2)
{
	while (*str1)
	{
		__io_putchar(*str1++);
	}
	while (*str2)
	{
		__io_putchar(*str2++);
	}
}

void dmc_puts2cr(char* str1, char* str2)
{
	dmc_puts2(str1, str2);
	__io_putchar('\n');
}

void dmc_puts3(char* str1, char* str2, char* str3)
{
	dmc_puts2(str1, str2);
	dmc_puts(str3);
}

void dmc_puts3cr(char* str1, char* str2, char* str3)
{
	dmc_puts3(str1, str2, str3);
	__io_putchar('\n');
}

void dmc_puts4(char* str1, char* str2, char* str3, char* str4)
{
	dmc_puts2(str1, str2);
	dmc_puts2(str3, str4);
}

void dmc_puts4cr(char* str1, char* str2, char* str3, char* str4)
{
	dmc_puts4(str1, str2, str3, str4);
	__io_putchar('\n');
}

