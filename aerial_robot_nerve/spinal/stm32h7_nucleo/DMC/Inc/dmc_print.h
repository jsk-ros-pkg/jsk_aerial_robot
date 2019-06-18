#ifndef __DMC_PRINT_H
#define __DMC_PRINT_H
#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "main.h"

void debugPrint(UART_HandleTypeDef *huart, char _out[]);
void debugPrintln(UART_HandleTypeDef *huart, char _out[]);
void VprintUart2(const char *fmt, va_list argp);
void VprintUart3(const char *fmt, va_list argp);
void printfUart2(const char *fmt, ...);
void printfUart3(const char *fmt, ...);
void enableFlushAfterPrintf(void);
void dmc_putint(int value);
void dmc_putintcr(int value);
void dmc_putint2(int value, char c);
void dmc_putint2cr(int value, char c);
void dmc_putint4(int value, char c);
void dmc_putint4cr(int value, char c);
void dmc_putint6(int value, char c);
void dmc_putint6cr(int value, char c);
void dmc_putint8(int value, char c);
void dmc_putint8cr(int value, char c);
void dmc_putintstr(int value, char* str);
void dmc_putintstrcr(int value, char* str);
void dmc_putstrint(char* str, int value);
void dmc_putstrintcr(char* str, int value);
void dmc_putstrintstr(char* str1, int value, char* str2);
void dmc_putbin8cr(uint32_t value);
void dmc_putbin16cr(uint32_t value);
void dmc_putbin32cr(uint32_t value);
void dmc_putbin8str(uint32_t value, char* str);
void dmc_putbin16str(uint32_t value, char* str);
void dmc_putbin32str(uint32_t value, char* str);
void dmc_putstrbin8str(char* str1, uint32_t value, char* str2);
void dmc_putstrbin16str(char* str1, uint32_t value, char* str2);
void dmc_putstrbin32str(char* str1, uint32_t value, char* str2);
void dmc_puthex(uint32_t value);
void dmc_puthex2(uint32_t value);
void dmc_puthex4(uint32_t value);
void dmc_puthex8(uint32_t value);
void dmc_puthexcr(uint32_t value);
void dmc_puthex2cr(uint32_t value);
void dmc_puthex4cr(uint32_t value);
void dmc_puthex8cr(uint32_t value);
void dmc_puthexstr(uint32_t value, char* str);
void dmc_puthex2str(uint32_t value, char* str);
void dmc_puthex4str(uint32_t value, char* str);
void dmc_puthex8str(uint32_t value, char* str);
void dmc_putstrhexstr(char* str1, uint32_t value, char* str2);
void dmc_putstrhex2str(char* str1, uint32_t value, char* str2);
void dmc_putstrhex4str(char* str1, uint32_t value, char* str2);
void dmc_putstrhex8str(char* str1, uint32_t value, char* str2);
void dmc_puta(uint8_t v);
void dmc_put2a(uint16_t v);
void dmc_put2a2(uint16_t v1, uint16_t v2);
void dmc_put4a(uint32_t v);
void dmc_putc(char c);
void dmc_putccr(char c);
void dmc_putcr(void);
void dmc_puts(char* str);
void dmc_putslen(char* str, uint16_t len);
void dmc_putscr(char* str);
void dmc_puts2(char* str1, char* str2);
void dmc_puts2cr(char* str1, char* str2);
void dmc_puts3(char* str1, char* str2, char* str3);
void dmc_puts3cr(char* str1, char* str2, char* str3);
void dmc_puts4(char* str1, char* str2, char* str3, char* str4);
void dmc_puts4cr(char* str1, char* str2, char* str3, char* str4);

#ifdef __cplusplus
}
#endif
#endif /* __DMC_PRINT_H */
