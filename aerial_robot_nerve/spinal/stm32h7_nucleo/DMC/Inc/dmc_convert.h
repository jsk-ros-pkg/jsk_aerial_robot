#ifndef __DMC_CONVERT_H
#define __DMC_CONVERT_H
#ifdef __cplusplus
extern "C"
{
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"
#include "main.h"

char * dmc_itoa(int value, char *result, int base);
char * dmc_itoa_len_0(int value, char* result, int base, uint8_t length);
char * dmc_itoa_len_space(int value, char* result, int base, uint8_t length);
void reverse(char *str, int len);
int intToStr(int x, char str[], int d);
char * dmc_ftoa(float value, char *result, int afterpoint);
char * dmc_utox(uint32_t value, char *result, int digits);
int32_t dmc_Dec2Ascii(char *pSrc, int32_t value);
void dmc_swap_case_len(char* str, uint16_t len);

#ifdef __cplusplus
}
#endif
#endif /* __DMC_CONVERT_H */
