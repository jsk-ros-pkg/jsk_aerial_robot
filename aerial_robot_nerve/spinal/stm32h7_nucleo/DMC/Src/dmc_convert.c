#include "dmc_convert.h"

// C program for implementation of ftoa()
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "defines.h"
/**
 * C version 0.4 char* style "itoa":
 */
char * dmc_itoa(int value, char* result, int base)
{
	// check that the base if valid
	if (base < 2 || base > 36)
	{
		*result = '\0';
		return result;
	}

	char* ptr = result, *ptr1 = result, tmp_char;
	int tmp_value;

	do
	{
		tmp_value = value;
		value /= base;
		*ptr++ = "zyxwvutsrqponmlkjihgfedcba9876543210123456789abcdefghijklmnopqrstuvwxyz"[35
				+ (tmp_value - value * base)];
	}
	while (value);

	// Apply negative sign
	if (tmp_value < 0)
		*ptr++ = '-';
	*ptr-- = '\0';
	while (ptr1 < ptr)
	{
		tmp_char = *ptr;
		*ptr-- = *ptr1;
		*ptr1++ = tmp_char;
	}
	return result;
}

char * dmc_itoa_len_0(int value, char* result, int base, uint8_t length)
{
	dmc_itoa(value, result, base);
	// Add leading zeros
	uint8_t len = strlen(result);
	while (len < length)
	{
		for (int8_t i = len; i >= 0; i--)
		{
			result[i+1] = result[i];
		}
		result[0] = '0';
		len++;
	}
	return result;
}

char * dmc_itoa_len_space(int value, char* result, int base, uint8_t length)
{
	dmc_itoa(value, result, base);
	// Add leading spaces
	uint8_t len = strlen(result);
	while (len < length)
	{
		for (int8_t i = len; i >= 0; i--)
		{
			result[i+1] = result[i];
		}
		result[0] = ' ';
		len++;
	}
	return result;
}

// reverses a string 'str' of length 'len'
void reverse(char *str, int len)
{
	int i = 0, j = len - 1, temp;
	while (i < j)
	{
		temp = str[i];
		str[i] = str[j];
		str[j] = temp;
		i++;
		j--;
	}
}

// Converts a given integer x to string str[].  d is the number
// of digits required in output. If d is more than the number
// of digits in x, then 0s are added at the beginning.
int intToStr(int x, char str[], int d)
{
	int i = 0;
	while (x)
	{
		str[i++] = (x % 10) + '0';
		x = x / 10;
	}

	// If number of digits required is more, then
	// add 0s at the beginning
	while (i < d)
		str[i++] = '0';

	reverse(str, i);
	str[i] = '\0';
	return i;
}

// Converts a floating point number to string.
char * dmc_ftoa(float value, char *result, int afterpoint)
{
	// Extract integer part
	int ipart = (int) value;

	// Extract floating part
	float fpart = value - (float) ipart;

	// convert integer part to string
	int i = intToStr(ipart, result, 0);

	// check for display option after point
	if (afterpoint != 0)
	{
		result[i] = '.';  // add dot

		// Get the value of fraction part upto given no.
		// of points after dot. The third parameter is needed
		// to handle cases like 233.007
		fpart = fpart * pow(10, afterpoint);

		intToStr((int) fpart, result + i + 1, afterpoint);
	}
	return result;
}

char * dmc_utox(uint32_t value, char *result, int digits)
{
	static char hexstr[sizeof(value) * 2 + 1];
	char * p = hexstr + sizeof(hexstr) - 1;
	int x;

	memset(hexstr, '0', sizeof(hexstr));

	*p-- = '\0';

	while (value)
	{
		x = value % 16;
		if (x < 10)
		{
			*p-- = '0' + x;
		}
		else
		{
			*p-- = 'A' + x - 10;
		}
		value /= 16;
	}
	uint8_t start = 8 - digits;
	for (uint8_t i = 0; i <= digits; i++)
	{
		result[i] = hexstr[start + i];
	}

	return hexstr;
}

int32_t dmc_Dec2Ascii(char *pSrc, int32_t value)
{
	int16_t Length;
	char *pdst;
	char charval;
	int32_t CurrVal = value;
	int32_t tmpval;
	int32_t i;
	char tmparray[16];
	uint8_t idx = 0;

	Length = strlen(pSrc);
	pdst = pSrc + Length;

	if (0 == value)
	{
		*pdst++ = '0';
		*pdst++ = '\0';
		return 0;
	}

	if (CurrVal < 0)
	{
		*pdst++ = '-';
		CurrVal = -CurrVal;
	}
	/* insert the value */
	while (CurrVal > 0)
	{
		tmpval = CurrVal;
		CurrVal /= 10;
		tmpval = tmpval - CurrVal * 10;
		charval = '0' + tmpval;
		tmparray[idx++] = charval;
	}

	for (i = 0; i < idx; i++)
	{
		*pdst++ = tmparray[idx - i - 1];
	}
	*pdst++ = '\0';

	return 0;
}

// prints a number with 2 digits following the decimal place
// creates the string backwards, before printing it character-by-character from
// the end to the start
//
// Usage: dmc_floatDot3Toa(270.458, resultOut, 2)
//  Output: 270.45
void dmc_floatDot3Toa(double fVal, char* resultOut, uint8_t afterDP)
{
	uint negative = FALSE;
	if (fVal < 0.0)
	{
		fVal = -fVal;
		negative = TRUE;
	}

	// Rounding
    int iVal = (int) ((fVal + 0.0005) * 1000.0);
	if (afterDP == 2)
	{
		iVal = (int) ((fVal + 0.005) * 1000.0);
	}
	if (afterDP == 1)
	{
		iVal = (int) ((fVal + 0.05) * 1000.0);
	}
	if (afterDP == 0)
	{
		iVal = (int) ((fVal + 0.5) * 1000.0);
	}

	// Convert to Acsii string
	dmc_itoa(iVal, resultOut, 10);

	// For our purpose we support up to 3 dights after the decimal point.
    if (afterDP > 3)
    {
    	strcpy(resultOut, "afterDP <= 3");
    	return;
    }

    // Insert decimal point
	// "6158" -> "6.158"
	uint8_t len = strlen(resultOut);

	for (uint8_t i = len; i > 0; i--)
	{
		resultOut[i] = resultOut[i - 1];
	}
	resultOut[len - 3] = '.';
	resultOut[len + 1] = 0;

	// Insert minus sign, if negative number
	if (negative)
	{
		len++;
		for (uint8_t i = len; i > 0; i--)
		{
			resultOut[i] = resultOut[i - 1];
		}
		resultOut[0] = '-';
	}

	// Reduce length of string to given didgits after the decimal point
	if (afterDP == 0)
	{
		resultOut[len - 3] = 0;
	}
	if (afterDP == 1)
	{
		resultOut[len - 1] = 0;
	}
	if (afterDP == 2)
	{
		resultOut[len] = 0;
	}
}

void dmc_swap_case_len(char* str, uint16_t len)
{
	for (uint16_t i = 0; i < len; i++)
	{
		char c = str[i];
		if ((c >= 0x41) && (c <= 0x5A))
		{
			str[i] = c | 0x20;
		}
		if ((c >= 0x61) && (c <= 0x7A))
		{
			str[i] = c & ~0x20;
		}
	}
}

