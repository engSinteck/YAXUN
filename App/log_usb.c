/*
 * log_usb.c
 *
 *  Created on: Aug 21, 2023
 *      Author: rinaldo.santos
 */

#include "log_usb.h"

char string_usb[512];

void HAL_printf_valist(const char *fmt, va_list argp)
{
  if (vsprintf(string_usb, fmt, argp) > 0) {
    CDC_Transmit_FS((uint8_t*)string_usb, strlen(string_usb));				// send message via USB CDC
  } else {
	CDC_Transmit_FS((uint8_t*)"E - Print\n", 14);
  }
  HAL_Delay(10);
}

void HAL_printf(const char *fmt, ...)
{
  va_list argp;

  va_start(argp, fmt);
  HAL_printf_valist(fmt, argp);
  va_end(argp);
}

void logUSB(const char *fmt, va_list argp)
{
	HAL_printf_valist(fmt, argp);
}

void LogDebug(const char* fmt, ...)
{
	va_list argp;

	va_start(argp, fmt);
	logUSB(fmt, argp);
	va_end(argp);
}
