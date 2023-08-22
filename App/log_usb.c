/*
 * log_usb.c
 *
 *  Created on: Aug 21, 2023
 *      Author: rinaldo.santos
 */

#include "log_usb.h"

extern RTC_TimeTypeDef RTC_Time;
extern float temperature_K, temperature_iron, temperature_air;
extern uint32_t ADC_iron, ADC_air;
extern uint16_t pwm_iron, pwm_air;

char string_usb[512];
char string_log[512];

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

void Log_temp_iron(void)
{
	// HH:MM:SS - Termopar °C - ADC - PWM
	sprintf(string_log, "%02d:%02d:%02d - %0.1f°C - %ld [%0.2fmV] - %d - %0.0F°C\n\r",
			RTC_Time.Hours,
			RTC_Time.Minutes,
			RTC_Time.Seconds,
			temperature_K,
			ADC_iron,
			(float)ADC_iron * ((float)3300.0/4095.0),
			pwm_iron,
			temperature_iron );
	LogDebug(string_log);
}

void Log_temp_gun(void)
{
	// HH:MM:SS - Termopar °C - ADC - PWM
	sprintf(string_log, "%02d:%02d:%02d - %0.1f°C - %ld [%0.2fmV] - %d - %0.0F°C\n\r",
			RTC_Time.Hours,
			RTC_Time.Minutes,
			RTC_Time.Seconds,
			temperature_K,
			ADC_air,
			(float)ADC_air * ((float)3300.0/4095.0),
			pwm_air,
			temperature_air );
	LogDebug(string_log);

}
