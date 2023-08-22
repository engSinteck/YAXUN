/*
 * log_usb.h
 *
 *  Created on: Aug 21, 2023
 *      Author: rinaldo.santos
 */

#ifndef LOG_USB_H_
#define LOG_USB_H_

#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include "usbd_cdc_if.h"

void LogDebug(const char* fmt, ...);
void Log_temp_iron(void);
void Log_temp_gun(void);

#endif /* LOG_USB_H_ */
