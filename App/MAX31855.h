/*
 * MAX31855.h
 *
 *  Created on: Aug 28, 2023
 *      Author: rinaldo.santos
 */

#ifndef MAX31855_H_
#define MAX31855_H_

#include "main.h"

// ------------------------- Defines -------------------------
extern uint8_t Error;	   			// Error Detection - 1-> No Connection / 2-> Short to GND / 4-> Short to VCC
#define SSPORT CS_MAX_GPIO_Port		// GPIO Port of Chip Select(Slave Select)
#define SSPIN  CS_MAX_Pin  			// GPIO PIN of Chip Select(Slave Select)
// ------------------------- Functions  ----------------------
float Max31855_Read_Temp(void);

#endif /* MAX31855_H_ */
