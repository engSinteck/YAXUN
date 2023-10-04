/*
 * MAX6675.h
 *
 *  Created on: Aug 21, 2023
 *      Author: rinaldo.santos
 */

#ifndef MAX6675_H_
#define MAX6675_H_

#include "main.h"

// ------------------------- Defines -------------------------
#define SSPORT1 CS_MAX_GPIO_Port       	// GPIO Port of Chip Select(Slave Select)
#define SSPIN1  CS_MAX_Pin  				// GPIO PIN  of Chip Select(Slave Select)

#define SSPORT2 LED2_GPIO_Port       	// GPIO Port of Chip Select(Slave Select)
#define SSPIN2  LED2_Pin  				// GPIO PIN  of Chip Select(Slave Select)
// ------------------------- Functions  ----------------------

float Max6675_Read_Temp(uint8_t channel);

#endif /* MAX6675_H_ */
