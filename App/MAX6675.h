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
#define SSPORT CS_MAX_GPIO_Port       	// GPIO Port of Chip Select(Slave Select)
#define SSPIN  CS_MAX_Pin  				// GPIO PIN of Chip Select(Slave Select)
// ------------------------- Functions  ----------------------

float Max6675_Read_Temp(void);

#endif /* MAX6675_H_ */
