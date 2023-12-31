/*
 * MAX6675.c
 *
 *  Created on: Aug 21, 2023
 *      Author: rinaldo.santos
 */

#include "MAX6675.h"
#include "spi.h"

// ------------------- Variables ----------------
_Bool TCF_IRON = 0, TCF_AIR = 0;                            // Thermocouple Connection acknowledge Flag
uint8_t DATAMAX[2];                                    		// Raw Data from MAX6675

// ------------------- Functions ----------------
float Max6675_Read_Temp(uint8_t channel)
{
	float Temp=0;                                        	// Temperature Variable

	if(channel == 0)
		HAL_GPIO_WritePin(SSPORT1, SSPIN1, GPIO_PIN_RESET);       	// Low State for SPI Communication
	else
		HAL_GPIO_WritePin(SSPORT2, SSPIN2, GPIO_PIN_RESET);

	HAL_SPI_Receive(&hspi1, DATAMAX, 2, HAL_MAX_DELAY);            // DATA Transfer

	if(channel == 0)
		HAL_GPIO_WritePin(SSPORT1, SSPIN1, GPIO_PIN_SET);         // High State for SPI Communication
	else
		HAL_GPIO_WritePin(SSPORT2, SSPIN2, GPIO_PIN_SET);

	if(channel == 0)
		TCF_IRON =(((DATAMAX[1]|(DATAMAX[0]<<8))>>2) & 0x0001);        // State of Connecting
	else
		TCF_AIR = (((DATAMAX[1]|(DATAMAX[0]<<8))>>2) & 0x0001);

	Temp = ((((DATAMAX[1]|DATAMAX[0]<<8)))>>3);               // Temperature Data Extraction
	Temp *= 0.25;                                             // Data to Centigrade Conversation

	return Temp;
}
