/*
 * MAX6675.c
 *
 *  Created on: Aug 21, 2023
 *      Author: rinaldo.santos
 */

#include "MAX6675.h"
#include "spi.h"

// ------------------- Variables ----------------
_Bool TCF = 0;                                          		// Thermocouple Connection acknowledge Flag
uint8_t DATAMAX[2];                                    		// Raw Data from MAX6675

// ------------------- Functions ----------------
float Max6675_Read_Temp(void)
{
	float Temp=0;                                        	// Temperature Variable

	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
	 hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
	if (HAL_SPI_Init(&hspi1) != HAL_OK)
	{
		Error_Handler();
	}

	HAL_GPIO_WritePin(SSPORT,SSPIN, GPIO_PIN_RESET);       	// Low State for SPI Communication
	HAL_SPI_Receive(&hspi1, DATAMAX, 1, 50);                // DATA Transfer
	HAL_GPIO_WritePin(SSPORT, SSPIN, GPIO_PIN_SET);         // High State for SPI Communication

	TCF=(((DATAMAX[0]|(DATAMAX[1]<<8))>>2)& 0x0001);        // State of Connecting

	Temp=((((DATAMAX[0]|DATAMAX[1]<<8)))>>3);               // Temperature Data Extraction
	Temp*=0.25;                                           	// Data to Centigrade Conversation

	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	 hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	if (HAL_SPI_Init(&hspi1) != HAL_OK)
	{
		Error_Handler();
	}


	return Temp;
}
