/*
 * tab_temp.h
 *
 *  Created on: Aug 15, 2023
 *      Author: rinaldo.santos
 */

#ifndef TAB_TEMP_H_
#define TAB_TEMP_H_

#include "main.h"
#include "string.h"
#include "stdio.h"

extern uint32_t ADC_iron, ADC_air;

uint32_t temp_iron;
uint32_t temp_air;

//Table autogenerator driver power
#define AIRTEMP_POW_TABLE_EXPAND_COEF       10
#define AIRTEMP_POW_TABLE_LENGTH            53
#define AIRTEMP_POW_TABLE2_LENGTH           (AIRTEMP_POW_TABLE_LENGTH-1)*(AIRTEMP_POW_TABLE_EXPAND_COEF+1) //LENGTH OF INTERPOLATED VECTOR

//Voltage Checking
const uint32_t AIRTEMP_TABLE[AIRTEMP_POW_TABLE_LENGTH][2] = {
   {    0,   0 },
   {   20,  10 },
   {   40,  20 },
   {   60,  30 },
   {   80,  40 },
   {  100,  50 },
   {  120,  60 },
   {  140,  70 },
   {  160,  80 },
   {  180,  90 },	// 10
   {  200, 100 },
   {  220, 110 },
   {  240, 120 },
   {  260, 130 },
   {  280, 140 },
   {  300, 150 },
   {  350, 160 },
   {  400, 170 },
   {  450, 180 },
   {  500, 190 },	// 20
   {  550, 200 },
   {  600, 210 },
   {  650, 220 },
   {  700, 230 },
   {  750, 240 },
   {  800, 250 },
   {  850, 260 },
   {  900, 270 },
   {  950, 280 },
   { 1000, 290 },	// 30
   { 1050, 300 },
   { 1100, 310 },
   { 1200, 320 },
   { 1300, 330 },
   { 1400, 340 },
   { 1500, 350 },
   { 1600, 360 },
   { 1700, 370 },
   { 1800, 380 },
   { 1900, 390 },	// 40
   { 2000, 400 },
   { 2200, 410 },
   { 2400, 420 },
   { 2600, 430 },
   { 2800, 440 },
   { 3000, 450 },
   { 3200, 460 },
   { 3400, 470 },
   { 3600, 480 },
   { 3700, 490 },	// 50
   { 3800, 500 },
   { 3900, 510 },
   { 4050, 520 }
};
uint32_t AIRTEMP_TABLE_2[AIRTEMP_POW_TABLE2_LENGTH][2]; //generated table

void AIRTEMP_TABLE_Interpolation(void)
{
  uint32_t pwdTableIndex  =  0; //Original table
  uint32_t pwdTableIndex2 = 0; //Interpolated table
  uint32_t tableDelta[2];

  memset(AIRTEMP_TABLE_2, 0, sizeof(AIRTEMP_TABLE_2));

  for(pwdTableIndex = 0; pwdTableIndex <= AIRTEMP_POW_TABLE_LENGTH - 2; pwdTableIndex++){
    tableDelta[0] = (uint32_t)(AIRTEMP_TABLE[pwdTableIndex+1][0] - AIRTEMP_TABLE[pwdTableIndex][0]) / (uint32_t)(AIRTEMP_POW_TABLE_EXPAND_COEF);
    tableDelta[1] = (uint32_t)(AIRTEMP_TABLE[pwdTableIndex+1][1] - AIRTEMP_TABLE[pwdTableIndex][1]) / (uint32_t)(AIRTEMP_POW_TABLE_EXPAND_COEF);

    for(pwdTableIndex2 = 0; pwdTableIndex2 <= AIRTEMP_POW_TABLE_EXPAND_COEF - 1; pwdTableIndex2++){

    	AIRTEMP_TABLE_2[pwdTableIndex*AIRTEMP_POW_TABLE_EXPAND_COEF + pwdTableIndex2][0] = AIRTEMP_TABLE[pwdTableIndex][0] + pwdTableIndex2 * tableDelta[0];
    	AIRTEMP_TABLE_2[pwdTableIndex*AIRTEMP_POW_TABLE_EXPAND_COEF + pwdTableIndex2][1] = AIRTEMP_TABLE[pwdTableIndex][1] + pwdTableIndex2 * tableDelta[1];
    }
  }
  AIRTEMP_TABLE_2[AIRTEMP_POW_TABLE2_LENGTH-1][0] = AIRTEMP_TABLE[AIRTEMP_POW_TABLE_LENGTH-1][0];
  AIRTEMP_TABLE_2[AIRTEMP_POW_TABLE2_LENGTH-1][1] = AIRTEMP_TABLE[AIRTEMP_POW_TABLE_LENGTH-1][1];
}

//Table autogenerator driver power
#define IRON_POW_TABLE_EXPAND_COEF       10
#define IRON_POW_TABLE_LENGTH            53
#define IRON_POW_TABLE2_LENGTH           (IRON_POW_TABLE_LENGTH-1)*(IRON_POW_TABLE_EXPAND_COEF+1) //LENGTH OF INTERPOLATED VECTOR

//Voltage Checking
const uint32_t IRON_TABLE[IRON_POW_TABLE_LENGTH][2] = {
   {    0,   0 },
   {   20,  10 },
   {   40,  20 },
   {   60,  30 },
   {   80,  40 },
   {  100,  50 },
   {  120,  60 },
   {  140,  70 },
   {  160,  80 },
   {  180,  90 },	// 10
   {  200, 100 },
   {  220, 110 },
   {  240, 120 },
   {  260, 130 },
   {  280, 140 },
   {  300, 150 },
   {  350, 160 },
   {  400, 170 },
   {  450, 180 },
   {  500, 190 },	// 20
   {  550, 200 },
   {  600, 210 },
   {  650, 220 },
   {  700, 230 },
   {  750, 240 },
   {  800, 250 },
   {  850, 260 },
   {  900, 270 },
   {  950, 280 },
   { 1000, 290 },	// 30
   { 1050, 300 },
   { 1100, 310 },
   { 1200, 320 },
   { 1300, 330 },
   { 1400, 340 },
   { 1500, 350 },
   { 1600, 360 },
   { 1700, 370 },
   { 1800, 380 },
   { 1900, 390 },	// 40
   { 2000, 400 },
   { 2200, 410 },
   { 2400, 420 },
   { 2600, 430 },
   { 2800, 440 },
   { 3000, 450 },
   { 3200, 460 },
   { 3400, 470 },
   { 3600, 480 },
   { 3700, 490 },	// 50
   { 3800, 500 },
   { 3900, 510 },
   { 4050, 520 }
};
uint32_t IRON_TABLE_2[IRON_POW_TABLE2_LENGTH][2]; //generated table

void IRON_TABLE_Interpolation(void)
{
  uint32_t pwdTableIndex  = 0; //Original table
  uint32_t pwdTableIndex2 = 0; //Interpolated table
  uint32_t tableDelta[2];

  memset(IRON_TABLE_2, 0, sizeof(IRON_TABLE_2));

  for(pwdTableIndex = 0; pwdTableIndex <= IRON_POW_TABLE_LENGTH - 2; pwdTableIndex++){
    tableDelta[0] = (uint32_t)(IRON_TABLE[pwdTableIndex+1][0] - IRON_TABLE[pwdTableIndex][0]) / (uint32_t)(IRON_POW_TABLE_EXPAND_COEF);
    tableDelta[1] = (uint32_t)(IRON_TABLE[pwdTableIndex+1][1] - IRON_TABLE[pwdTableIndex][1]) / (uint32_t)(IRON_POW_TABLE_EXPAND_COEF);

    for(pwdTableIndex2 = 0; pwdTableIndex2 <= IRON_POW_TABLE_EXPAND_COEF - 1; pwdTableIndex2++){

    	IRON_TABLE_2[pwdTableIndex*AIRTEMP_POW_TABLE_EXPAND_COEF + pwdTableIndex2][0] = IRON_TABLE[pwdTableIndex][0] + pwdTableIndex2 * tableDelta[0];
    	IRON_TABLE_2[pwdTableIndex*AIRTEMP_POW_TABLE_EXPAND_COEF + pwdTableIndex2][1] = IRON_TABLE[pwdTableIndex][1] + pwdTableIndex2 * tableDelta[1];
    }
  }
  IRON_TABLE_2[IRON_POW_TABLE2_LENGTH-1][0] = IRON_TABLE[IRON_POW_TABLE_LENGTH-1][0];
  IRON_TABLE_2[IRON_POW_TABLE2_LENGTH-1][1] = IRON_TABLE[IRON_POW_TABLE_LENGTH-1][1];
}

void ADC_MeasurementCorrection(void)
{
    uint32_t i = 0;

    //Approximate Iron temperature
    for (i = 0; i< IRON_POW_TABLE2_LENGTH; i++){
    	if( ADC_iron >= IRON_TABLE_2[i][0]) { 				//compare in millivolts
    		temp_iron = IRON_TABLE_2[i][1];
        }
    }
    //Approximate Air temperature
    for (i = 0; i< AIRTEMP_POW_TABLE2_LENGTH; i++){
    	if( ADC_air >= AIRTEMP_TABLE_2[i][0]) { 	//compare in millivolts
            temp_air = AIRTEMP_TABLE_2[i][1];
        }
    }
}

#endif /* TAB_TEMP_H_ */
