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

uint32_t temp_iron, temp_air;
float temperature_iron, temperature_air;

#define IRON_POW_MAX_MEASURABLE_MILIVOLTS (float)(845.30)
#define IRON_POW_TABLE_EXPAND_COEF        10
#define IRON_POW_TABLE_LENGTH             56
#define IRON_POW_TABLE2_LENGTH            (IRON_POW_TABLE_LENGTH-1)*IRON_POW_TABLE_EXPAND_COEF+1 //LENGTH OF INTERPOLATED VECTOR

const float IRON_TABLE_WATTS[IRON_POW_TABLE_LENGTH][2] = {
	{   00.00,    0},		//this line was inserted only for test
	{    0.60,   10},
	{    1.70,   20},
	{    3.50,   30},
	{    7.10,   40},
	{   11.40,   50},
	{   19.50,   60},
	{   31.50,   70},
	{   45.00,   80},
	{   61.00,   90},		// 10
	{   79.00,  100},
	{   96.60,  110},
	{  115.90,  120},
	{  134.40,  130},
	{  155.70,  140},
	{  173.70,  150},
	{  198.00,  160},
	{  220.30,  170},
	{  241.80,  180},
	{  261.50,  190},		// 20
	{  280.00,  200},
	{  303.50,  210},
	{  322.30,  220},
	{  340.00,  230},
	{  361.60,  240},
	{  381.80,  250},
	{  398.70,  260},
	{  413.10,  270},
	{  431.40,  280},
	{  451.00,  290},		// 30
	{  469.00,  300},
	{  484.60,  310},
	{  501.70,  320},
	{  517.30,  330},
	{  532.60,  340},
	{  545.40,  350},
	{  561.70,  360},
	{  568.70,  370},
	{  575.40,  380},
	{  589.80,  390},		// 40
	{  602.60,  400},
	{  623.60,  410},
	{  638.10,  420},
	{  650.90,  430},
	{  670.80,  440},
	{  689.10,  450},
	{  704.90,  460},
	{  722.60,  470},
	{  739.00,  480},
	{  756.70,  490},		// 50
	{  771.70,  500},
	{  786.20,  510},
	{  803.60,  520},
	{  820.80,  530},
	{  831.70,  540},
	{  845.30,  550}
};
float IRON_TABLE_WATTS2[IRON_POW_TABLE2_LENGTH][2];   //generated table

void IRON_TABLE_Interpolation(void)
{
	uint32_t pwdTableIndex = 0;
	uint32_t pwdTableIndex2 = 0;
	float pwdTableDelta[2];

	memset(IRON_TABLE_WATTS2, 0, sizeof(IRON_TABLE_WATTS2));

	for(pwdTableIndex = 0; pwdTableIndex <= IRON_POW_TABLE_LENGTH - 2; pwdTableIndex++){
		pwdTableDelta[0] = (float)(IRON_TABLE_WATTS[pwdTableIndex+1][0] - IRON_TABLE_WATTS[pwdTableIndex][0]) / (float)(IRON_POW_TABLE_EXPAND_COEF);
		pwdTableDelta[1] = (float)(IRON_TABLE_WATTS[pwdTableIndex+1][1] - IRON_TABLE_WATTS[pwdTableIndex][1]) / (float)(IRON_POW_TABLE_EXPAND_COEF);

		for(pwdTableIndex2 = 0; pwdTableIndex2 <= IRON_POW_TABLE_EXPAND_COEF  - 1; pwdTableIndex2++){
			IRON_TABLE_WATTS2[pwdTableIndex*IRON_POW_TABLE_EXPAND_COEF + pwdTableIndex2][0] = IRON_TABLE_WATTS[pwdTableIndex][0] + pwdTableIndex2 * pwdTableDelta[0];
			IRON_TABLE_WATTS2[pwdTableIndex*IRON_POW_TABLE_EXPAND_COEF + pwdTableIndex2][1] = IRON_TABLE_WATTS[pwdTableIndex][1] + pwdTableIndex2 * pwdTableDelta[1];
		}
	}
	IRON_TABLE_WATTS2[IRON_POW_TABLE2_LENGTH-1][0] = IRON_TABLE_WATTS[IRON_POW_TABLE_LENGTH-1][0];
	IRON_TABLE_WATTS2[IRON_POW_TABLE2_LENGTH-1][1] = IRON_TABLE_WATTS[IRON_POW_TABLE_LENGTH-1][1];
}

#define AIR_POW_MAX_MEASURABLE_MILIVOLTS (float)(3200.00)
#define AIR_POW_TABLE_EXPAND_COEF        10
#define AIR_POW_TABLE_LENGTH             56
#define AIR_POW_TABLE2_LENGTH            (AIR_POW_TABLE_LENGTH-1)*AIR_POW_TABLE_EXPAND_COEF+1 //LENGTH OF INTERPOLATED VECTOR

const float AIR_TABLE_WATTS[AIR_POW_TABLE_LENGTH][2] = {
	{   00.00,    0},		//this line was inserted only for test
	{   30.00,   10},
	{   40.00,   20},
	{   50.00,   30},
	{   60.00,   40},
	{   70.00,   50},
	{   80.00,   60},
	{   90.00,   70},
	{  100.00,   80},
	{  110.00,   90},		// 10
	{  120.00,  100},
	{  130.00,  110},
	{  140.00,  120},
	{  150.00,  130},
	{  160.00,  140},
	{  170.00,  150},
	{  180.00,  160},
	{  190.00,  170},
	{  200.00,  180},
	{  250.00,  190},		// 20
	{  300.00,  200},
	{  350.00,  210},
	{  400.00,  220},
	{  450.00,  230},
	{  500.00,  240},
	{  550.00,  250},
	{  600.00,  260},
	{  650.00,  270},
	{  700.00,  280},
	{  750.00,  290},		// 30
	{  800.00,  300},
	{  850.00,  310},
	{  900.00,  320},
	{ 1000.00,  330},
	{ 1100.00,  340},
	{ 1200.00,  350},
	{ 1300.00,  360},
	{ 1400.00,  370},
	{ 1500.00,  380},
	{ 1600.00,  390},		// 40
	{ 1700.00,  400},
	{ 1800.00,  410},
	{ 1900.00,  420},
	{ 2000.00,  430},
	{ 2100.00,  440},
	{ 2200.00,  450},
	{ 2300.00,  460},
	{ 2400.00,  470},
	{ 2500.00,  480},
	{ 2600.00,  490},		// 50
	{ 2700.00,  500},
	{ 2800.00,  510},
	{ 2900.00,  520},
	{ 3000.00,  530},
	{ 3100.00,  540},
	{ 3200.00,  550}
};
float AIR_TABLE_WATTS2[AIR_POW_TABLE2_LENGTH][2];   //generated table

void AIR_TABLE_Interpolation(void)
{
	uint32_t pwdTableIndex = 0;
	uint32_t pwdTableIndex2 = 0;
	float pwdTableDelta[2];

	memset(AIR_TABLE_WATTS2, 0, sizeof(AIR_TABLE_WATTS2));

	for(pwdTableIndex = 0; pwdTableIndex <= IRON_POW_TABLE_LENGTH - 2; pwdTableIndex++){
		pwdTableDelta[0] = (float)(AIR_TABLE_WATTS[pwdTableIndex+1][0] - AIR_TABLE_WATTS[pwdTableIndex][0]) / (float)(AIR_POW_TABLE_EXPAND_COEF);
		pwdTableDelta[1] = (float)(AIR_TABLE_WATTS[pwdTableIndex+1][1] - AIR_TABLE_WATTS[pwdTableIndex][1]) / (float)(AIR_POW_TABLE_EXPAND_COEF);

		for(pwdTableIndex2 = 0; pwdTableIndex2 <= IRON_POW_TABLE_EXPAND_COEF  - 1; pwdTableIndex2++){
			AIR_TABLE_WATTS2[pwdTableIndex*AIR_POW_TABLE_EXPAND_COEF + pwdTableIndex2][0] = AIR_TABLE_WATTS[pwdTableIndex][0] + pwdTableIndex2 * pwdTableDelta[0];
			AIR_TABLE_WATTS2[pwdTableIndex*AIR_POW_TABLE_EXPAND_COEF + pwdTableIndex2][1] = AIR_TABLE_WATTS[pwdTableIndex][1] + pwdTableIndex2 * pwdTableDelta[1];
		}
	}
	AIR_TABLE_WATTS2[AIR_POW_TABLE2_LENGTH-1][0] = AIR_TABLE_WATTS[AIR_POW_TABLE_LENGTH-1][0];
	AIR_TABLE_WATTS2[AIR_POW_TABLE2_LENGTH-1][1] = AIR_TABLE_WATTS[AIR_POW_TABLE_LENGTH-1][1];
}

void ADC_MeasurementCorrection(void)
{
    uint32_t i = 0;

    //Approximate IRON  Power
    for (i = 0; i< IRON_POW_TABLE2_LENGTH; i++){
    	if( ((float)ADC_iron * (float)(3300.0/4095.0) ) >= IRON_TABLE_WATTS2[i][0] ) { 	//compare in millivolts
    		temperature_iron = IRON_TABLE_WATTS2[i][1];                  //watts
        }
    }
    //Approximate Air  Power
    for (i = 0; i< AIR_POW_TABLE2_LENGTH; i++){
    	if( ((float)ADC_air * (float)(3300.0/4095.0) ) >= AIR_TABLE_WATTS2[i][0] ) { 	//compare in millivolts
    		temperature_air = AIR_TABLE_WATTS2[i][1];                  //watts
        }
    }
}

#endif /* TAB_TEMP_H_ */
