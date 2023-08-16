/*
 * tab_temp.h
 *
 *  Created on: Aug 15, 2023
 *      Author: rinaldo.santos
 */

#ifndef TAB_TEMP_H_
#define TAB_TEMP_H_

extern float temp_iron, temp_air;
extern uint32_t ADC_iron, ADC_air;

//Table autogenerator driver power
#define AIRTEMP_POW_TABLE_EXPAND_COEF       20
#define AIRTEMP_POW_TABLE_LENGTH            53
#define AIRTEMP_POW_TABLE2_LENGTH           (AIRTEMP_POW_TABLE_LENGTH-1)*(AIRTEMP_POW_TABLE_EXPAND_COEF+1) //LENGTH OF INTERPOLATED VECTOR

//Voltage Checking
const float AIRTEMP_TABLE[AIRTEMP_POW_TABLE_LENGTH][2] = {
   {    0.0,   0 } ,
   {  120.0,  10 },
   {  216.0,  20 },
   {  294.0,  30 },
   {  362.0,  40 },
   {  418.0,  50 },
   {  470.0,  60 },
   {  520.0,  70 },
   {  566.0,  80 },
   {  610.0,  90 },	// 10
   {  650.0, 100 },
   {  688.0, 110 },
   {  724.0, 120 },
   {  760.0, 130 },
   {  796.0, 140 },
   {  832.0, 150 },
   {  864.0, 160 },
   {  892.0, 170 },
   {  924.0, 180 },
   {  954.0, 190 },	// 20
   {  982.0, 200 },
   { 1012.0, 210 },
   { 1038.0, 220 },
   { 1064.0, 230 },
   { 1090.0, 240 },
   { 1118.0, 250 },
   { 1142.0, 260 },
   { 1170.0, 270 },
   { 1190.0, 280 },
   { 1216.0, 290 },	// 30
   { 1242.0, 300 },
   { 1264.0, 310 },
   { 1288.0, 320 },
   { 1310.0, 330 },
   { 1332.0, 340 },
   { 1354.0, 350 },
   { 1376.0, 360 },
   { 1396.0, 370 },
   { 1418.0, 380 },
   { 1440.0, 390 },	// 40
   { 1460.0, 400 },
   { 1480.0, 410 },
   { 1500.0, 420 },
   { 1520.0, 430 },
   { 1540.0, 440 },
   { 1558.0, 450 },
   { 1578.0, 460 },
   { 1596.0, 470 },
   { 1616.0, 480 },
   { 1634.0, 490 },	// 50
   { 1652.0, 500 },
   { 1670.0, 510 },
   { 1690.0, 520 }
};
float AIRTEMP_TABLE_2[AIRTEMP_POW_TABLE2_LENGTH][2]; //generated table

void AIRTEMP_TABLE_Interpolation(void)
{
  uint32_t pwdTableIndex = 0; //Original table
  uint32_t pwdTableIndex2 =0; //Interpolated table

  memset(AIRTEMP_TABLE_2, 0, sizeof(AIRTEMP_TABLE_2));
  float tableDelta[2];

  for(pwdTableIndex = 0; pwdTableIndex <= AIRTEMP_POW_TABLE_LENGTH -2; pwdTableIndex++){
    tableDelta[0] = (float)(AIRTEMP_TABLE[pwdTableIndex+1][0] - AIRTEMP_TABLE[pwdTableIndex][0]) / (float)(AIRTEMP_POW_TABLE_EXPAND_COEF);
    tableDelta[1] = (float)(AIRTEMP_TABLE[pwdTableIndex+1][1] - AIRTEMP_TABLE[pwdTableIndex][1]) / (float)(AIRTEMP_POW_TABLE_EXPAND_COEF);

    for(pwdTableIndex2 = 0; pwdTableIndex2 <= AIRTEMP_POW_TABLE_EXPAND_COEF  -1   ; pwdTableIndex2++){

    	AIRTEMP_TABLE_2[pwdTableIndex*AIRTEMP_POW_TABLE_EXPAND_COEF + pwdTableIndex2][0] = AIRTEMP_TABLE[pwdTableIndex][0] + pwdTableIndex2 * tableDelta[0];
    	AIRTEMP_TABLE_2[pwdTableIndex*AIRTEMP_POW_TABLE_EXPAND_COEF + pwdTableIndex2][1] = AIRTEMP_TABLE[pwdTableIndex][1] + pwdTableIndex2 * tableDelta[1];
    }
  }
  AIRTEMP_TABLE_2[AIRTEMP_POW_TABLE2_LENGTH-1][0] = AIRTEMP_TABLE[AIRTEMP_POW_TABLE_LENGTH-1][0];
  AIRTEMP_TABLE_2[AIRTEMP_POW_TABLE2_LENGTH-1][1] = AIRTEMP_TABLE[AIRTEMP_POW_TABLE_LENGTH-1][1];
}

//Table autogenerator driver power
#define IRON_POW_TABLE_EXPAND_COEF       20
#define IRON_POW_TABLE_LENGTH            53
#define IRON_POW_TABLE2_LENGTH           (IRON_POW_TABLE_LENGTH-1)*(IRON_POW_TABLE_EXPAND_COEF+1) //LENGTH OF INTERPOLATED VECTOR

//Voltage Checking
const float IRON_TABLE[IRON_POW_TABLE_LENGTH][2] = {
   {    0.0,   0 } ,
   {  120.0,  10 },
   {  216.0,  20 },
   {  294.0,  30 },
   {  362.0,  40 },
   {  418.0,  50 },
   {  470.0,  60 },
   {  520.0,  70 },
   {  566.0,  80 },
   {  610.0,  90 },	// 10
   {  650.0, 100 },
   {  688.0, 110 },
   {  724.0, 120 },
   {  760.0, 130 },
   {  796.0, 140 },
   {  832.0, 150 },
   {  864.0, 160 },
   {  892.0, 170 },
   {  924.0, 180 },
   {  954.0, 190 },	// 20
   {  982.0, 200 },
   { 1012.0, 210 },
   { 1038.0, 220 },
   { 1064.0, 230 },
   { 1090.0, 240 },
   { 1118.0, 250 },
   { 1142.0, 260 },
   { 1170.0, 270 },
   { 1190.0, 280 },
   { 1216.0, 290 },	// 30
   { 1242.0, 300 },
   { 1264.0, 310 },
   { 1288.0, 320 },
   { 1310.0, 330 },
   { 1332.0, 340 },
   { 1354.0, 350 },
   { 1376.0, 360 },
   { 1396.0, 370 },
   { 1418.0, 380 },
   { 1440.0, 390 },	// 40
   { 1460.0, 400 },
   { 1480.0, 410 },
   { 1500.0, 420 },
   { 1520.0, 430 },
   { 1540.0, 440 },
   { 1558.0, 450 },
   { 1578.0, 460 },
   { 1596.0, 470 },
   { 1616.0, 480 },
   { 1634.0, 490 },	// 50
   { 1652.0, 500 },
   { 1670.0, 510 },
   { 1690.0, 520 }
};
float IRON_TABLE_2[IRON_POW_TABLE2_LENGTH][2]; //generated table

void IRON_TABLE_Interpolation(void)
{
  uint32_t pwdTableIndex = 0; //Original table
  uint32_t pwdTableIndex2 =0; //Interpolated table

  memset(IRON_TABLE_2, 0, sizeof(IRON_TABLE_2));
  float tableDelta[2];

  for(pwdTableIndex = 0; pwdTableIndex <= IRON_POW_TABLE_LENGTH -2; pwdTableIndex++){
    tableDelta[0] = (float)(IRON_TABLE[pwdTableIndex+1][0] - IRON_TABLE[pwdTableIndex][0]) / (float)(IRON_POW_TABLE_EXPAND_COEF);
    tableDelta[1] = (float)(IRON_TABLE[pwdTableIndex+1][1] - IRON_TABLE[pwdTableIndex][1]) / (float)(IRON_POW_TABLE_EXPAND_COEF);

    for(pwdTableIndex2 = 0; pwdTableIndex2 <= IRON_POW_TABLE_EXPAND_COEF  -1   ; pwdTableIndex2++){

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
    	if( ((float)ADC_iron * (float)(3300.0/4095.0) )  >= IRON_TABLE_2[i][0]) { 	//compare in millivolts
            temp_iron = IRON_TABLE_2[i][1];
        }
    }
    //Approximate Air temperature
    for (i = 0; i< AIRTEMP_POW_TABLE2_LENGTH; i++){
    	if( ((float)ADC_air * (float)(3300.0/4095.0) )  >= AIRTEMP_TABLE_2[i][0]) { 	//compare in millivolts
            temp_air = AIRTEMP_TABLE_2[i][1];
        }
    }
}

#endif /* TAB_TEMP_H_ */
