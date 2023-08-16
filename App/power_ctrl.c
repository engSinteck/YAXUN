/*
 * power_ctrl.c
 *
 *  Created on: Aug 15, 2023
 *      Author: rinaldo.santos
 */

#include "main.h"
#include "tim.h"
#include "power_ctrl.h"

extern float temp_iron, temp_air, target_iron, target_air;
extern uint32_t flag_iron, flag_air;

uint32_t timer_iron = 0, timer_air = 0;
uint16_t pwm_iron = 0, pwm_air = 0;

void control_temperature_iron(void)
{
	if(HAL_GetTick() - timer_iron > 250) {
		timer_iron = HAL_GetTick();
		//
		if((float)temp_iron > target_iron) {
			if(pwm_iron >= 1) pwm_iron--;
			__HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_1, pwm_iron);	// PWM_CH1 = 0 IRON
			flag_iron = 1;
		}
		else if((float)temp_iron < target_iron) {
			pwm_iron++;
			if(pwm_iron > 4095) pwm_iron = 4095;
			__HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_1, pwm_iron);	// PWM_CH1 = 0 IRON
			flag_iron = 2;
		}
		else {
			flag_iron = 0;
		}
	}
}

void control_temperature_air(void)
{
	if(HAL_GetTick() - timer_air > 250) {
		timer_air = HAL_GetTick();
		//
		if((float)temp_air > target_air) {
			if(pwm_air >= 1) pwm_air--;
			//__HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_1, pwm_iron);	// PWM_CH1 = 0 IRON
			flag_air = 1;
		}
		else if((float)temp_iron < target_iron) {
			pwm_iron++;
			if(pwm_air > 4095) pwm_air = 4095;
			//__HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_1, pwm_iron);	// PWM_CH1 = 0 IRON
			flag_air = 2;
		}
		else {
			flag_air = 0;
		}
	}
}

void control_speed_air(void)
{

}
