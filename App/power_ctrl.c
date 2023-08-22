/*
 * power_ctrl.c
 *
 *  Created on: Aug 15, 2023
 *      Author: rinaldo.santos
 */

#include "main.h"
#include "tim.h"
#include "power_ctrl.h"

extern uint16_t pwm_iron;
extern float target_iron, target_air, temperature_iron , temperature_air;

uint32_t timer_iron = 0, timer_air = 0, timer_speed = 0;
uint16_t pwm_air = 0, pwm_speed = 0;

void control_temperature_iron(void)
{
	if(HAL_GetTick() - timer_iron > 250) {
		timer_iron = HAL_GetTick();
		//
		if(temperature_iron > target_iron) {
			if(pwm_iron >= 1) pwm_iron--;
			__HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_1, pwm_iron);	// PWM_CH1 IRON
		}
		else if(temperature_iron < target_iron) {
			pwm_iron++;
			if(pwm_iron > 4095) pwm_iron = 4095;
			__HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_1, pwm_iron);	// PWM_CH1 IRON
		}
		else {

		}
	}
}

void control_temperature_air(void)
{
	if(HAL_GetTick() - timer_air > 250) {
		timer_air = HAL_GetTick();
		//
		if(temperature_air > target_air) {

		}
		else if(temperature_air < target_air) {

		}
		else {

		}
	}
}

void control_speed_air(void)
{
	if(HAL_GetTick() - timer_speed > 250) {
		timer_speed = HAL_GetTick();
	}
}
