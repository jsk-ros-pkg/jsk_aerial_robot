/*
 * buzzer.c
 *
 *  Created on: Mar 26, 2021
 *      Author: T. Anzai
 */
#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif
#include "buzzer/buzzer.h"
#include "stdbool.h"
//#include "tim.h"
#include "main.h"

extern TIM_HandleTypeDef htim13;


void buzzer_init() {
	HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1);
}

void buzzer_on(float hz) {
	int period = 4000000 / hz;
	__HAL_TIM_SET_AUTORELOAD(&htim13, period - 1);
	__HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, period / 2);
}

void buzzer_mute() {
	__HAL_TIM_SET_AUTORELOAD(&htim13, 1000);
	__HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, 0);
}

void buzzer_alert() {
	static bool state = true;
	if (state) buzzer_on(5274.04); //E8
	else buzzer_on(6271.926); //G8
	state = !state;
}

void buzzer_init_sound() {
	buzzer_on(1046.502); //C6
	HAL_Delay(80);
	buzzer_mute();
	HAL_Delay(50);
	buzzer_on(1046.502); //C6
	HAL_Delay(80);
	buzzer_mute();
	HAL_Delay(50);
	buzzer_on(2093.005); //C7
	HAL_Delay(80);
	buzzer_mute();
	HAL_Delay(50);
	buzzer_on(2093.005); //C7
	HAL_Delay(80);
	buzzer_mute();
}
