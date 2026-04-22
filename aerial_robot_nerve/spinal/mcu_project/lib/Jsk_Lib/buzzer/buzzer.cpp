#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif
#include "buzzer/buzzer.h"

Buzzer::Buzzer()
{
}

void Buzzer::init(TIM_HandleTypeDef* htim, int ch) {
	pwm_htim_ = htim;
	pwm_ch_ = ch;
	HAL_TIM_PWM_Start(htim, ch);
}

void Buzzer::on(float hz) {
	int period = 4000000 / hz;
	__HAL_TIM_SET_AUTORELOAD(pwm_htim_, period - 1);
	__HAL_TIM_SET_COMPARE(pwm_htim_, pwm_ch_, period / 2);
}

void Buzzer::mute() {
	__HAL_TIM_SET_AUTORELOAD(pwm_htim_, 1000);
	__HAL_TIM_SET_COMPARE(pwm_htim_, pwm_ch_, 0);
}

void Buzzer::alert() {
	static bool state = true;
	if (state) Buzzer::on(5274.04); //E8
	else Buzzer::on(6271.926); //G8
	state = !state;
}

void Buzzer::init_sound() {
	Buzzer::on(1046.502); //C6
	HAL_Delay(80);
	Buzzer::mute();
	HAL_Delay(50);
	Buzzer::on(1046.502); //C6
	HAL_Delay(80);
	Buzzer::mute();
	HAL_Delay(50);
	Buzzer::on(2093.005); //C7
	HAL_Delay(80);
	Buzzer::mute();
	HAL_Delay(50);
	Buzzer::on(2093.005); //C7
	HAL_Delay(80);
	Buzzer::mute();
}
