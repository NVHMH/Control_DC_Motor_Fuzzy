/*
 * motor.c
 *
 *  Created on: Oct 30, 2023
 *      Author: Hien Nguyen
 */


#include "motor.h"



static void Set_duty(float duty, TIM_HandleTypeDef *htim, int channel){
	if (channel == 1){
		htim->Instance->CCR1 =  (duty*MAX_CNT) + MIN_CNT;
	}
	if (channel == 2){
		htim->Instance->CCR2 =  (duty*MAX_CNT) + MIN_CNT;
	}
}

void Init_tim_pwm(){
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}

void Stop_motor(){
	Set_duty(0, &htim3, 1);
	Set_duty(0, &htim1, 1);
}

void Forward (float duty){
	Set_duty(0, &htim3, 1);
	Set_duty(duty, &htim1, 1);
}

void Reverse (float duty){
	Set_duty(duty, &htim3, 1);
	Set_duty(0, &htim1, 1);
}

float Read_enc (int *pre_cnt){
	int delta_cnt;
	delta_cnt = (int16_t)(htim2.Instance->CNT)-*pre_cnt;
	*pre_cnt = htim2.Instance->CNT;
	return  delta_cnt*60.0f/TIME_EXAMPLE/PPR;
}

void Controller (float new_speed, Parameter_controller_t *parameter){

	float ek, ek_dot;
	ek =parameter->f_setpoint- new_speed;
	ek_dot = (ek-2*parameter->f_ek_1+parameter->f_ek_2)/TIME_EXAMPLE;
	parameter->f_inp_fuzzy[0] = ek*parameter->f_K_e;
	parameter->f_inp_fuzzy[1] = ek_dot*parameter->f_K_e_dot;
	fuzzy_run(parameter->f_inp_fuzzy,&(parameter->f_uk));
	if((parameter->f_uk<=1)&&(parameter->f_uk>0)){
		Forward(parameter->f_uk);
	} else if ((parameter->f_uk<=0)&&(parameter->f_uk>=-1)){
		parameter->f_uk*=(-1);
		Reverse(parameter->f_uk);
	}

	//  fuzzy: input (ek_norm, ek_dot_norm), output (uk_dot_norm)
//	parameter->f_uk = TIME_EXAMPLE*(uk_dot_norm+parameter->uk_dot_norm_1);
//	if(parameter->f_uk> U_MAX)  parameter->f_uk = U_MAX;
//	if(parameter->f_uk<-U_MAX) parameter->f_uk = -U_MAX;


//	parameter->f_ek_2				= parameter->f_ek_1	;
//	parameter->f_ek_1				= ek;
//	parameter->uk_dot_norm_1		= uk_dot_norm;
}

void Init_parameter(float setpoint, Parameter_controller_t *parameter){
	parameter->f_K_e = K_IN0;
	parameter->f_K_e_dot = K_IN1;
	parameter->f_K_u = K_OUT_FUZZY;
	parameter->f_ek_1 = 0;
	parameter->f_ek_2 = 0;
	parameter->i_pre_cnt = 0;
	parameter->f_setpoint = setpoint;
}
