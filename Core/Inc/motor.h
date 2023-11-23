/*
 * motor.h
 *
 *  Created on: Oct 30, 2023
 *      Author: Hien Nguyen
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_



#endif /* INC_MOTOR_H_ */

#include <stdint.h>
#include "main.h"
#include <stm32f1xx_hal_tim.h>


#define		MAX_CNT			300
#define 	MIN_CNT			20
#define 	TIME_EXAMPLE	0.010 		// 10ms
#define 	PPR				1320
#define 	K_IN0 			0.01
#define 	K_IN1 			0.001
#define 	K_OUT_FUZZY 	24
//#define 	U_MAX			1

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;


typedef struct
{
	float	f_K_e;
	float	f_K_e_dot;
	float	f_K_u;
	float	f_ek_1;
	float   f_ek_2;
	float	f_inp_fuzzy[2];
	float	f_uk;		// -1 -> 1
	float 	f_setpoint;	// speed (round/minute)
	int 	i_pre_cnt;

} Parameter_controller_t;
static void Set_duty(float duty, TIM_HandleTypeDef *htim, int channel); // duty 0-1
void Init_tim_pwm();
void Forward (float duty);
void Reverse (float duty);
void Stop_motor();
float Read_enc (int * pre_cnt); // round/minute
void Controller (float new_speed, Parameter_controller_t *parameter); // PD fuzzy
void Init_parameter(float setpoint, Parameter_controller_t *parameter);
