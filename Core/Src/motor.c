/*
 * motor.c
 *
 *  Created on: Oct 27, 2026
 *      Author: Usuario
 */

#include <motor.h>


/*float pid_controller(float setpoint, float measured, float preverror, PID_HandleTypeDef *hpid){
	// compute error
	    float error = setpoint - measured;

	// compute proportional term
	    float proportional = pid->Kp*error;

	// compute integral term
	    float integra = ((preverror+error)/2) * TIME_STEP;
	    float integral = (pid->Ki)*integra;

	// compute derivative term
	    float deriterm = (error - preverror)/TIME_STEP;
	    float derivative = pid->Kd*deriterm;

	// compute total output
	    float total_output = proportional +  integral + derivative;

	//return
	    return total_output;
}*/

void* MOTOR_Init(MOTOR_TypeDef *m, TIM_HandleTypeDef *htim, uint32_t ch,float idle,float limit){
	m->CHANNEL = ch;
	m->htim = htim;
	m->IDLE = idle;
	m->LIMIT = limit;
	return (void*)m;
}

void MOTOR_Arm(MOTOR_TypeDef *m){
	m->THROTTLE = 1000;

	__HAL_TIM_SET_COMPARE(m->htim, m->CHANNEL,1000);
	HAL_Delay(5000);

	m->ARMED = 1;

}

void MOTOR_Break(MOTOR_TypeDef *m){

	__HAL_TIM_SET_COMPARE(m->htim, m->CHANNEL,1000);

	m->THROTTLE = 1000;

}

void MOTOR_Set(MOTOR_TypeDef *m, float t, float dt){
    if (!m->ARMED) t = 0.0f;
    if (t < 0.0f) t = 0.0f;
    if (t > m->LIMIT) t = m->LIMIT;
    m->THROTTLE = t;
    __HAL_TIM_SET_COMPARE(m->htim, m->CHANNEL,MIN_PULSE + (uint16_t)(t * (MAX_PULSE - MIN_PULSE)));
}


