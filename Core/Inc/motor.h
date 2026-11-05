/*
 * motor.h
 *
 *  Created on: Oct 27, 2026
 *      Author: Usuario
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "stm32g4xx_hal.h"


#define TIME_STEP 0.005f
#define MAX_PULSE 2000u
#define MIN_PULSE 1000u

typedef struct {

    TIM_HandleTypeDef 	*htim;

    uint32_t 			CHANNEL;

    float    			LIMIT;            	// max allowed throttle, 0..1 (start at 0.2-0.3)

    float 				IDLE;				// motor starts spinning

    float    			THROTTLE;         	// current command after limits (values between 1 and 0 they represent portions of maximum throttle)

    uint8_t  			ARMED;

} MOTOR_TypeDef;

void* MOTOR_Init(MOTOR_TypeDef *m, TIM_HandleTypeDef *htim, uint32_t ch,float idle,float limit);

void MOTOR_Arm(MOTOR_TypeDef *m);

void MOTOR_Set(MOTOR_TypeDef *m, float t, float dt);

               // after the arming delay
//float pid_controller(float setpoint, float measured, float preverror, PID_HandleTypeDef *hpid);

#endif /* INC_MOTOR_H_ */
