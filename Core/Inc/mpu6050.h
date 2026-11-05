/*
 * mpu6050.h
 *
 *  Created on: Oct 20, 2026
 *      Author: Usuario
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include "stm32g4xx_hal.h"

#define GYRO_CALIBRATION_SAMPLES 	500 	//amount off samples taken during calibration
#define DT							0.001f 	//time between samples (used for gyroscope)
#define SET_SLEEP					1
#define NO_SET_SLEEP				0
#define	SET_RESET					1
#define	NO_SET_RESET				0
#define ENABLE_INT					1
#define DISABLE_INT					0
#define ENABLE_G_DLPF				1
#define DISABLE_G_DLPF				0
#define ENABLE_A_DLPF				1
#define DISABLE_A_DLPF				0


typedef struct
{
	uint8_t	 			SLEEP;			//When set to 1, this bit puts the MPU-60X0 into sleep mode.

	uint8_t	 			DEVICE_RESET;	//When set to 1, this bit resets all internal registers to their default values.
										//The bit automatically clears to 0 once the reset is done.

	uint8_t	 			CLKSEL;

	uint8_t				DATA_RDY_EN;

	uint8_t				DLPF_CFG;

	uint8_t 			FCHOICE_B;

	uint8_t				A_DLPF_CFG;

	uint8_t 			ACCEL_FCHOICE_B;

	uint8_t 			PRESCALER;

	uint8_t				AFS_SEL;

	uint8_t				FS_SEL;

	float				GYRO_BIAS;

	uint8_t 			RAW_DATA[8];

	float 				ROLL;

	I2C_HandleTypeDef   *HI2C;


}MPU6050_HandleTypeDef;


void* MPU6050_1_Init(MPU6050_HandleTypeDef *hmpu, uint8_t sleep,uint8_t reset, uint8_t clksel, uint8_t int_enable, uint8_t gyro_dlpf_fc, uint8_t gyro_dlpf_en,
					uint8_t a_dlpf_fc,uint8_t  a_dlpf_en,uint8_t prescaler,uint8_t afs_sel,uint8_t gyro_fs_sel,I2C_HandleTypeDef   *hi2c);

uint8_t MPU6050_Wakeup(MPU6050_HandleTypeDef *hmpu);

void MPU6050_calibration(MPU6050_HandleTypeDef *hmpu);

void MPU6050_Read_Accel_Raw(MPU6050_HandleTypeDef *hmpu,uint8_t *mpuData);

void MPU6050_Read_Accel(MPU6050_HandleTypeDef *hmpu,float *Ax,float *Ay,float *Az);

void MPU6050_Read_Gyro_Raw(MPU6050_HandleTypeDef *hmpu,uint8_t *mpuData);

void MPU6050_Read_Gyro(MPU6050_HandleTypeDef *hmpu,float *Ax,float *Ay,float *Az);

float MPU6050_Read_Roll_Complementary(MPU6050_HandleTypeDef *hmpu);

HAL_StatusTypeDef MPU6050_Request_Read_DMA(MPU6050_HandleTypeDef *hmpu);

float MPU6050_Compute_Roll_DMA(MPU6050_HandleTypeDef *hmpu);

#endif /* INC_MPU6050_H_ */
