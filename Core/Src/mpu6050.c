/*
 * mpu6050.c
 *me vendieron gato por liebre al final es una mpu6500
 *  Created on: Oct 20, 2026
 *      Author: Usuario
 */

#include "mpu6050.h"
#include "math.h"
#include "i2c.h"

#define INTERRUPT 			0x01
#define NON_INTERRUPT 		0x00

//registros
#define MPU6050_ADDR		0xD0
#define WHO_AM_I			0X75	//Address of identification																	(R)
#define	PWR_MGMT_1   		0x6B	/*Address of PWR_MGMT_1 register that controls:												(R/W)
											Bit0/1/2:CLKSEL
										  	Bit3:TEMP_DIS
											Bit4:GYRO_STANDBY
											Bit5:CYCLE
											Bit6:SLEEP
											Bit7:DEVICE_RESET

											DEVICE_RESET 	When set to 1, this bit resets all internal registers to their default values.
														 	The bit automatically clears to 0 once the reset is done.

											SLEEP 			When set to 1, this bit puts the MPU-60X0 into sleep mode.

											CYCLE 			When set, and SLEEP and STANDBY are not set, the chip will cycle
															between sleep and taking a single sample at a rate determined by
															LP_ACCEL_ODR (MPU-6500 mode) or LP_WAKE_CTRL (MPU-6050 compatible mode)

											GYRO_STANDBY	When set, the gyro drive and pll circuitry are enabled, but the sense paths
															are disabled. This is a low power mode that allows quick enabling of the
															gyros

											TEMP_DIS 		When set to 1, this bit disables the temperature sensor.

											CLKSEL 			3-bit unsigned value. Specifies the clock source of the device
															0:Internal 20Mhz oscillator
															1:Auto selects the best available clock source – PLL if ready, else use the Internal oscillator
															2:Auto selects the best available clock source – PLL if ready, else use the Internal oscillator
															3:Auto selects the best available clock source – PLL if ready, else use the Internal oscillator
															4:Auto selects the best available clock source – PLL if ready, else use the Internal oscillator
															5:Auto selects the best available clock source – PLL if ready, else use the Internal oscillator
															6:Internal 20MHz oscillator
															7:Stops the clock and keeps the timing generator in reset
	 	 	 	 	 	 	 	 	 	 	*/
#define	CONFIG				0x1A	//Address of register used to set the Gyroscopes DLPF										(R/W)
#define	SMPRT_DIV			0x19	//Address of register used to set sample rate of mpu6050									(R/W)
#define	GYRO_CONFIG			0x1B	//Address of register used to configure the gyroscope										(R/W)
#define	ACCEL_CONFIG		0x1C	//Address of register used to configure the accelerometer									(R/W)
#define	ACCEL_CONFIG_2		0x1D	//Address of register used to configure the accelerometer DLPF								(R/W)
#define	GYRO_READ_BEGIN		0x43	//Address of register with the first gyroscope data											(R)
#define	ACCEL_READ_BEGIN 	0x3B	//Address of register with the first accelerometer data										(R)
#define	INT_ENABLE 			0x38	//Address of register used to enable interrupt	(0x01 enable interrupt 0x00 disable)		(R/W)


void* MPU6050_1_Init(MPU6050_HandleTypeDef *hmpu, uint8_t sleep,uint8_t reset, uint8_t clksel, uint8_t int_enable, uint8_t gyro_dlpf_fc, uint8_t gyro_dlpf_en,
					uint8_t a_dlpf_fc,uint8_t  a_dlpf_en,uint8_t prescaler,uint8_t afs_sel,uint8_t gyro_fs_sel,I2C_HandleTypeDef *hi2c){

	//configuracion de la mpu6500
	hmpu->DEVICE_RESET = sleep;
	hmpu->SLEEP = reset;
	hmpu->CLKSEL = clksel;
	hmpu->DATA_RDY_EN = int_enable;
	hmpu->PRESCALER = prescaler;
	hmpu->DLPF_CFG = gyro_dlpf_fc; 		//for later application try 3 (41hz) if vibration doesn't hurt up it
	hmpu->FCHOICE_B = gyro_dlpf_en;
	hmpu->A_DLPF_CFG = a_dlpf_fc; 		//for later application try 4 (21hz) if delay doesn't hurt lower it
	hmpu->ACCEL_FCHOICE_B = a_dlpf_en;
	hmpu->AFS_SEL = afs_sel;
	hmpu->FS_SEL = gyro_fs_sel;

	//data of the imu used  during runtime
	hmpu->HI2C = hi2c;
	hmpu->GYRO_BIAS = 0;
	hmpu->RAW_DATA[0] = 0;
	hmpu->ROLL= 0;

	return (void*)hmpu;
}

uint8_t MPU6050_Wakeup(MPU6050_HandleTypeDef *hmpu) {
	/*
	 *	set_fc setea la frecuencia de corte del filtro pasabajos integrado en la imu
	 *	set_fc || 		accelerometer bandwidth || 	gyroscope bandwidth
	 *	0:							260						250
	 *	1:							184						184
	 *	2:							94						92
	 *	3:							44						41
	 *	4:							21						20
	 *	5:							10						10
	 *	6:							5						5
	 *	7:						RESERVED					3600
	*/
	uint8_t check = 0;
	uint8_t Data = 0x80;


	HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1, 1, &Data, 1,1000);
	HAL_Delay(100);

	HAL_StatusTypeDef stwho = HAL_I2C_Mem_Read(hmpu->HI2C, MPU6050_ADDR, WHO_AM_I, 1, &check, 1,1000);

	HAL_Delay(100);


	if (check == 0x70 && stwho == HAL_OK) {// 0x70 will be returned by the sensor if everything goes well

		//
		Data = (hmpu->DEVICE_RESET << 7 | hmpu->SLEEP << 6 | hmpu->CLKSEL);
		HAL_I2C_Mem_Write(hmpu->HI2C, MPU6050_ADDR, PWR_MGMT_1, 1, &Data, 1,1000);
		HAL_Delay(100);


		//set DATA_RDY_EN 1 deja que la imu setee en uno su pin de interrupcion
		//cuando todos sus sensores habilitados cambian una por lo menos una ves sus valores
		Data = hmpu->DATA_RDY_EN;
		HAL_I2C_Mem_Write(hmpu->HI2C,MPU6050_ADDR,INT_ENABLE, 1, &Data,1, 1000);
		HAL_Delay(10);


		//This register is only effective when FCHOICE = 2’b11 (FCHOICE_B
		//register bits are 2’b00), and (0 < DLPF_CFG < 7)
		//This is the update rate of sensor register.
		//SAMPLE_RATE = INTERNAL_SAMPLE_RATE / (1 + PRESCALER)
		//where INTERNAL_SAMPLE_RATE = 1kHz
		Data = hmpu->PRESCALER;
		HAL_I2C_Mem_Write(hmpu->HI2C,MPU6050_ADDR,SMPRT_DIV, 1, &Data, 1,1000);
		HAL_Delay(10);


		//Set frecuencia de corte
		Data = hmpu->DLPF_CFG;
		HAL_I2C_Mem_Write(hmpu->HI2C,MPU6050_ADDR,CONFIG, 1, &Data, 1,1000);
		HAL_Delay(10);


		// Set Gyroscopio
		Data = (hmpu->FS_SEL << 3 | hmpu->FCHOICE_B); // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0  250 ̐/s
		HAL_I2C_Mem_Write(hmpu->HI2C,MPU6050_ADDR,GYRO_CONFIG, 1, &Data, 1,1000);
		HAL_Delay(10);

		// Set accelerometero
		Data = hmpu->AFS_SEL << 3; // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0  2g
		HAL_I2C_Mem_Write(hmpu->HI2C,MPU6050_ADDR,ACCEL_CONFIG, 1, &Data, 1,1000);
		HAL_Delay(10);

		// Set accelerometero
		Data = (hmpu->ACCEL_FCHOICE_B << 3 | hmpu->A_DLPF_CFG); // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0  2g
		HAL_I2C_Mem_Write(hmpu->HI2C,MPU6050_ADDR,ACCEL_CONFIG_2, 1, &Data, 1,1000);
		HAL_Delay(10);


		return 0;
	} else {
		return 1;
	}

}

void MPU6050_calibration(MPU6050_HandleTypeDef *hmpu){
	//con la imu quieta (el gyroscopio deberia dar 0)
	//se toman muestras y se promedian con el fin de eliminar deriva estatica
	uint8_t mpudata[2];
	int16_t gyro_RAW = 0;
	float gyro = 0;
	uint16_t i;
	for ( i = 0;  i < GYRO_CALIBRATION_SAMPLES; ++ i) {
		//leo los 2 valores que conforman el valor del gyroscopio en eje x
		HAL_I2C_Mem_Read (hmpu->HI2C, MPU6050_ADDR, GYRO_READ_BEGIN, 1, mpudata, 2, 1000);

		gyro_RAW = (int16_t)(mpudata[0] << 8 | mpudata [1]);
		gyro += (float)(gyro_RAW)/131.0f;
		HAL_Delay(10);
	}
	hmpu->GYRO_BIAS = (float)(gyro/GYRO_CALIBRATION_SAMPLES);
}

void MPU6050_Read_Accel_Raw(MPU6050_HandleTypeDef *hmpu, uint8_t *mpudata){

	//leo los 6 valores que conforman los tres valores del accelerometro
	HAL_I2C_Mem_Read (hmpu->HI2C, MPU6050_ADDR, ACCEL_READ_BEGIN, 1, mpudata, 6, 1000);

}

void MPU6050_Read_Accel(MPU6050_HandleTypeDef *hmpu,float *Ax,float *Ay,float *Az){
	uint8_t mpudata[6];
	int16_t accel_X_RAW = 0,accel_Y_RAW = 0,accel_Z_RAW = 0;

	//leo los 6 valores que conforman los tres valores del accelerometro
	HAL_I2C_Mem_Read (hmpu->HI2C, MPU6050_ADDR, ACCEL_READ_BEGIN, 1, mpudata, 6, 1000);

	accel_X_RAW = (int16_t)(mpudata[0] << 8 | mpudata [1]);
	accel_Y_RAW = (int16_t)(mpudata[2] << 8 | mpudata [3]);
	accel_Z_RAW = (int16_t)(mpudata[4] << 8 | mpudata [5]);

	*Ax = (float)accel_X_RAW/16384.0f;
	*Ay = (float)accel_Y_RAW/16384.0f;
	*Az = (float)accel_Z_RAW/16384.0f;

}

void MPU6050_Read_Gyro_Raw(MPU6050_HandleTypeDef *hmpu,uint8_t *mpudata){

	//leo los 6 valores que conforman los tres valores del gyroscopio
	HAL_I2C_Mem_Read (hmpu->HI2C, MPU6050_ADDR, GYRO_READ_BEGIN, 1, mpudata, 6, 1000);

}

void MPU6050_Read_Gyro(MPU6050_HandleTypeDef *hmpu,float *Gx,float *Gy,float *Gz){
	uint8_t mpudata[6];
	int16_t gyro_X_RAW = 0,gyro_Y_RAW = 0,gyro_Z_RAW = 0;

	//leo los 6 valores que conforman los tres valores del gyroscopio
	HAL_I2C_Mem_Read (hmpu->HI2C, MPU6050_ADDR, GYRO_READ_BEGIN, 1, mpudata, 6, 1000);

	gyro_X_RAW = (int16_t)(mpudata[0] << 8 | mpudata [1]);
	gyro_Y_RAW = (int16_t)(mpudata[2] << 8 | mpudata [3]);
	gyro_Z_RAW = (int16_t)(mpudata[4] << 8 | mpudata [5]);

	*Gx = (float)gyro_X_RAW/131.0f;
	*Gy = (float)gyro_Y_RAW/131.0f;
	*Gz = (float)gyro_Z_RAW/131.0f;

}

float MPU6050_Read_Roll_Complementary(MPU6050_HandleTypeDef *hmpu){
	float gyro = 0,roll_accel = 0,accel_z = 0,accel_y = 0;
	uint8_t mpudata[8];
	int16_t gyro_RAW = 0,accel_Y_RAW = 0,accel_Z_RAW = 0;

	//leo valores raw de accelerometro y y z
	//leo sensor de temperatura y descarto
	//leo valores raw de Gyroscopio x
	HAL_I2C_Mem_Read (hmpu->HI2C, MPU6050_ADDR, ACCEL_READ_BEGIN + 2, 1, mpudata, 8, 1000); //leer los 8 descartar es un pelin mas rapido que leer 4 del accel y despues 2 de gyro

	accel_Y_RAW = (int16_t)(mpudata[0] << 8 | mpudata [1]);
	accel_Z_RAW = (int16_t)(mpudata[2] << 8 | mpudata [3]);

	accel_y = (float)accel_Y_RAW/16384.0f;
	accel_z = (float)accel_Z_RAW/16384.0f;

	roll_accel = atan2f(accel_y,accel_z) * 57.29578f;

	gyro_RAW = (int16_t)(mpudata[6] << 8 | mpudata [7]);
	gyro = (float)(gyro_RAW/131.0f) - hmpu->GYRO_BIAS;

	hmpu->ROLL = 0.98f * (hmpu->ROLL + gyro * DT) + roll_accel * 0.02f;

	return hmpu->ROLL;
}

//future possible DMA implementation
HAL_StatusTypeDef MPU6050_Request_Read_DMA(MPU6050_HandleTypeDef *hmpu){

	return HAL_I2C_Mem_Read_DMA(hmpu->HI2C, MPU6050_ADDR, 0x3D,I2C_MEMADD_SIZE_8BIT,hmpu->RAW_DATA, 8);

}

float  MPU6050_Compute_Roll_DMA(MPU6050_HandleTypeDef *hmpu){
    int16_t ay_raw = (int16_t)(hmpu->RAW_DATA[0] << 8 | hmpu->RAW_DATA[1]);
    int16_t az_raw = (int16_t)(hmpu->RAW_DATA[2] << 8 | hmpu->RAW_DATA[3]);
    int16_t gx_raw = (int16_t)(hmpu->RAW_DATA[6] << 8 | hmpu->RAW_DATA[7]);

    float roll_accel = atan2f(ay_raw / 16384.0f, az_raw / 16384.0f) * 57.29578f;
    float gyro       = gx_raw / 131.0f - hmpu->GYRO_BIAS;

    hmpu->ROLL = 0.98f * (hmpu->ROLL + gyro * DT) + 0.02f * roll_accel;

    return hmpu->ROLL;
}






