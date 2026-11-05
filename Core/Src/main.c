/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mpu6050.h"
#include "motor.h"
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CIRCLE_RADIUS 15
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
	uint8_t imu_data_ready = 0,request_data = 0, i2c_failed = 0,batery = 0,arm_ready = 0,transmit_ready = 0,flag_oled_free = 0,updatescreen = 0,x1 = 0,x2 = 0,y1angle = 0,y2 = 0;
	MPU6050_HandleTypeDef hmpu1;
//	extern MOTOR_TypeDef m1;
	float roll = 0,pwm = 0,angle = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	char txt[7] = {0};
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */

  //initialize the oled display
  ssd1306_Init();
  ssd1306_Fill(Black);
  ssd1306_SetCursor(0, 0);
  ssd1306_WriteString("Porfavor espere", Font_7x10, White);
  ssd1306_SetCursor(0, 11);
  ssd1306_WriteString("mientras", Font_7x10, White);
  ssd1306_SetCursor(0, 22);
  ssd1306_WriteString("configuramos", Font_7x10, White);
  ssd1306_SetCursor(0, 33);
  ssd1306_WriteString("el sistema.", Font_7x10, White);
  ssd1306_UpdateScreen();




  //initialize the imu
  MPU6050_1_Init(&hmpu1, NO_SET_SLEEP, NO_SET_RESET, 1, DISABLE_INT, 0, DISABLE_G_DLPF, 0, DISABLE_A_DLPF, 7, 0, 0, &hi2c1);

  //intialize the motor
//  MOTOR_Init(&m1,&htim3,TIM_CHANNEL_2,1000,2000);

  //prendo la imu (le leo el registro WHO_AM_I hasta que me responda el
  //valor que se guarda en ese registro en todas las mpu6500 por igual 0x70)
  while(MPU6050_Wakeup(&hmpu1)){
	  HAL_Delay(10);
  }

  //calibro el giroscopio de la imu
  MPU6050_calibration(&hmpu1);

  //armo el motor
//  MOTOR_Arm(&m1);
  ssd1306_Fill(Black);
  ssd1306_SetCursor(0, 0);
  ssd1306_WriteString("Motor armado,", Font_7x10, White);
  ssd1306_SetCursor(0, 11);
  ssd1306_WriteString("conecte la bateria", Font_7x10, White);
  ssd1306_SetCursor(0, 22);
  ssd1306_WriteString("luego presione", Font_7x10, White);
  ssd1306_SetCursor(0, 33);
  ssd1306_WriteString("el boton.", Font_7x10, White);
  ssd1306_UpdateScreen();

  arm_ready = 1;
  while(!batery){

  }
  ssd1306_Fill(Black);
  ssd1306_SetCursor(43, 0);
  ssd1306_WriteString("Angulo:", Font_7x10, White);
  ssd1306_UpdateScreen();



  //prendo los timers
  HAL_TIM_Base_Start_IT(&htim2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  /*if(arm_ready == 1){
		  MOTOR_Set(&m1,pwm);
		  arm_ready = 0;
	  }*/

	  //refresh screen
	  if(updatescreen && flag_oled_free){
		  ssd1306_FillRectangle(0,22,127,64,Black);
/*		  angle = roll/57.29578f;
		  x1 = (uint8_t)(cosf(angle) *CIRCLE_RADIUS) + 63;
		  x2 = (uint8_t)(cosf(angle + 3.1415f) * CIRCLE_RADIUS) + 63;
		  y1angle = (uint8_t)(cosf(angle) * CIRCLE_RADIUS) + 45;
		  y2 = (uint8_t)(sinf(angle + 3.1415f) * CIRCLE_RADIUS) + 45;
*/
		  //ssd1306_Line(x1,y1angle,x2,y2,White);
		  ssd1306_DrawArc(63,37,15,90,(int16_t)roll + 180,White);
		  snprintf(txt, sizeof(txt), "%6.1f", roll);
	      ssd1306_SetCursor(40, 11);
  	  	  ssd1306_WriteString(txt, Font_7x10, White);
		  ssd1306_UpdateScreen_DMA(&flag_oled_free);
		  updatescreen = 0;

	  }

	  //request data form the imu at 500Hz (possible interruption implementation)
	  if(request_data){
		  request_data = 0;
		  if(!MPU6050_Request_Read_DMA(&hmpu1)){
			  i2c_failed = 0;
		  }
		  else{
			  i2c_failed = 1;
		  }
	  }
	  //process the data gathered from the imu
	  if(imu_data_ready){
		  imu_data_ready = 0;
		  roll = MPU6050_Compute_Roll_DMA(&hmpu1);
		  pwm = (roll + 90.0f)/180.0f;
		  //pid(roll);
	  }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	static uint8_t i = 0;
	if(htim->Instance == TIM2){
		request_data = 1;
		i++;
		if(i >= 250){
			updatescreen = 1;
			//arm_ready = 1;
			i = 0;
		}
	}
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c){
	if(hi2c->Instance == I2C1){
		imu_data_ready = 1;
	}
	if(hi2c->Instance == I2C1){
		flag_oled_free = 1;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == battery_connect_Pin)batery = 1;
	//if(GPIO_Pin == mpu6050_int_Pin)imu_data_ready = 1;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
