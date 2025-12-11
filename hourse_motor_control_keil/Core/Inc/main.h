/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
	
	
	//大腿幅度
	#define HOURSE_BIG_LEG_RANGE_FRONT 0.5f * PI  //motor1motor2
	#define HOURSE_BIG_LEG_RANGE_BEHIND 0.5f * PI  //motor3motor4
	
	//脖子幅度
	#define HOURSE_BOZI_RANGE 0.5f * PI  //motor5
	
	//小腿幅度
	#define HOURSE_SMALL_LEG_RANGE_FRONT 0.15f * PI  
	#define HOURSE_SMALL_LEG_RANGE_BEHIND 0.5f * PI  
	
	
	//相位差(大腿)
	#define LEG_1_MOTOR_PHASE 0.25f * PI //motor1右前大腿
	#define LEG_2_MOTOR_PHASE 0.00f * PI //motor2左前大腿
	#define LEG_3_MOTOR_PHASE -0.25f * PI //motor3左后大腿
	#define LEG_4_MOTOR_PHASE 0.00f * PI //motor4右后大腿
	
	//相位差(脖子)
	#define BOZI_5_MOTOR_PHASE 0.25f * PI //motor5脖子
	
	//相位差（小腿）
	#define LEG_6_MOTOR_PHASE 0.25f * PI //motor1右前大腿
	#define LEG_7_MOTOR_PHASE 0.00f * PI //motor2左前大腿
	#define LEG_8_MOTOR_PHASE -0.25f * PI //motor3左后大腿
	#define LEG_9_MOTOR_PHASE 0.00f * PI //motor4右后大腿
	
	
	
	
	#define LEG1_KP 2.0f
	#define LEG2_KP 2.0f
	#define LEG3_KP 2.0f
	#define LEG4_KP 2.0f
	
	#define BOZI_KP 2.0f
	
	#define LEG6_KP 2.0f
	#define LEG7_KP 15.0f
	#define LEG8_KP 2.0f
	#define LEG9_KP 2.0f
	
	
	
	//共参
	#define HOURSE_SPEED 0.001f//速度
	#define INIT_TIME 1.0f
	
	
	
	//常用定参
	#define PI 3.14159265358979323846f
	
	
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_R_Pin GPIO_PIN_12
#define LED_R_GPIO_Port GPIOH
#define LED_G_Pin GPIO_PIN_11
#define LED_G_GPIO_Port GPIOH
#define LED_B_Pin GPIO_PIN_10
#define LED_B_GPIO_Port GPIOH
#define CS1_ACCEL_Pin GPIO_PIN_4
#define CS1_ACCEL_GPIO_Port GPIOA
#define INT1_ACCEL_Pin GPIO_PIN_4
#define INT1_ACCEL_GPIO_Port GPIOC
#define INT1_GRYO_Pin GPIO_PIN_5
#define INT1_GRYO_GPIO_Port GPIOC
#define CS1_GYRO_Pin GPIO_PIN_0
#define CS1_GYRO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
