/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "RobStride.h"
#include "uart_printf.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

//备份例程

//    switch(mode)
//    {
//        // ===== 普通模式接口 =====
//        case 0: // 使能（普通模式）
//            RobStride_01.Enable_Motor();
//		HAL_Delay(500);
//            break;
//        case 1: // 失能（普通模式）
//            RobStride_01.Disenable_Motor(1);
//            break;
//        case 2: // 运控模式
//            HAL_Delay(5);
//            RobStride_01.RobStride_Motor_move_control(5, 0, 0, 0.0, 0.0);
//            break;
//        case 3: // PP位置模式
//            RobStride_01.RobStride_Motor_Pos_control(2.0, 0);
//						HAL_Delay(5);
//            break;
//				case 4:	//CSP位置模式
//						RobStride_01.RobStride_Motor_CSP_control(2.0, 2.0);
//						HAL_Delay(5);
//						break;
//        case 5: // 速度模式
//            RobStride_01.RobStride_Motor_Speed_control(3.5, 5.0);
//						HAL_Delay(5);
//            break;
//        case 6: // 电流模式
//            HAL_Delay(5);
//            RobStride_01.RobStride_Motor_current_control(1.2);
//            break;
//        case 7: // 设置机械零点
//            RobStride_01.Set_ZeroPos();
//            break;
//        case 8: // 读取参数
//            RobStride_01.Get_RobStride_Motor_parameter(0x7014);
//            break;
//        case 9: // 设置参数
//            RobStride_01.Set_RobStride_Motor_parameter(0x7014, 0.35f, Set_parameter);
//            break;
//        case 10: // 协议切换（如切MIT协议/Canopen/私有协议）
//            RobStride_01.RobStride_Motor_MotorModeSet(0x02); // 0x02=MIT
//            break;

//        // ===== MIT模式接口（只能用MIT专用函数！） =====
//        case 11: // MIT 使能
//            RobStride_01.RobStride_Motor_MIT_Enable();
//            break;
//        case 12: // MIT 失能
//            RobStride_01.RobStride_Motor_MIT_Disable();
//            break;
//        case 13: // MIT 综合控制
//						RobStride_01.RobStride_Motor_MIT_SetMotorType(0x01);
//            RobStride_01.RobStride_Motor_MIT_Enable();
//            HAL_Delay(5);
//            RobStride_01.RobStride_Motor_MIT_Control(0, 0, 0, 0, -1.0f);
//            break;
//        case 14: // MIT 位置控制
//						RobStride_01.RobStride_Motor_MIT_SetMotorType(0x01);
//						RobStride_01.RobStride_Motor_MIT_Enable();
//            HAL_Delay(5);
//            RobStride_01.RobStride_Motor_MIT_PositionControl(1.57f, 3.0f);
//            break;
//        case 15: // MIT 速度控制
//						RobStride_01.RobStride_Motor_MIT_SetMotorType(0x02);
//						RobStride_01.RobStride_Motor_MIT_Enable();
//            HAL_Delay(5);
//            RobStride_01.RobStride_Motor_MIT_SpeedControl(4.5f, 3.2f);
//            break;
//        case 16: // MIT 零点设置（运行前需保证 MIT_Type != positionControl）
//            RobStride_01.RobStride_Motor_MIT_SetZeroPos();
//            break;
//        case 17: // MIT 清错
//            RobStride_01.RobStride_Motor_MIT_ClearOrCheckError(0x01);
//            break;
//        case 18: // MIT 设置电机运行模式
//            RobStride_01.RobStride_Motor_MIT_SetMotorType(0x01);
//            break;
//        case 19: // MIT 设置电机ID
//            RobStride_01.RobStride_Motor_MIT_SetMotorId(0x05);
//            break;
//        case 20: //主动上报
//            RobStride_01.RobStride_Motor_ProactiveEscalationSet(0x00);
//            break;
//        case 21: // 波特率修改
//            RobStride_01.RobStride_Motor_BaudRateChange(0x01);
//            break;
//        case 22: // MIT 参数保存
//            RobStride_01.RobStride_Motor_MotorDataSave();
//            break;
//				case 23: // MIT 协议切换（如切MIT协议/Canopen/私有协议）
//						RobStride_01.RobStride_Motor_MIT_MotorModeSet(0x00);
//						break;

//        default:
//            break;
//    }
//	
//		HAL_Delay(50);
//	mode = 6;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
CAN_FilterTypeDef CAN_FilterStrue; 
CAN_TxHeaderTypeDef CAN_TxHeaderStrue; 
CAN_RxHeaderTypeDef CAN_RxHeaderStrue; 
uint8_t pRxdata[8], pTxdata[8]; 
RobStride_Motor RobStride_01(0x07, false);

uint8_t mode = 0; 
uint8_t init_state = 0 ;
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
  MX_USART6_UART_Init();
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_TIM12_Init();
  /* USER CODE BEGIN 2 */
  CAN_FilterStrue.FilterBank = 0; 
  CAN_FilterStrue.FilterMode = CAN_FILTERMODE_IDMASK; 
  CAN_FilterStrue.FilterScale = CAN_FILTERSCALE_16BIT; 
  CAN_FilterStrue.FilterIdHigh = 0;
	CAN_FilterStrue.FilterIdLow = 0;
	CAN_FilterStrue.FilterActivation = ENABLE;
	CAN_FilterStrue.FilterFIFOAssignment = CAN_RX_FIFO0;
	
	HAL_Delay(2000);
	
	HAL_CAN_ConfigFilter(&hcan1, &CAN_FilterStrue);
  HAL_CAN_Start(&hcan1); //鍚姩CAN
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); 
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_Base_Start(&htim8);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
  HAL_Delay(1000);
  
  
  RobStride_01.RobStride_Motor_MotorModeSet(0x02);
  
  HAL_Delay(1000);
  RobStride_01.RobStride_Motor_ProactiveEscalationSet(0x01);
  
  HAL_Delay(1000);
  RobStride_01.RobStride_Motor_MIT_Disable();
  
  HAL_Delay(1000);
  RobStride_01.RobStride_Motor_MIT_SetZeroPos();
  
  HAL_Delay(1000);
  
  RobStride_01.RobStride_Motor_MIT_Enable();
  
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  
	  
	  
	  
	  float motor1_angle = RobStride_01.Pos_Info.Angle;
	  
	  usart6_printf("%f \r\n",motor1_angle);
	  
	  RobStride_01.RobStride_Motor_MIT_Control(0, 0, 0, 0, 0.0f);
	  HAL_Delay(10);
	  
	  
	  
//	  RobStride_01.Enable_Motor();
	  
//	  HAL_Delay(2000);
	  
	  
	  
//	  RobStride_01.RobStride_Motor_MIT_Disable();
	  
//	  RobStride_01.Disenable_Motor(1);
	  
//	  HAL_Delay(2000);
	  
	  
	  HAL_Delay(1);
	  
	  
	  
	  
	  
	  
	  
	  
	  
	  
	  
	  

	//步态
	
	
	
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
CAN_RxHeaderTypeDef RXHeader;
uint8_t RxData[8];
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)							
{

	if(HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&RXHeader,RxData) == HAL_OK)
	{
		if (RXHeader.IDE == CAN_ID_EXT)
		{
			if (RxData[0] == 0x07)
            {
                RobStride_01.RobStride_Motor_Analysis(RxData, RXHeader.ExtId);
            }
            
				
		}
		else
		{
			if (RxData[0] == 0x07)
            {
                RobStride_01.RobStride_Motor_Analysis(RxData, RXHeader.StdId);
            }
            
				
		}
	}	
}




/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
	if(htim == &htim12)
	{
		float motor1_angle = RobStride_01.Pos_Info.Angle;
		float motor1_speed = RobStride_01.Pos_Info.Speed;
		RobStride_01.RobStride_Motor_MIT_Control(0, 0, 0, 0, -0.1f);
		usart6_printf("%f,%f \r\n",motor1_angle,motor1_speed);
	}

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

#ifdef  USE_FULL_ASSERT
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
