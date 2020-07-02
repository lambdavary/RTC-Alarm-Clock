/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;
RTC_AlarmTypeDef alarm_time;
int alarmFlag = 0;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
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
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_RTC_Init();
	
  RTC_TimeTypeDef rtc_time;
	RTC_TimeTypeDef time;
  while (1)
  {
		HAL_RTC_GetAlarm(&hrtc, &alarm_time, RTC_ALARM_A, RTC_FORMAT_BIN);
		HAL_RTC_GetTime(&hrtc, &time, RTC_FORMAT_BIN);
		
    if(alarmFlag == 1){
			rtc_time = alarm_time.AlarmTime;
		}else{
			rtc_time = time;
		}
		
		//led
		if(alarm_time.AlarmTime.Hours == time.Hours 
			&& alarm_time.AlarmTime.Minutes == time.Minutes
			&& alarm_time.AlarmTime.Seconds == time.Seconds)
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
		
		
		int f_sec = rtc_time.Seconds%10;
		int s_sec = rtc_time.Seconds/10;
		int f_min = rtc_time.Minutes%10;
		int s_min = rtc_time.Minutes/10;
		int f_hour = rtc_time.Hours%10;
		int s_hour = rtc_time.Hours/10;
		
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, f_sec&0x1);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, f_sec&0x2);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, f_sec&0x4);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, f_sec&0x8);
		
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, s_sec&0x1);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, s_sec&0x2);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, s_sec&0x4);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, s_sec&0x8);
		
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, f_min&0x1);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, f_min&0x2);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, f_min&0x4);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, f_min&0x8);
		
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, s_min&0x1);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_14, s_min&0x2);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, s_min&0x4);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, s_min&0x8);
		
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, f_hour&0x1);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, f_hour&0x2);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, f_hour&0x4);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, f_hour&0x8);
		
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, s_hour&0x1);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, s_hour&0x2);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, s_hour&0x4);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, s_hour&0x8);
		
  }
  /* USER CODE END 3 */
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  
  UNUSED(GPIO_Pin);
	switch(GPIO_Pin){
		case AlarmSet_Pin:
			if(alarmFlag==0)
				alarmFlag = 1;
			else
				alarmFlag = 0;
			break;
		case SecondPlus_Pin: 
			if(alarmFlag==1) 
				alarm_time.AlarmTime.Seconds++;
			HAL_RTC_SetAlarm(&hrtc, &alarm_time, RTC_FORMAT_BIN);
			break;
		case SecondMinus_Pin: 
			if(alarmFlag==1) 
				alarm_time.AlarmTime.Seconds--;
			HAL_RTC_SetAlarm(&hrtc, &alarm_time, RTC_FORMAT_BIN);
			break;
		case MinutePlus_Pin: 
			if(alarmFlag==1) 
				alarm_time.AlarmTime.Minutes++;
			HAL_RTC_SetAlarm(&hrtc, &alarm_time, RTC_FORMAT_BIN);
			break;
		case MinuteMinus_Pin: 
			if(alarmFlag==1) 
				alarm_time.AlarmTime.Minutes--;
			HAL_RTC_SetAlarm(&hrtc, &alarm_time, RTC_FORMAT_BIN);
			break;
		case HourPlus_Pin: 
			if(alarmFlag==1) 
				alarm_time.AlarmTime.Hours++;
			HAL_RTC_SetAlarm(&hrtc, &alarm_time, RTC_FORMAT_BIN);
			break;
		case HourMinus_Pin: 
			if(alarmFlag==1) 
				alarm_time.AlarmTime.Hours--;
			HAL_RTC_SetAlarm(&hrtc, &alarm_time, RTC_FORMAT_BIN);
			break;
	}
	
}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only 
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
    
  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date 
  */
  sTime.Hours = 6;
  sTime.Minutes = 28;
  sTime.Seconds = 5;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
  DateToUpdate.Month = RTC_MONTH_JANUARY;
  DateToUpdate.Date = 1;
  DateToUpdate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4 
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8 
                          |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12 
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7 
                          |GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : AlarmSet_Pin SecondPlus_Pin SecondMinus_Pin MinutePlus_Pin 
                           MinuteMinus_Pin HourPlus_Pin HourMinus_Pin */
  GPIO_InitStruct.Pin = AlarmSet_Pin|SecondPlus_Pin|SecondMinus_Pin|MinutePlus_Pin 
                          |MinuteMinus_Pin|HourPlus_Pin|HourMinus_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 PA3 PA4 
                           PA5 PA6 PA7 PA8 
                           PA9 PA10 PA11 PA12 
                           PA13 PA14 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4 
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8 
                          |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12 
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB3 
                           PB4 PB5 PB6 PB7 
                           PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7 
                          |GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
