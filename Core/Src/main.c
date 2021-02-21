/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
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
UART_HandleTypeDef huart2;

osThreadId blink01Handle;
osThreadId blink02Handle;
osThreadId readButton03Handle;
osTimerId periodicTimerHandle;
osTimerId onceTimerHandle;
osMutexId myMutex01Handle;
/* USER CODE BEGIN PV */

static uint32_t Led_Delay = 250;
static uint32_t Delay_Multiplier = 4;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void StartBlink01(void const * argument);
void StartBlink02(void const * argument);
void StartReadButton03(void const * argument);
void PTCallback(void const * argument);
void OTCallback(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len)
{
	HAL_UART_Transmit(&huart2,(uint8_t *)ptr,len,10);
	return len;
}
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
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  char *str0 = "\r\n\r\nInit Done\r\n";
  printf(str0);

  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of myMutex01 */
  osMutexDef(myMutex01);
  myMutex01Handle = osMutexCreate(osMutex(myMutex01));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of periodicTimer */
  osTimerDef(periodicTimer, PTCallback);
  periodicTimerHandle = osTimerCreate(osTimer(periodicTimer), osTimerPeriodic, NULL);

  /* definition and creation of onceTimer */
  osTimerDef(onceTimer, OTCallback);
  onceTimerHandle = osTimerCreate(osTimer(onceTimer), osTimerOnce, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of blink01 */
  osThreadDef(blink01, StartBlink01, osPriorityNormal, 0, 1024);
  blink01Handle = osThreadCreate(osThread(blink01), NULL);

  /* definition and creation of blink02 */
  osThreadDef(blink02, StartBlink02, osPriorityNormal, 0, 1024);
  blink02Handle = osThreadCreate(osThread(blink02), NULL);

  /* definition and creation of readButton03 */
  osThreadDef(readButton03, StartReadButton03, osPriorityBelowNormal, 0, 1024);
  readButton03Handle = osThreadCreate(osThread(readButton03), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartBlink01 */
/**
  * @brief  Function implementing the blink01 thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartBlink01 */
void StartBlink01(void const * argument)
{
  /* USER CODE BEGIN 5 */
	/* Infinite loop */
	//osTimerStart(periodicTimerHandle, 5000U);
	for(;;)
	{
		HAL_GPIO_TogglePin(GPIOA, LD2_Pin);
		osDelay(Led_Delay * Delay_Multiplier);
	}
	// In case we accidentally exit from task loop
	osThreadTerminate(NULL);
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartBlink02 */
/**
* @brief Function implementing the blink02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBlink02 */
void StartBlink02(void const * argument)
{
  /* USER CODE BEGIN StartBlink02 */
	//void *char1, *char2, *char3, *char4;
	/* Infinite loop */
	for(;;)
	{
		if (1)
		{
			if (osMutexWait(myMutex01Handle, 10U) == osOK)
			{
				char *str1 = "Bl02> Mutex taken\r\n";
				printf(str1);
				if (Delay_Multiplier != 4)
				{
					Delay_Multiplier = 4;
					char *str2 = "Bl02> Multiplier set to 4\r\n";
					printf(str2);
				}
				osDelay(5000);
				osMutexRelease(myMutex01Handle);
				char *str3 = "Bl02> Mutex released\r\n";
				printf(str3);
			}
			else
			{
				char *str4 = "Bl02> !! Mutex NOT taken\r\n";
				printf(str4);
				osDelay(5000);
			}
			osDelay(5000);
		}
		else
		{
			osDelay(0);
		}
	}
  // In case we accidentally exit from task loop
  osThreadTerminate(NULL);
  /* USER CODE END StartBlink02 */
}

/* USER CODE BEGIN Header_StartReadButton03 */
/**
* @brief Function implementing the readButton03 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartReadButton03 */
void StartReadButton03(void const * argument)
{
  /* USER CODE BEGIN StartReadButton03 */
	/* Infinite loop */
	osTimerStop(onceTimerHandle);
	for(;;)
	{
		if(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin))
		{
			osDelay(0);
		}
		else
		{
			// Button pushed
			if (Delay_Multiplier != 1)
			{
				if (osMutexWait(myMutex01Handle, 10U) == osOK)
				{
					char *str03_1 = ">Bt03>>> Mutex taken\r\n";
					printf(str03_1);
					osTimerStop(onceTimerHandle);
					//osTimerStart(onceTimerHandle, 5000U);
					//char *str03_2 = ">Bt03> One Shot Timer started\r\n";
					//printf(str03_2);
					Delay_Multiplier = 1;
					char *str03_3 = ">Bt03>>> Multiplier set to 1\r\n";
					printf(str03_3);
					osMutexRelease(myMutex01Handle);
					char *str03_4 = ">Bt03>>> Mutex released\r\n";
					printf(str03_4);
				}
				else
				{
					char *str03_5 = ">Bt03>>> !! Mutex NOT taken\r\n";
					printf(str03_5);
				}
			}
			osDelay(10);

		}
		osDelay(1);
	}
  /* USER CODE END StartReadButton03 */
}

/* PTCallback function */
void PTCallback(void const * argument)
{
  /* USER CODE BEGIN PTCallback */
	//HAL_UART_Transmit(&huart2, "Sending from PERIODIC TIMER\r\n", 29, 10);
  /* USER CODE END PTCallback */
}

/* OTCallback function */
void OTCallback(void const * argument)
{
  /* USER CODE BEGIN OTCallback */
	Delay_Multiplier = 4;
	char *str3 = ">>>> Multiplier set to 4\r\n";
	printf(str3);
  /* USER CODE END OTCallback */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
