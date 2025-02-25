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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
volatile bool mod_operare = false; // 0 = STARE AUTOMATA/TERMISTOR//
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* Definitions for task_button */
osThreadId_t task_buttonHandle;
const osThreadAttr_t task_button_attributes = {
  .name = "task_button",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for task_termistor */
osThreadId_t task_termistorHandle;
const osThreadAttr_t task_termistor_attributes = {
  .name = "task_termistor",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for task_pote */
osThreadId_t task_poteHandle;
const osThreadAttr_t task_pote_attributes = {
  .name = "task_pote",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for task_PWM */
osThreadId_t task_PWMHandle;
const osThreadAttr_t task_PWM_attributes = {
  .name = "task_PWM",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for task_uart */
osThreadId_t task_uartHandle;
const osThreadAttr_t task_uart_attributes = {
  .name = "task_uart",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for que_term */
osMessageQueueId_t que_termHandle;
const osMessageQueueAttr_t que_term_attributes = {
  .name = "que_term"
};
/* Definitions for que_pote */
osMessageQueueId_t que_poteHandle;
const osMessageQueueAttr_t que_pote_attributes = {
  .name = "que_pote"
};
/* Definitions for que_buton */
osMessageQueueId_t que_butonHandle;
const osMessageQueueAttr_t que_buton_attributes = {
  .name = "que_buton"
};
/* Definitions for que_temp_to_uart */
osMessageQueueId_t que_temp_to_uartHandle;
const osMessageQueueAttr_t que_temp_to_uart_attributes = {
  .name = "que_temp_to_uart"
};
/* Definitions for que_duty_to_uart */
osMessageQueueId_t que_duty_to_uartHandle;
const osMessageQueueAttr_t que_duty_to_uart_attributes = {
  .name = "que_duty_to_uart"
};
/* Definitions for que_state_to_uart */
osMessageQueueId_t que_state_to_uartHandle;
const osMessageQueueAttr_t que_state_to_uart_attributes = {
  .name = "que_state_to_uart"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
void start_button(void *argument);
void start_termistor(void *argument);
void start_pote(void *argument);
void start_PWM(void *argument);
void start_uart(void *argument);

/* USER CODE BEGIN PFP */
void change_channel(uint32_t channel, uint32_t  rank);
float calc_temp(uint16_t adc_value);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void change_channel(uint32_t channel, uint32_t  rank)
{
ADC_ChannelConfTypeDef sConfig = {0};
sConfig.Channel = channel;
sConfig.Rank = rank;
if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
    Error_Handler();
    }
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
  MX_TIM2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of que_term */
  que_termHandle = osMessageQueueNew (16, sizeof(float), &que_term_attributes);

  /* creation of que_pote */
  que_poteHandle = osMessageQueueNew (16, sizeof(float), &que_pote_attributes);

  /* creation of que_buton */
  que_butonHandle = osMessageQueueNew (16, sizeof(bool), &que_buton_attributes);

  /* creation of que_temp_to_uart */
  que_temp_to_uartHandle = osMessageQueueNew (16, sizeof(float), &que_temp_to_uart_attributes);

  /* creation of que_duty_to_uart */
  que_duty_to_uartHandle = osMessageQueueNew (16, sizeof(uint16_t), &que_duty_to_uart_attributes);

  /* creation of que_state_to_uart */
  que_state_to_uartHandle = osMessageQueueNew (16, sizeof(bool), &que_state_to_uart_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of task_button */
  task_buttonHandle = osThreadNew(start_button, NULL, &task_button_attributes);

  /* creation of task_termistor */
  task_termistorHandle = osThreadNew(start_termistor, NULL, &task_termistor_attributes);

  /* creation of task_pote */
  task_poteHandle = osThreadNew(start_pote, NULL, &task_pote_attributes);

  /* creation of task_PWM */
  task_PWMHandle = osThreadNew(start_PWM, NULL, &task_PWM_attributes);

  /* creation of task_uart */
  task_uartHandle = osThreadNew(start_uart, NULL, &task_uart_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 500;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

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

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

float calc_temp(uint16_t adc_value)
{
    float voltaj;
    float temp;
	float rez;

    voltaj = (float)adc_value * 3.3 /  4096.0;
    rez =  10000.0 * voltaj / (3.3 - voltaj);
    temp = (1.0 / ((1.0 / 298.15) + (1.0 / 3950.0) * log(rez / 10000.0))) - 273.15;

    return temp;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_start_button */
/**
  * @brief  Function implementing the task_button thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_start_button */
void start_button(void *argument)
{
  /* USER CODE BEGIN 5 */
	uint8_t lastButtonState = GPIO_PIN_RESET;
	uint8_t currentButtonState;

  /* Infinite loop */
	for(;;)
		  {
			currentButtonState = HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin);
			if (currentButtonState == GPIO_PIN_SET && lastButtonState == GPIO_PIN_RESET)
				{
				mod_operare = !mod_operare;
				if (!mod_operare)
					{
					osThreadResume(task_poteHandle);
				    osThreadSuspend(task_termistorHandle);
				    }
				else
					{
					osThreadResume(task_termistorHandle);
				    osThreadSuspend(task_poteHandle);
				    }
				}
			lastButtonState = currentButtonState;
			osDelay(20);
		  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_start_termistor */
/**
* @brief Function implementing the task_termistor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_start_termistor */
void start_termistor(void *argument)
{
  /* USER CODE BEGIN start_termistor */
	uint32_t val_adc_T;
	float temperatura;

  /* Infinite loop */
  for(;;)
  {
	  if (mod_operare) {
	  change_channel(1,1);
	  if(HAL_ADC_Start(&hadc1) != HAL_OK)
	  	  {
	      continue;
	      }
	  if(HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
	  	  {
	      val_adc_T = HAL_ADC_GetValue(&hadc1);
	      temperatura = calc_temp(val_adc_T);
	      osMessageQueuePut(que_temp_to_uartHandle, &temperatura, 0, 0);
	      if(osMessageQueuePut(que_termHandle, &temperatura, 0, 0) != osOK) {

	      	  }
	      }
	      HAL_ADC_Stop(&hadc1);
	      osDelay(1000);
	  	  }
	  else
	  	  {
		  osThreadSuspend(task_termistorHandle);
	  	  }

  }
  /* USER CODE END start_termistor */
}

/* USER CODE BEGIN Header_start_pote */
/**
* @brief Function implementing the task_pote thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_start_pote */
void start_pote(void *argument)
{
  /* USER CODE BEGIN start_pote */
	uint16_t val_adc_P;
	float voltaj;
  /* Infinite loop */
  for(;;)
  {
	  if (!mod_operare)
	  	  {
		  change_channel(4,1);
	  	  if(HAL_ADC_Start(&hadc1) != HAL_OK)
	  	  	  {
	  	      continue;
	  	      }
	  	  if(HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
	  	  	  {
	  	      val_adc_P = HAL_ADC_GetValue(&hadc1);
	  	      voltaj = (float)val_adc_P * (3.3 / 4096.0);
	  	      if(osMessageQueuePut(que_poteHandle, &voltaj, 0, 0) != osOK)
	  	      	  {

	  	      	  }
	  	       }
	  	  HAL_ADC_Stop(&hadc1);
	  	  osDelay(1);
	  	  }
    else
    	{
    	osThreadSuspend(task_poteHandle);
    	}
  }
  /* USER CODE END start_pote */
}

/* USER CODE BEGIN Header_start_PWM */
/**
  * @brief  Function implementing the task_PWM thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_start_PWM */
void start_PWM(void *argument)
{
  /* USER CODE BEGIN start_PWM */
	float temperatura;
	float voltag;
	uint32_t duty;
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  /* Infinite loop */
  for(;;)
  {
  if(osMessageQueueGet(que_termHandle, &temperatura, 0, 0) == osOK)
  	  {
	  duty = (uint32_t)(temperatura * 100.0);
	  osMessageQueuePut(que_duty_to_uartHandle, &duty, 0, 0);
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty);
	  }
  if (osMessageQueueGet(que_poteHandle, &voltag, 0, 0) == osOK)
  	  {
	  duty = (uint32_t)((voltag / 3.3) * 100);
	  osMessageQueuePut(que_duty_to_uartHandle, &duty, 0, 0) ;
	  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, duty);
	  }

  }
  /* USER CODE END start_PWM */
}

/* USER CODE BEGIN Header_start_uart */
/**
* @brief Function implementing the task_uart thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_start_uart */
void start_uart(void *argument)
{
  /* USER CODE BEGIN start_uart */
	  char buffer[30];
	  float temperatura;
	  uint32_t factor_umplere;
	  uint32_t temp_conv = 0;
	  bool mod_functionare;

	  MX_USART2_UART_Init();
  /* Infinite loop */
  for(;;)
  {

	  osMessageQueueGet(que_temp_to_uartHandle, &temperatura, 0, 0);
	  temp_conv = (uint32_t)temperatura;
	  snprintf(buffer, sizeof(buffer), "%d Grade Celsius\n\r", (int)temperatura);
	  HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 100);
	  osMessageQueueGet(que_duty_to_uartHandle, &factor_umplere, 0, 0);
	  snprintf(buffer, sizeof(buffer), "%u factor de umplere\n\r", (int)factor_umplere);
	  HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 100);

	  if (mod_functionare)
	  	  {
		  mod_functionare = mod_operare;
		  snprintf(buffer, sizeof(buffer), "mod_AUTOMAT\n\r\n");
	  	  }
	  else
	  	  {
		  mod_functionare = mod_operare;
		  snprintf(buffer, sizeof(buffer), "mod_MANUAL\n\r\n");
	  	  }

	  HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), 100);
	  osDelay(50);
  }
  /* USER CODE END start_uart */
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
