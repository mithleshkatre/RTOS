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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
//#include "cmsis_os.h"


#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"
#include "event_groups.h"

#include "stdlib.h"
#include "string.h"
#include "stdio.h"

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
UART_HandleTypeDef huart2;
TaskHandle_t HPT_Handler;
void HPTask(void * argument);

TaskHandle_t MPT_Handler;
void MPTask(void * argument);

TaskHandle_t LPT_Handler;
void LPTask(void * argument);

TaskHandle_t VLPT_Handler;
void VLPTask(void * argument);

SemaphoreHandle_t semaphorecount;


//resource

uint32_t resource[]={333,222,111};

uint8_t indx = 0;

uint8_t rx_data=0;


/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);


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

semaphorecount= xSemaphoreCreateCounting(3,0);

if(semaphorecount==NULL){
	HAL_UART_Transmit(&huart2, (uint8_t)"Failed to create semaphore", 27, 300);
}
else{
	HAL_UART_Transmit(&huart2, (uint8_t)"create semaphore Succesfull", 27, 300);
}

xTaskCreate(HPTask, "HPTASK", 128, NULL, 3, &HPT_Handler);
xTaskCreate(MPTask, "MPTask", 128, NULL, 2, &MPT_Handler);
xTaskCreate(LPTask, "LPTask", 128, NULL, 1, &LPT_Handler);
xTaskCreate(VLPTask, "VLPTask", 128, NULL, 0, &VLPT_Handler);


//Schedule start

vTaskStartScheduler();

  /* USER CODE END 2 */



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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 84;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HPTask(void * argument){

	char sresource[3];
		int semcount = 0;
		char ssemcount[2];

		// Give 3 semaphores at the beginning..
		xSemaphoreGive(semaphorecount);
		xSemaphoreGive(semaphorecount);
		xSemaphoreGive(semaphorecount);

		while (1)
		{
			char str[150];
			strcpy(str, "Entered HPT Task\r\n About to ACQUIRE the Semaphore\r\n ");
			semcount = uxSemaphoreGetCount(semaphorecount);
			itoa (semcount, ssemcount, 10);
			strcat (str, "Tokens available are: ");
			strcat (str, ssemcount);
			strcat (str, "\r\n\n");
			HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen (str), HAL_MAX_DELAY);

			xSemaphoreTake(semaphorecount, portMAX_DELAY);

			itoa (resource[indx], sresource, 10);
			strcpy (str, "Leaving HPT Task\r\n Data ACCESSED is:: ");
			strcat (str, sresource);
			strcat (str, "\r\n Not releasing the Semaphore\\r n\n\n");
			HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen (str), HAL_MAX_DELAY);

			indx++;
			if (indx>2) indx=0;

			vTaskDelay(3000);
	//		vTaskDelete(NULL);
		}
}

void MPTask(void * argument){

	char sresource[3];
			int semcount = 0;
			char ssemcount[2];


		while (1)
		{
			char str[150];
			strcpy(str, "Entered MPT Task\r\n About to ACQUIRE the Semaphore\r\n ");
			semcount = uxSemaphoreGetCount(semaphorecount);
			itoa (semcount, ssemcount, 10);
			strcat (str, "Tokens available are: ");
			strcat (str, ssemcount);
			strcat (str, "\r\n\n");
			HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen (str), HAL_MAX_DELAY);

			xSemaphoreTake(semaphorecount, portMAX_DELAY);

			itoa (resource[indx], sresource, 10);
			strcpy (str, "Leaving MPT Task\r\n Data ACCESSED is:: ");
			strcat (str, sresource);
			strcat (str, "\r\n Not releasing the Semaphore\r\n\n\n");
			HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen (str), HAL_MAX_DELAY);

			indx++;
			if (indx>2) indx=0;

			vTaskDelay(2000);
	//		vTaskDelete(NULL);
		}
}

void LPTask(void * argument){

	char sresource[3];
		int semcount = 0;
		char ssemcount[2];



		while (1)
		{
			char str[150];
			strcpy(str, "Entered LPT Task\r\n About to ACQUIRE the Semaphore\r\n ");
			semcount = uxSemaphoreGetCount(semaphorecount);
			itoa (semcount, ssemcount, 10);
			strcat (str, "Tokens available are: ");
			strcat (str, ssemcount);
			strcat (str, "\r\n\n");
			HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen (str), HAL_MAX_DELAY);

			xSemaphoreTake(semaphorecount, portMAX_DELAY);

			itoa (resource[indx], sresource, 10);
			strcpy (str, "Leaving LPT Task\r\n Data ACCESSED is:: ");
			strcat (str, sresource);
			strcat (str, "\r\n Not releasing the Semaphore\r\n\n\n");
			HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen (str), HAL_MAX_DELAY);

			indx++;
			if (indx>2) indx=0;

			vTaskDelay(1000);
	//		vTaskDelete(NULL);
		}
}


void VLPTask(void * argument){

	char sresource[3];
		int semcount = 0;
		char ssemcount[2];



		while (1)
		{
			char str[150];
			strcpy(str, "Entered VLPT Task\r\n About to ACQUIRE the Semaphore\r\n ");
			semcount = uxSemaphoreGetCount(semaphorecount);
			itoa (semcount, ssemcount, 10);
			strcat (str, "Tokens available are: ");
			strcat (str, ssemcount);
			strcat (str, "\r\n\n");
			HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen (str), HAL_MAX_DELAY);

			xSemaphoreTake(semaphorecount, portMAX_DELAY);

			itoa (resource[indx], sresource, 10);
			strcpy (str, "Leaving VLPT Task\r\n Data ACCESSED is:: ");
			strcat (str, sresource);
			strcat (str, "\\rn Not releasing the Semaphore\r\n\n\n");
			HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen (str), HAL_MAX_DELAY);

			indx++;
			if (indx>2) indx=0;

			vTaskDelay(000);
	//		vTaskDelete(NULL);
		}
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	HAL_UART_Receive_IT(huart, &rx_data, 1);
	if (rx_data == 'r')
	{
		// release the semaphore here
		 /* The xHigherPriorityTaskWoken parameter must be initialized to pdFALSE as
		 it will get set to pdTRUE inside the interrupt safe API function if a
		 context switch is required. */
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;

		xSemaphoreGiveFromISR(semaphorecount, &xHigherPriorityTaskWoken);  // ISR SAFE VERSION
		xSemaphoreGiveFromISR(semaphorecount, &xHigherPriorityTaskWoken);  // ISR SAFE VERSION
		xSemaphoreGiveFromISR(semaphorecount, &xHigherPriorityTaskWoken);  // ISR SAFE VERSION

		/* Pass the xHigherPriorityTaskWoken value into portEND_SWITCHING_ISR(). If
		 xHigherPriorityTaskWoken was set to pdTRUE inside xSemaphoreGiveFromISR()
		 then calling portEND_SWITCHING_ISR() will request a context switch. If
		 xHigherPriorityTaskWoken is still pdFALSE then calling
		 portEND_SWITCHING_ISR() will have no effect */

		portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */


 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3) {
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
