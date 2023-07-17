/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include <stdio.h>
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
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

osThreadId encoderTaskHandle;
osThreadId lcdTaskHandle;
/* USER CODE BEGIN PV */

//Counts per rotation
static int cpr = 0;

//Rotation direction
static uint8_t rotation = 0;

//Counts for individual channels
static uint32_t aChannel = 0;
static uint32_t bChannel = 0;
static uint32_t zChannel = 0;

//RTOS message queue
QueueHandle_t msgQueue;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
void taskEncoder(void const *argument);
void taskLCD(void const *argument);

/* USER CODE BEGIN PFP */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_TIM2_Init();
	MX_USART2_UART_Init();
	MX_SPI1_Init();
	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* definition and creation of encoderTask */
	osThreadDef(encoderTask, taskEncoder, osPriorityIdle, 0, 512);
	encoderTaskHandle = osThreadCreate(osThread(encoderTask), NULL);

	/* definition and creation of lcdTask */
	osThreadDef(lcdTask, taskLCD, osPriorityHigh, 0, 512);
	lcdTaskHandle = osThreadCreate(osThread(lcdTask), NULL);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */
	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 100;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 65535;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 10;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 10;
	if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

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
	if (HAL_UART_Init(&huart2) != HAL_OK) {
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
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, LCD_DC_Pin | LCD_RESET_Pin | SPI1_CS_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin : PA4 */
	GPIO_InitStruct.Pin = GPIO_PIN_4;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : LCD_DC_Pin LCD_RESET_Pin SPI1_CS_Pin */
	GPIO_InitStruct.Pin = LCD_DC_Pin | LCD_RESET_Pin | SPI1_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : A_CHANNEL_Pin B_CHANNEL_Pin */
	GPIO_InitStruct.Pin = A_CHANNEL_Pin | B_CHANNEL_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI4_IRQn, 14, 0);
	HAL_NVIC_EnableIRQ(EXTI4_IRQn);

	HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

/**
 * @brief GPIO interrupt callback function
 * @param GPIO_Pin the corresponding pin that raised an interrupt
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	//Check if pin 4 raised the interrupt
	if (GPIO_Pin == GPIO_PIN_4) {

		uint32_t count = TIM2->CNT;	//get the timer count

		zChannel++;

		if (rotation == CW_ROTATION) {
			cpr = count;
			__HAL_TIM_SET_COUNTER(&htim2, 0);
		} else if (rotation == CCW_ROTATION) {
			cpr = 65535 - count;
			__HAL_TIM_SET_COUNTER(&htim2, 65535);
		}
		aChannel = 0;
		bChannel = 0;

		return;
	} else if (GPIO_Pin == GPIO_PIN_5) {	//check if pin 5 raised interrupt
		aChannel++;
		return;
	} else if (GPIO_Pin == GPIO_PIN_6) {	//check if pin 6 raised interrupt
		bChannel++;
		return;
	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_taskEncoder */
/**
 * @brief  Function implementing the encoderTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_taskEncoder */
void taskEncoder(void const *argument) {
	/* USER CODE BEGIN 5 */

	//Initialise message queue
	msgQueue = xQueueCreate(1, sizeof(encoderInstance_t));

	encoderInstance_t msgStruct;

	uint32_t prevTime = HAL_GetTick();
	uint32_t time = HAL_GetTick();
	uint32_t stillTime = 0;
	uint32_t rotationTime = 0;

	int64_t timerCount = 0;
	int64_t prevTimerCount = timerCount;
	int64_t timerCountPrint = timerCount;

	uint32_t pps = 0;	//pulses per second
	double rpm;
	double freq = 0;

	char buf[100];
	memset(buf, 0, sizeof(buf));

	sprintf(buf, "Start Decoding\r\n");
	HAL_UART_Transmit(&huart2, buf, sizeof(buf), 10);
	memset(buf, 0, sizeof(buf));

	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);	//start the encoder

	/* Infinite loop */
	for (;;) {

		timerCount = TIM2->CNT;	//get timer count

		//Check if the timer has changed value, set rotation direction appropriately
		if ((timerCount - prevTimerCount) > 0) {
			//clockwise rotation
			stillTime = HAL_GetTick();
			rotation = CW_ROTATION;
			pps++;
		} else if ((timerCount - prevTimerCount) < 0) {
			//counter clockwise rotation
			stillTime = HAL_GetTick();
			rotation = CCW_ROTATION;
			pps++;
		} else {
			//no rotation, no velocity
			if ((HAL_GetTick() - stillTime) >= 500) {
				rotation = NO_ROTATION;
			}
		}

		//Calculate rpm and frequency every second
		if ((HAL_GetTick() - rotationTime) >= 1000) {
			if (cpr == 0) {
				rpm = 0;
			} else {
				rpm = pps * 60 / cpr;
			}

			freq = 0.0166667 * rpm;
			rotationTime = HAL_GetTick();
			tempRotation = timerCount;
			pps = 0;
		}

		prevTimerCount = timerCount;
		prevRotation = rotation;

		//Adjust timer value for user friendly value
		if (timerCount <= 10000) {
			timerCountPrint = timerCount;
		} else if (timerCount >= 50000) {
			timerCountPrint = -(65536 - timerCount);
		}

		//Every 100ms send parameters to struct
		if ((HAL_GetTick() - prevTime) >= 100) {
			msgStruct.timerCount = timerCountPrint;
			msgStruct.rpm = rpm;
			msgStruct.freq = freq;
			msgStruct.rotationDirection = rotation;
			msgStruct.aCount = aChannel;
			msgStruct.bCount = bChannel;
			msgStruct.zCount = zChannel;
			msgStruct.pulsesPerRotation = cpr;

			//Overwrite current message so params in struct are most current
			if (msgQueue != NULL) {
				xQueueOverwrite(msgQueue, &msgStruct);
			}

			prevTime = HAL_GetTick();
		}

		osDelay(1);
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_taskLCD */
/**
 * @brief Function implementing the lcdTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_taskLCD */
void taskLCD(void const *argument) {
	/* USER CODE BEGIN taskLCD */

	//  ST7735_Init();	//initialise the lcd display
	encoderInstance_t msgStruct;
	char buf[100];
	memset(buf, 0, sizeof(buf));

	/* Infinite loop */
	for (;;) {
		if (msgQueue != NULL) {
			if (xQueueReceive(msgQueue, &msgStruct, (TickType_t) 10) == pdPASS) {
				sprintf(buf, "\r\nTimer Counter: %ld\r\n",
						msgStruct.timerCount);
				HAL_UART_Transmit(&huart2, buf, sizeof(buf), 10);
				memset(buf, 0, sizeof(buf));
//			sprintf(buf, "\r\nA Counter: %ld\r\n", msgStruct.aCount);
//			HAL_UART_Transmit(&huart2, buf, sizeof (buf), 10);
//			memset(buf, 0, sizeof (buf));
//			sprintf(buf, "\r\nB Counter: %ld\r\n", msgStruct.bCount);
//			HAL_UART_Transmit(&huart2, buf, sizeof (buf), 10);
//			memset(buf, 0, sizeof (buf));
//			sprintf(buf, "\r\nZ Counter: %ld\r\n", msgStruct.zCount);
//			HAL_UART_Transmit(&huart2, buf, sizeof (buf), 10);
//			memset(buf, 0, sizeof (buf));
//			sprintf(buf, "Rotation: %u\r\n", msgStruct.rotationDirection);
//			HAL_UART_Transmit(&huart2, buf, sizeof (buf), 10);
//			memset(buf, 0, sizeof (buf));
				sprintf(buf, "Counts per Revolution: %d\r\n",
						msgStruct.pulsesPerRotation);
				HAL_UART_Transmit(&huart2, buf, sizeof(buf), 10);
				memset(buf, 0, sizeof(buf));
//			sprintf(buf, "RPM: %lf\r\n", msgStruct.rpm);
//			HAL_UART_Transmit(&huart2, buf, sizeof (buf), 10);
//			memset(buf, 0, sizeof (buf));
			}
		}
		osDelay(100);
	}
	/* USER CODE END taskLCD */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
