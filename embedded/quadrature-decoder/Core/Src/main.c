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
#include <string.h>
#include <math.h>
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

//Counts overflows per second
static int overflowCount = 0;

//RTOS message queue
QueueHandle_t msgQueue;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
void taskEncoder(void const * argument);
void taskLCD(void const * argument);

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
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

	ST7735_Init();	//initialise the lcd display
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
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
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

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
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
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
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_RESET_Pin|LCD_DC_Pin|SPI1_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RESET_Pin LCD_DC_Pin SPI1_CS_Pin */
  GPIO_InitStruct.Pin = LCD_RESET_Pin|LCD_DC_Pin|SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

}

/* USER CODE BEGIN 4 */

/**
 * @brief GPIO interrupt callback function
 * @param GPIO_Pin the corresponding pin that raised an interrupt
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	//Check if pin 4 raised the interrupt
	if (GPIO_Pin == GPIO_PIN_4) {

		overflowCount++;
		aChannel = 0;
		bChannel = 0;

		uint32_t count = TIM2->CNT;	//get the timer count

		zChannel++;

		if (rotation == CW_ROTATION) {
			cpr = count;
			__HAL_TIM_SET_COUNTER(&htim2, 0);
		} else if (rotation == CCW_ROTATION) {
			cpr = 65535 - count;
			__HAL_TIM_SET_COUNTER(&htim2, 65535);
		}

		return;
	}
}

/**
 * @brief timer callback function for overflow
 * @param htim timer handle
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	overflowCount++;
}

/**
 * @brief draw the border around the screen
 */
void LCD_DrawBorder(void) {
	ST7735_FillScreenFast(RED);
	ST7735_FillRectangleFast(4, 4, ST7735_WIDTH - 4, ST7735_HEIGHT - 4, WHITE);
}

/**
 * @brief draw the start screen of the lcd
 */
void LCD_Start(void) {
	LCD_DrawBorder();	//draw border

	ST7735_WriteString(14, ST7735_HEIGHT / 2 - 7, "Encoder Test", Font_11x18,
			BLACK, WHITE);

	HAL_Delay(2500);

	LCD_DrawBorder();

	ST7735_WriteString(5, 14, "Count:", Font_11x18, BLACK, WHITE);
	LCD_TimerCount(0);

	ST7735_FillRectangleFast(6, 86, 60, 120, WHITE);
	ST7735_WriteString(6, 86, "A:", Font_7x10, BLACK, WHITE);
	ST7735_WriteString(6, 97, "B:", Font_7x10, BLACK, WHITE);
	ST7735_WriteString(6, 108, "Z:", Font_7x10, BLACK, WHITE);

	encoderInstance_t temp;
	temp.aCount = 0;
	temp.bCount = 0;
	temp.zCount = 0;
	LCD_ChannelCount(temp);

	ST7735_WriteString(5, 34, "P.P.R:", Font_11x18, BLACK, WHITE);
	LCD_PulsesPerRotation(0);

	ST7735_WriteString(16, 54, "DIR.:", Font_11x18, BLACK, WHITE);
	LCD_RotationDirection(NO_ROTATION);

	LCD_RpmFreq(0, 0);
	LCD_Degrees(0, 0);
}

/**
 * @brief Update the rpm, frequency
 * @param rpm rotations per minute
 * @param freq frequency of rotation
 * @param count	current encoder count
 */
void LCD_RpmFreq(double rpm, double frequency) {
	char rpmBuf[20], freqBuf[20];
	memset(rpmBuf, 0, sizeof(rpmBuf));
	memset(freqBuf, 0, sizeof(freqBuf));
	sprintf(rpmBuf, "%.1frpm", rpm);
	sprintf(freqBuf, "%.1fHz", frequency);

	ST7735_FillRectangleFast(80, 86, 154, 107, WHITE);
	if ((strlen(rpmBuf) < 10) || (strlen(freqBuf) < 10)) {
		ST7735_WriteString(80, 86, rpmBuf, Font_7x10, BLACK, WHITE);
		ST7735_WriteString(80, 97, freqBuf, Font_7x10, BLACK, WHITE);
	} else {
		ST7735_WriteString(80, 86, "error", Font_7x10, BLACK, WHITE);
		ST7735_WriteString(80, 97, "error", Font_7x10, BLACK, WHITE);
	}


}

/**
 * @brief update the position
 * @param count encoder time count
 * @param pulseRotation pulses per rotation
 */
void LCD_Degrees(int64_t count, int pulseRotation) {

	ST7735_FillRectangleFast(80, 108, 154, 120, WHITE);

	double pos;
	if (pulseRotation != 0) {
		pos = (((double) abs(count) / (double) pulseRotation) * 360);
	} else {
		pos = 0;
	}
	char posBuf[20];
	memset(posBuf, 0, sizeof (posBuf));
	sprintf(posBuf, "%.1fdeg", pos);
	if (strlen(posBuf) < 10) {
		ST7735_WriteString(80, 108, posBuf, Font_7x10, BLACK, WHITE);
	} else {
		ST7735_WriteString(80, 108, "error", Font_7x10, BLACK, WHITE);
	}
}

/**
 * @brief update the rotation direction
 * @param rotation direction
 */
void LCD_RotationDirection(int direction) {
	char *buf;

	if (direction == NO_ROTATION) {
		buf = "NULL";
	} else if (direction == CW_ROTATION) {
		buf = "CW";
	} else {
		buf = "CCW";
	}

	ST7735_FillRectangleFast(80, 54, 130, 74, WHITE);
	ST7735_WriteString(80, 54, buf, Font_11x18, BLACK, WHITE);
}

/**
 * @brief update the pulses per rotation
 * @param pulses pulses per rotation
 */
void LCD_PulsesPerRotation(int pulses) {
	char buffer[10];
	memset(buffer, 0, sizeof(buffer));
	sprintf(buffer, "%d", pulses);

	uint8_t len = strlen(buffer);
	uint8_t pos = (int) (90 - (len - 1) / 2 * 11);

	ST7735_FillRectangleFast(69, 34, 140, 56, WHITE);
	ST7735_WriteString(pos, 34, buffer, Font_11x18, BLACK, WHITE);
}

/**
 * @brief update the timer count
 */
void LCD_TimerCount(int64_t count) {
	char buffer[6];
	memset(buffer, 0, sizeof(buffer));
	sprintf(buffer, "%ld", count);

	uint8_t len = strlen(buffer);
	uint8_t pos = (int) (90 - (len - 1) / 2 * 11);

	ST7735_FillRectangleFast(70, 14, 140, 32, WHITE);
	ST7735_WriteString(pos, 14, buffer, Font_11x18, BLACK, WHITE);
}

/**
 * @brief update the channel counts
 * @param data msg struct
 */
void LCD_ChannelCount(encoderInstance_t data) {
	char buffer[5];
	memset(buffer, 0, sizeof(buffer));
	ST7735_FillRectangleFast(24, 86, 75, 120, WHITE);

	sprintf(buffer, "%ld", data.aCount);
	ST7735_WriteString(24, 86, buffer, Font_7x10, BLACK, WHITE);
	memset(buffer, 0, sizeof(buffer));

	sprintf(buffer, "%ld", data.bCount);
	ST7735_WriteString(24, 97, buffer, Font_7x10, BLACK, WHITE);
	memset(buffer, 0, sizeof(buffer));

	sprintf(buffer, "%ld", data.zCount);
	ST7735_WriteString(24, 108, buffer, Font_7x10, BLACK, WHITE);
	memset(buffer, 0, sizeof(buffer));
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_taskEncoder */
/**
 * @brief  Function implementing the encoderTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_taskEncoder */
void taskEncoder(void const * argument)
{
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
	int64_t timerCountSecond = timerCount;

	uint32_t pps = 0;	//pulses per second
	uint32_t rps = 0;
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
		} else if ((timerCount - prevTimerCount) < 0) {
			//counter clockwise rotation
			stillTime = HAL_GetTick();
			rotation = CCW_ROTATION;
		} else {
			//no rotation, no velocity
			if ((HAL_GetTick() - stillTime) >= 500) {
				rotation = NO_ROTATION;
			}
		}

		//Calculate rpm and frequency every second
		if ((HAL_GetTick() - rotationTime) >= 200) {

			if (overflowCount > 0) {
				if (timerCount < timerCountSecond) {
					pps = 4096 * overflowCount - timerCountSecond + timerCount;
				} else if (timerCount > timerCountSecond) {
					pps = 4096 * overflowCount + timerCountSecond - timerCount;
				} else {
					pps = 4096 * overflowCount;
				}
			} else {
				pps = abs(timerCount - timerCountSecond);
			}

			if (cpr == 0) {
				rpm = 0;
			} else {
				rpm = pps * 5 * 60 / cpr;
			}

			freq = 0.0166667 * rpm;
			rotationTime = HAL_GetTick();
			pps = 0;
			overflowCount = 0;
			timerCountSecond = timerCount;
		}

		prevTimerCount = timerCount;

		//Adjust timer value for user friendly value
		if (timerCount <= 10000) {
			timerCountPrint = timerCount;
		} else if (timerCount >= 50000) {
			timerCountPrint = -(65536 - timerCount);
		}

		if (timerCountPrint > 0) {
			aChannel = (int) ceil(timerCountPrint / 2.00);
			bChannel = timerCountPrint - aChannel;
		} else if (timerCountPrint < 0) {
			aChannel = (int) ceil(timerCountPrint / 2.00);
			bChannel = timerCountPrint - aChannel;
		} else {
			aChannel = 0;
			bChannel = 0;
		}

		//Every 100ms send parameters to struct
		if ((HAL_GetTick() - prevTime) >= 0) {
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
void taskLCD(void const * argument)
{
  /* USER CODE BEGIN taskLCD */

	LCD_Start();

	encoderInstance_t msgStruct;
	encoderInstance_t msgStructPrev;
	char buf[100];
	memset(buf, 0, sizeof(buf));

	/* Infinite loop */
	for (;;) {
		if (msgQueue != NULL) {
			if (xQueueReceive(msgQueue, &msgStruct, (TickType_t) 0) == pdPASS) {

				if (msgStruct.timerCount != msgStructPrev.timerCount) {
					LCD_TimerCount(msgStruct.timerCount);
				}
				if ((msgStruct.aCount != msgStructPrev.aCount)
						|| (msgStruct.bCount != msgStructPrev.bCount)
						|| (msgStruct.zCount != msgStructPrev.zCount)) {
					LCD_ChannelCount(msgStruct);
				}
				if (msgStruct.pulsesPerRotation
						!= msgStructPrev.pulsesPerRotation) {
					LCD_PulsesPerRotation(msgStruct.pulsesPerRotation);
				}
				if (msgStruct.rotationDirection
						!= msgStructPrev.rotationDirection) {
					LCD_RotationDirection(msgStruct.rotationDirection);
				}
				if (msgStruct.rpm != msgStructPrev.rpm) {
					LCD_Degrees(msgStruct.timerCount, msgStruct.pulsesPerRotation);
					LCD_RpmFreq(msgStruct.rpm, msgStruct.freq);
				}
			}
		}

		memcpy(&msgStructPrev, &msgStruct, sizeof(encoderInstance_t));
		osDelay(25);
	}
  /* USER CODE END taskLCD */
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
