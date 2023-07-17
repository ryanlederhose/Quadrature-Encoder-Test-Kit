/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
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
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "math.h"

#define NO_ROTATION 0
#define CW_ROTATION 1
#define CCW_ROTATION 2

typedef struct __attribute__((packed)) _encoderInstance {
	int64_t timerCount;
	double rpm;
	double freq;
	uint8_t rotationDirection;
	uint32_t aCount;
	uint32_t bCount;
	uint32_t zCount;
	int pulsesPerRotation;
} encoderInstance_t;

#define MAX_SIZE 150

typedef struct {
	int data[MAX_SIZE];
	int top;
} FILOBuffer;
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

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
#define LCD_DC_Pin GPIO_PIN_13
#define LCD_DC_GPIO_Port GPIOB
#define LCD_RESET_Pin GPIO_PIN_14
#define LCD_RESET_GPIO_Port GPIOB
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOB
#define A_CHANNEL_Pin GPIO_PIN_5
#define A_CHANNEL_GPIO_Port GPIOB
#define A_CHANNEL_EXTI_IRQn EXTI9_5_IRQn
#define B_CHANNEL_Pin GPIO_PIN_6
#define B_CHANNEL_GPIO_Port GPIOB
#define B_CHANNEL_EXTI_IRQn EXTI9_5_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
