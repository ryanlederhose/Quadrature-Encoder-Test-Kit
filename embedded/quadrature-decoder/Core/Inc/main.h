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
#include "st7735.h"

#define NO_ROTATION 0
#define CW_ROTATION 1
#define CCW_ROTATION 2

#define	BLACK   0x0000
#define	BLUE    0x001F
#define	RED 	0xF800
#define	GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF
#define RGB(r, g, b) (((r & 0xF8) << 8) | ((g & 0xFC) << 3) | ((b & 0xF8) >> 3))

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

/**
 * @brief draw the start screen of the lcd
 */
void LCD_Start(void);

/**
 * @brief draw the border around the screen
 */
void LCD_DrawBorder(void);

/**
 * @brief update the timer count
 */
void LCD_TimerCount(int64_t count);

/**
 * @brief update the channel counts
 * @param data msg struct
 */
void LCD_ChannelCount(encoderInstance_t data);

/**
 * @brief update the pulses per rotation
 * @param pulses pulses per rotation
 */
void LCD_PulsesPerRotation(int pulses);

/**
 * @brief update the rotation direction
 * @param rotation direction
 */
void LCD_RotationDirection(int direction);

/**
 * @brief Update the rpm, frequency
 * @param rpm rotations per minute
 * @param freq frequency of rotation
 * @param count	current encoder count
 */
void LCD_RpmFreq(double rpm, double frequency);

/**
 * @brief update the position
 * @param count encoder time count
 * @param pulseRotation pulses per rotation
 */
void LCD_Degrees(int64_t count, int pulseRotation);

/**
 * @brief timer callback function for overflow
 * @param htim timer handle
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
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
#define LCD_RESET_Pin GPIO_PIN_12
#define LCD_RESET_GPIO_Port GPIOB
#define LCD_DC_Pin GPIO_PIN_13
#define LCD_DC_GPIO_Port GPIOB
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
