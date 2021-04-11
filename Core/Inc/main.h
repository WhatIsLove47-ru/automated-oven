/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "music.h"
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SR_Pin GPIO_PIN_1
#define SR_GPIO_Port GPIOB
#define MAX6675_CS_Pin GPIO_PIN_10
#define MAX6675_CS_GPIO_Port GPIOB
#define SPI2_CS_Pin GPIO_PIN_12
#define SPI2_CS_GPIO_Port GPIOB
#define SPI2_RST_Pin GPIO_PIN_15
#define SPI2_RST_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */
#define MUSICSIZE 48
#define PRESCALER 72
typedef enum {
	BTN_STATE_NOTHING = 0u, BTN_STATE_SHORT, BTN_STATE_LONG
} BTN_STATE;

typedef enum {
	WE_CHANGE_HOURS = 0u, WE_CHANGE_MINUTES, WE_CHANGE_SECONDS
} WE_CHANGE_TIME;

typedef struct {
	uint32_t time_key;
	IRQn_Type exti;
	GPIO_PinState key_state;
	_Bool short_state;
	_Bool long_state;
} Button;

Button btn[5];
uint16_t btns, pointer, pointer_max;
_Bool update;

music_t Music;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
