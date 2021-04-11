#ifndef __MAX6675_H
#define __MAX6675_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

#define MAX6675_CS_Pin GPIO_PIN_10
#define MAX6675_CS_GPIO_Port GPIOB

#define MAX6675_CS_SET() HAL_GPIO_WritePin(MAX6675_CS_GPIO_Port, MAX6675_CS_Pin, GPIO_PIN_RESET)
#define MAX6675_CS_RESET() HAL_GPIO_WritePin(MAX6675_CS_GPIO_Port, MAX6675_CS_Pin, GPIO_PIN_SET)

extern uint32_t reg;

void Max6675_ReadReg(SPI_HandleTypeDef *hspi);

#ifdef __cplusplus
}
#endif

#endif /* INC_MAX6675_H_ */
