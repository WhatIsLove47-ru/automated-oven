/*
 * hotplaces.h
 *
 *  Created on: Mar 18, 2023
 *      Author: Vlad
 */

#ifndef __HOTPLACES_H
#define __HOTPLACES_H

#ifdef __cplusplus
extern "C" {
#endif

//#include <main.h>
#include <GyverRelay.h>
#include <templates.h>

typedef enum {
	ZERO = 0x00u,
	ONE = 0x11,
	TWO = 0x0A,
	THREE = 0x0C,
	FOUR = 0x14,
	FIVE = 0x1C,
	SIX = 0x1D
} HOTPLACE_STATE;

typedef struct {
	RTC_TimeTypeDef time;
	HOTPLACE_STATE state;
} Hotplace_Profile;

typedef struct {
	Hotplace_Profile *profile;
	uint8_t size;
} Hotplace_Steps;

typedef struct {
	Hotplace_Profile profile;
	Hotplace_Steps *algorithm;
	uint8_t step;
	_Bool isOn;
} Hotplace;

typedef enum {
	OVEN_ZERO = 0x00u,
	POWERLESS = 0x15,
	LOWER = 0x16,
	UPPER = 0x1A,
	POWERFUL = 0x1E
} OVEN_STATE;

typedef struct {
	RTC_TimeTypeDef time;
	uint16_t targetTemp; // 0 - 300
	OVEN_STATE state;
} Oven_Profile;

typedef struct {
	Oven_Profile *profile;
	uint8_t size;
} Oven_Steps;

typedef struct {
	Oven_Profile profile;
	Oven_Steps *algorithm;
	uint8_t step;
	_Bool isOn;
} Oven;

#define cs_set() HAL_GPIO_WritePin(SR_GPIO_Port, SR_Pin, GPIO_PIN_RESET)
#define cs_reset() HAL_GPIO_WritePin(SR_GPIO_Port, SR_Pin, GPIO_PIN_SET)
#define cs_strob() cs_reset();\
	cs_set()

extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim3;
extern SoundTypeDef Notes[MUSICSIZE];

void SetState(Oven *oven, Hotplace *hotplace);
#ifdef Debug_WIFI
void WIFIDebug(Oven *oven);
#endif
void TEMPLATE(SetCustomProfile, Hotplace)(Hotplace *hotplace, uint8_t arg, uint8_t state,
		RTC_TimeTypeDef *time);
void TEMPLATE(SetCustomProfile, Oven)(Oven *oven, uint8_t state, uint16_t targetTemp,
		RTC_TimeTypeDef *time);
uint8_t TEMPLATE(GetState, HOTPLACE_STATE)(HOTPLACE_STATE hotplace_temp);
uint8_t TEMPLATE(GetState, OVEN_STATE)(OVEN_STATE oven_state);
uint8_t TEMPLATE(isNewState, Hotplace)(Hotplace *hotplace);
uint8_t TEMPLATE(isNewState, Oven)(Oven *hotplace);

#ifdef __cplusplus
}
#endif

#endif /* INC_HOTPLACES_H_ */
/*****END OF FILE****/