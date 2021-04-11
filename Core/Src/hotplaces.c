/*
 * hotplaces.c
 *
 *  Created on: Mar 18, 2021
 *      Author: Vlad
 */

#include <hotplaces.h>
#include <all_possible_NextStep.h>
#include <max6675.h>
#include <string.h>
#include <GyverRelay.h>

extern relay_t relay;

uint8_t ovenTxBuffer[4] = { }, counter = 0;

Hotplace_Profile custProfileH[3] = { };

Hotplace_Steps custAlgorithmH[3] = { { &custProfileH[0], 1 }, {
		&custProfileH[1], 1 }, { &custProfileH[2], 1 } };
Hotplace_Steps stepsH[10] = { };

Oven_Profile custProfileO = { };

Oven_Steps custAlgorithmO = { &custProfileO, 1 };
Oven_Steps stepsO[10] = { };

static inline OVEN_STATE GetOvenStateByInt(double *k, uint8_t state) {
	switch (state) {
	case 1:
		*k = 0.85;
		return POWERLESS;
	case 2:
		*k = 0.85;
		return LOWER;
	case 3:
		*k = 0.85;
		return UPPER;
	case 4:
		*k = 0.85;
		return POWERFUL;
	default:
		*k = 0.0;
		return OVEN_ZERO;
	}
}

static inline HOTPLACE_TEMP GetHotplaceTempByInt(uint8_t state) {
	switch (state) {
	case 1:
		return ONE;
	case 2:
		return TWO;
	case 3:
		return THREE;
	case 4:
		return FOUR;
	case 5:
		return FIVE;
	case 6:
		return SIX;
	default:
		return ZERO;
	}
}

void SetCustomOProfile(Oven *oven, uint8_t state, uint16_t targetTemp,
		RTC_TimeTypeDef *time) {
	double k;
	custProfileO.time = *time;
	custProfileO.targetTemp = targetTemp;
	custProfileO.state = GetOvenStateByInt(&k, state);
	oven->algorithm = &custAlgorithmO;
	oven->step = 0xff;
	memset(&oven->profile.time, 0, sizeof(RTC_TimeTypeDef));
	oven->isOn = 1;
	GyverRelay(&relay, k, (uint32_t) targetTemp * 100, 2000, REVERSE);
}

void SetState(Oven *oven, Hotplace *hotplace) {
	counter = 0;
	ovenTxBuffer[0] = (uint8_t) hotplace[0].profile.state;
	ovenTxBuffer[1] = (uint8_t) hotplace[1].profile.state;
	ovenTxBuffer[2] = (uint8_t) hotplace[2].profile.state;
	ovenTxBuffer[3] = (uint8_t) oven->profile.state;
	while (HAL_SPI_Transmit_DMA(&hspi1, (uint8_t*) ovenTxBuffer, 1) != HAL_OK)
		;
}

void SetCustomHProfile(Hotplace *hotplace, uint8_t arg, uint8_t state,
		RTC_TimeTypeDef *time) {
//	free(hProfile->profile);
//	hProfile->profile = (Hotplace_Profile*) malloc(
//			sizeof(Hotplace_Profile));
	memset(&hotplace->profile.time, 0, sizeof(RTC_TimeTypeDef));
	custProfileH[arg].time = *time;
	custProfileH[arg].state = GetHotplaceTempByInt(state);
	hotplace->algorithm = &custAlgorithmH[arg];
	hotplace->step = 0xff;
	hotplace->isOn = 1;
}

uint8_t GetTempH(HOTPLACE_TEMP hotplace_temp) {
	switch (hotplace_temp) {
	case ONE:
		return 1;
	case TWO:
		return 2;
	case THREE:
		return 3;
	case FOUR:
		return 4;
	case FIVE:
		return 5;
	case SIX:
		return 6;
	default:
		return 0;
	}
}
uint8_t GetStateO(OVEN_STATE oven_state) {
	switch (oven_state) {
	case POWERLESS:
		return 1;
	case LOWER:
		return 2;
	case UPPER:
		return 3;
	case POWERFUL:
		return 4;
	default:
		return 0;
	}
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
	if (hspi == &hspi1) {
		cs_strob();
		if (++counter < 4)
			while (HAL_SPI_Transmit_DMA(&hspi1,
					(uint8_t*) (ovenTxBuffer + counter), 1) != HAL_OK)
				;
	}
}
