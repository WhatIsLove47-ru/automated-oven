 /* USER CODE BEGIN Header */
 /**
  ******************************************************************************
  * @file           : hotplaces.c
  * @brief          : Classes hotplace and oven
  ******************************************************************************
  * @attention
  *
  *  <h2><center>Created on: Mar 18, 2023
  *  Author: Vlad</center></h2>
  *
  ******************************************************************************
  */
  /* USER CODE END Header */
#include <hotplaces.h>
#include <max6675.h>
#include <string.h>
#include <all_possible_NextStep.h>
#include <esp_01.h>
#include <stdio.h>

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

static inline HOTPLACE_STATE GetHotplaceTempByInt(uint8_t state) {
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

void SetState(Oven *oven, Hotplace *hotplace) {
	counter = 0;
	ovenTxBuffer[0] = (uint8_t) hotplace[0].profile.state;
	ovenTxBuffer[1] = (uint8_t) hotplace[1].profile.state;
	ovenTxBuffer[2] = (uint8_t) hotplace[2].profile.state;
	ovenTxBuffer[3] = (uint8_t) oven->profile.state;
	while (HAL_SPI_Transmit_DMA(&hspi1, (uint8_t*) ovenTxBuffer, 1) != HAL_OK)
		;
}
#ifdef Debug_WIFI
void WIFIDebug(Oven *oven) {
	char buf[50];
	sprintf(buf, "%s%d%s%d%s%d", "SETPOINT:", (int) relay.setpoint, ",INPUT:", (int) relay.input, ",OUTPUT:", (int) relay.output);
	ESP01_SendData(buf);
}
#endif

void TEMPLATE(SetCustomProfile, Hotplace)(Hotplace *hotplace, uint8_t arg,
		uint8_t state, RTC_TimeTypeDef *time) {
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

void TEMPLATE(SetCustomProfile, Oven)(Oven *oven, uint8_t state,
		uint16_t targetTemp, RTC_TimeTypeDef *time) {
	double k;
	const uint32_t hysteresis = 2000;
	custProfileO.time = *time;
	custProfileO.targetTemp = targetTemp;
	custProfileO.state = GetOvenStateByInt(&k, state);
	oven->algorithm = &custAlgorithmO;
	oven->step = 0xff;
	memset(&oven->profile.time, 0, sizeof(RTC_TimeTypeDef));
	oven->isOn = 1;
	GyverRelay(&relay, k, (uint32_t) targetTemp * 100, hysteresis, REVERSE);
#ifdef Debug_WIFI
	char buf[50];
	sprintf(buf, "%s%d%s%d%s%d", "K:", (int) (relay.k * 100),",HYSTERESIS:", (int) relay.hysteresis, ",TARGET:", (int) relay.setpoint);
	ESP01_SendData(buf);
#endif
}

uint8_t TEMPLATE(GetState, HOTPLACE_STATE)(HOTPLACE_STATE hotplace_temp) {
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
uint8_t TEMPLATE(GetState, OVEN_STATE)(OVEN_STATE oven_state) {
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

uint8_t TEMPLATE(isNewState, Hotplace)(Hotplace *hotplace) {
	if (hotplace->isOn) {
		if (hotplace->profile.time.Seconds > 0) {
			hotplace->profile.time.Seconds--;
		} else if (hotplace->profile.time.Minutes > 0) {
			hotplace->profile.time.Seconds = 59;
			hotplace->profile.time.Minutes--;
		} else if (hotplace->profile.time.Hours > 0) {
			hotplace->profile.time.Seconds = 59;
			hotplace->profile.time.Minutes = 59;
			hotplace->profile.time.Hours--;
		} else {
			return 1 + TEMPLATE(NextStep, Hotplace)(hotplace);
		}
	}
	return 0;
}

uint8_t TEMPLATE(isNewState, Oven)(Oven *oven) {
	uint8_t r = 0;
	if (oven->isOn) {
		relay.input = reg;
		if (relay.oldOutput != GyverRelay_getResultTimer(&relay)) {
			relay.oldOutput = relay.output;
			if (relay.output) {
				oven->profile.state = oven->algorithm->profile[oven->step].state;
			} else
				oven->profile.state = OVEN_ZERO;
			r = 1;
		}
		if (oven->profile.time.Seconds > 0) {
			oven->profile.time.Seconds--;
		} else if (oven->profile.time.Minutes > 0) {
			oven->profile.time.Seconds = 59;
			oven->profile.time.Minutes--;
		} else if (oven->profile.time.Hours > 0) {
			oven->profile.time.Seconds = 59;
			oven->profile.time.Minutes = 59;
			oven->profile.time.Hours--;
		} else {
			r = 1 + TEMPLATE(NextStep, Oven)(oven);
		}
	}
	return r;
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
/*****END OF FILE****/