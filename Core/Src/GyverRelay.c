/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : GyverRelay.c
 * @brief          : Class relay funcs
 ******************************************************************************
 * @attention
 *
 *  <h2><center>Created on: Mar 24, 2023
 *  Author: Vlad</center></h2>
 *
 ******************************************************************************
 */
/* USER CODE END Header */
#include <GyverRelay.h>

void GyverRelay(relay_t *relay, double k, uint32_t setpoint,
		uint32_t hysteresis, _Bool direction) {
	//relay->dT = 1000;				// время итерации, мс (по умолч. секунда)
	relay->k = k;      // коэффициент обратной связи (подбирается по факту)
	relay->setpoint = setpoint;  // установка (ставим на SETPOINT градусов)
	relay->hysteresis = hysteresis;  // ширина гистерезиса
	relay->_direction = direction;
	relay->output = !relay->_direction;   // выключить реле сразу
	relay->oldOutput = relay->_direction;
}

int signum(int32_t val) {
	return ((val > 0) ? 1 : ((val < 0) ? -1 : 0));
}

// вернёт выход, принимает время итерации в секундах
// моментальный расчёт. Принимает dt в секундах для режима с ОС
_Bool GyverRelay_compute(relay_t *relay, double dt) {
	int32_t signal;
	if (dt > 0) {
		double rate = (relay->input - relay->prevInput) / dt; // производная от величины (величина/секунду)
		relay->prevInput = relay->input;
		signal = relay->input + rate * relay->k;
	} else {
		signal = relay->input;
	}

	// жуткая функция реле из лекций по ТАУ
	int8_t f = (signum(signal - relay->setpoint - relay->hysteresis / 2)
			+ signum(signal - relay->setpoint + relay->hysteresis / 2)) / 2;

	if (f == 1)
		relay->output = !relay->_direction;
	else if (f == -1)
		relay->output = relay->_direction;
	return relay->output;
}

_Bool GyverRelay_getResult(relay_t *relay) {
	GyverRelay_compute(relay, (HAL_GetTick() - relay->prevTime) / 1000.0);
	relay->prevTime = HAL_GetTick();
	return relay->output;
}

_Bool GyverRelay_getResultTimer(relay_t *relay) {
//	GyverRelay_compute(relay, relay->dT / 1000.0);
	GyverRelay_compute(relay, 1.0);
	return relay->output;
}
/*****END OF FILE****/