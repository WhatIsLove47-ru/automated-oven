/*
 GyverRelay - библиотека классического релейного регулятора для Arduino
 Документация: https://alexgyver.ru/gyverrelay/
 - Обратная связь по скорости изменения величины
 - Настройка гистерезиса, коэффициента усиления ОС, направления регулирования
 - Возвращает результат по встроенному таймеру или в ручном режиме

 Версия 2.0 от 04.12.2019
 Версия 2.1 от 31.01.2020 - исправлена getResultTimer
 Imported on STM32 by 04.04.2021
 */

#ifndef GyverRelay_h
#define GyverRelay_h
#include <main.h>

#define NORMAL 0
#define REVERSE 1

typedef struct {
	double k;			// коэффициент усиления	по скорости (по умолч. 0)
	int32_t input;	// сигнал с датчика (например температура, которую мы регулируем)
	int32_t setpoint;// заданная величина, которую должен поддерживать регулятор (температура)
	int32_t hysteresis;				// ширина окна гистерезиса
	int32_t prevTime;
	int32_t prevInput;
	//int16_t dT;				// время итерации, мс (по умолч. секунда)
	_Bool _direction;
	_Bool output;					// выход регулятора (0 или 1)
	_Bool oldOutput;
} relay_t;

// принимает установку, ширину гистерезиса, направление (NORMAL, REVERSE)
// NORMAL - включаем нагрузку при переходе через значение снизу (пример: охлаждение)
// REVERSE - включаем нагрузку при переходе через значение сверху (пример: нагрев)
void GyverRelay(relay_t *relay, double k, uint32_t setpoint,
		uint32_t hysteresis, _Bool direction); // направление регулирования (NORMAL, REVERSE)

// расчёт возвращает состояние для управляющего устройства (реле, транзистор) (1 вкл, 0 выкл)
_Bool GyverRelay_compute(relay_t *relay_t, double dt);	// моментальный расчёт. Принимает dt в секундах для режима с ОС
_Bool GyverRelay_getResult(relay_t *relay_t);// моментальный расчёт. Встроенный таймер для режима с ОС
_Bool GyverRelay_getResultTimer(relay_t *relay_t);			// расчёт по встроенному таймеру
#endif
