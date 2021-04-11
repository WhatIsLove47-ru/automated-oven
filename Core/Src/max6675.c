#include <max6675.h>
#include <stdlib.h>
uint8_t temp[2] = { };
extern SPI_HandleTypeDef hspi1;

int32_t median(int32_t newValue) {
	static int32_t buf[3] = { };
	static uint8_t i = 0;
	buf[i] = newValue;
	if (++i > 2)
		i = 0;
	return buf[0] > buf[1] ?
			buf[0] > buf[2] ? buf[2] > buf[1] ? buf[2] : buf[1] : buf[0]
			: buf[1] > buf[2] ? buf[2] > buf[0] ? buf[2] : buf[0] : buf[1];
}

int32_t expRunningAverageAdaptive(int32_t newVal) {
	static int32_t filVal = 0;
	double k = (abs(newVal - filVal) > 150) ? 0.9 : 0.03;

	filVal += (newVal - filVal) * k;
	return filVal;
}
// Function return microprocessor's value
void Max6675_ReadReg(SPI_HandleTypeDef *hspi) {

	if (hspi->State != HAL_SPI_STATE_READY)
		return;
	//Set on microprocessor
	MAX6675_CS_SET();

	//Read from microprocessor
	HAL_SPI_Receive_DMA(hspi, temp, 2);
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
	if (hspi == &hspi1) {
		//Set off microprocessor
		MAX6675_CS_RESET();
		//Check connection
		if (temp[1] & 0x04)
			return; //MAX6675_ERROR;
		// Translating the data obtained
		uint32_t reg_raw = (uint32_t)(((uint16_t)temp[0] << 5) | ((uint16_t)temp[1] >> 3)) * 25;

		reg = median(reg_raw);
		reg = expRunningAverageAdaptive((int32_t)reg);
	}
}
