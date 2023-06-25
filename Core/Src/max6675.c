/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : max6675.c
 * @brief          : MAX6675 description
 ******************************************************************************
 * @attention
 *
 *  <h2><center>Created on: Mar 19, 2023
 *  Author: Vlad</center></h2>
 *
 ******************************************************************************
 */
/* USER CODE END Header */
#include <max6675.h>
#include <stdlib.h>
uint8_t temp[2] = { 0 };
extern SPI_HandleTypeDef hspi1;

uint32_t median(uint32_t newValue) {
	static uint32_t buf[3] = { };
	static uint8_t i = 0;
	buf[i] = newValue;
	if (++i > 2)
		i = 0;
	return 	buf[0] > buf[1] ?
				buf[0] > buf[2] ?
					buf[2] > buf[1] ? buf[2] : buf[1]
				: buf[0]
			: buf[1] > buf[2] ?
				buf[2] > buf[0] ? buf[2] : buf[0]
			: buf[1];
}

uint32_t expRunningAverageAdaptive(uint32_t newVal) {
	static uint32_t filVal = 0;
	//double k = (abs(newVal - filVal) > 150) ? 0.9 : 0.03;
	uint32_t kk = ((abs(newVal - filVal) > 15000) ? 90 : 3);

	filVal += (newVal - filVal) * kk;
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
		uint32_t reg_raw = ((uint32_t)(temp[0] << 5) | (uint32_t)(temp[1] >> 3)) * 25;

		reg = median(reg_raw);
		//reg = expRunningAverageAdaptive(reg);
		//reg = temp[0] | temp[1];
	}
}
/*****END OF FILE****/