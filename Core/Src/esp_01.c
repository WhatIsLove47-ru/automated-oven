/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : esp_01.c
 * @brief          : Commands for debug via Wi-Fi
 ******************************************************************************
 * @attention
 *
 *  <h2><center>Created on: Mar 24, 2023
 *  Author: Vlad</center></h2>
 *
 ******************************************************************************
 */
 /* USER CODE END Header */
#ifdef Debug_WIFI
#include "esp_01.h"
#include <stdio.h>
#include <string.h>
#include <main.h>

extern UART_HandleTypeDef huart3;

const char at_test[] = "AT+\r\n";
const char at_echo[] = "AT+E\r\n";
const char at_sta[] = "AT+WPRT=0\r\n";
const char at_ssid[] = "AT+SSID=\"SSID\"\r\n";
const char at_key[] = "AT+KEY=1,0,\"PARROLL\"\r\n";
const char at_dhcp[] = "AT+NIP=0\r\n";
const char at_wjoin[] = "AT+WJOIN\r\n";
const char at_socet_create[] = "AT+SKCT=0,0,\"192.168.31.203\",9076,1000\r\n";
const char at_socet_send[] = "AT+SKSND=1,";
const char at_socet_close[] = "AT+SKCLS=1\r\n";

#define RX_BUF_SIZE 1024//8192  это уже 13 бит
#define RX_BUF_MASK RX_BUF_SIZE-1
char rx_buf[RX_BUF_SIZE] = "";
volatile uint16_t rx_pos = 0;
char sendbuf[50];

uint8_t check_answer(char *str, uint16_t wait) {
	uint32_t time_Ms_Now = HAL_GetTick();
	char *istr = NULL;
	while (istr == NULL && ((HAL_GetTick() - time_Ms_Now) < wait)) {
		istr = strstr(rx_buf, str);
	}
	if (istr != NULL) {
		clear_rx_buf();
		return 1;
	} else
		return 0;
}

void esp_Start(void) {
	HAL_UART_Transmit(&huart3,(uint8_t*)RESTART,RESTART_LENGTH,100);
	uint8_t step = 0;

	while (1) {

		switch (step) {
		case (0):
			HAL_UART_Transmit(&huart3, (uint8_t*) RESTART, RESTART_LENGTH, 100);
			if (check_answer("ready", 2500))
				step++;
			break;
		case (1):
			HAL_UART_Transmit(&huart3, (uint8_t*) at_echo, strlen(at_echo),
					100);
			if (check_answer("+OK", 1000))
				step++;
			break;
		case (2):
			HAL_UART_Transmit(&huart3, (uint8_t*) at_sta, strlen(at_sta), 100);
			if (check_answer("+OK", 1000))
				step++;
			break;
		case (3):
			HAL_UART_Transmit(&huart3, (uint8_t*) at_ssid, strlen(at_ssid),
					100);
			if (check_answer("+OK", 1000))
				step++;
			break;
		case (4):
			HAL_UART_Transmit(&huart3, (uint8_t*) at_key, strlen(at_key), 100);
			if (check_answer("+OK", 1000))
				step++;
			break;
		case (5):
			HAL_UART_Transmit(&huart3, (uint8_t*) at_dhcp, strlen(at_dhcp),
					100);
			if (check_answer("+OK", 1000))
				step++;
			break;
		case (6):
			HAL_UART_Transmit(&huart3, (uint8_t*) at_wjoin, strlen(at_wjoin),
					100);
			if (check_answer("SSID", 5000))
				step++;
			break;
		}
		if (step == 7)
			break;
		HAL_Delay(50);
	}
}

void esp_Init(void) {
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
	esp_Start();
}

void clear_rx_buf(void) {
	for (; rx_pos != 0; rx_pos--) rx_buf[rx_pos] = '\0';
	//memset(rx_buf, '\0', sizeof(strlen(rx_buf)));
}

void ESP01_IRQHandler(void) {
	unsigned char rx_b;
	if (USART3->SR & USART_SR_RXNE) {
		rx_b = USART3->DR;
		rx_buf[rx_pos++] = rx_b;          //extern
		rx_pos &= RX_BUF_MASK;
		printf("%c", rx_b);
	}
	rx_buf[rx_pos + 1] = '\0';
}

void ESP01_SendData(const char *data) {
	int Slength = strlen(data);
	l: ;
	sprintf(sendbuf, "%s%d%s%c", at_socet_send, Slength, "\r\n", '\0');
	HAL_UART_Transmit(&huart3, (uint8_t*) sendbuf, strlen(sendbuf), 100);
	if (!check_answer("+OK", 100)) {
		HAL_UART_Transmit(&huart3, (uint8_t*) at_socet_create,
				strlen(at_socet_create), 100);
		check_answer("+OK", 5000);
		goto l;
	}
	HAL_UART_Transmit(&huart3, (uint8_t*) data, Slength, 100);
}
#endif
/*****END OF FILE****/