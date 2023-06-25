/*
 * esp_01.h
 *
 *  Created on: Mar 24, 2023
 *      Author: WhatIsLove
 */
#ifdef Debug_WIFI
#ifndef INC_ESP_01_H_
#define INC_ESP_01_H_

#define RESTART       "AT+Z\r\n"      //Restart module
#define RESTART_LENGTH 6

//+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++//

#include <stdint.h>

void esp_Init(void);
void clear_rx_buf(void);
void ESP01_SendData(const char *);
void ESP01_IRQHandler(void);


#endif /* INC_ESP_01_H_ */
#endif
/*****END OF FILE****/