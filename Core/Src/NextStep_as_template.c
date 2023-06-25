 /* USER CODE BEGIN Header */
 /**
  ******************************************************************************
  * @file           : NextStep_as_template.c
  * @brief          : Next Step for hotplace by timer
  ******************************************************************************
  * @attention
  *
  *  <h2><center>Created on: Mar 24, 2023
  *  Author: Vlad</center></h2>
  *
  ******************************************************************************
  */
  /* USER CODE END Header */
#ifdef T1

#include <templates.h>
#include <string.h>

uint8_t TEMPLATE(NextStep, T1)(T1 *hotplace) {
	if (++hotplace->step < hotplace->algorithm->size) {
		hotplace->profile = hotplace->algorithm->profile[hotplace->step];
		return 0; // Just next step
	}
	else {
		hotplace->isOn = 0;
		memset(&hotplace->profile, 0, sizeof(typeof(hotplace->profile)));
		return 1; // Let's start the PARTY
	}
}

#endif
/*****END OF FILE****/