 /* USER CODE BEGIN Header */
 /**
  ******************************************************************************
  * @file           : all_possible_NextStep.c
  * @brief          : All posible templates for NextStep_as_template.c
  ******************************************************************************
  * @attention
  *
  *  <h2><center>Created on: Mar 24, 2023
  *  Author: Vlad</center></h2>
  *
  ******************************************************************************
  */
  /* USER CODE END Header */
#include <templates.h>
#include <hotplaces.h>
#include <all_possible_NextStep.h>
#ifdef T1
#undef T1
#endif
#define T1 Hotplace
#include "NextStep_as_template.c"
#ifdef T1
#undef T1
#endif
#define T1 Oven
#include "NextStep_as_template.c"
/*****END OF FILE****/
