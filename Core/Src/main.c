/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "u8g2/u8g2.h"
#include <music.h>
#include <string.h>
#include <hotplaces.h>
#include <max6675.h>
#include <stdlib.h>
#include <esp_01.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define bitmap_width 64
//#define bitmap_height 16
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
Button btn[5];
uint16_t btns, pointer, pointer_max;
_Bool update;
music_t Music;
TIM_Encoder_InitTypeDef sConfig = { };
//static unsigned char bitmap[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//		0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
//		0x00, 0x00, 0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00,
//		0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x10, 0x00, 0x00,
//		0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//		0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//		0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,
//		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//		0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x20,
//		0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xE0, 255, 0x0F, 0x80, 0xFC, 0x3F,
//		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xF8, 255,
//		0x8F, 0x83, 0xFE, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//		0x00, 0x0E, 0x38, 0xE3, 0x8D, 0x83, 0x8E, 0x38, 0x00, 0x00, 0x00, 0x00,
//		0x00, 0x00, 0x00, 0x10, 0x00, 0x0F, 0x38, 0xE3, 0x8D, 0x83, 0x8E, 0x38,
//		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xC0, 0xE3,
//		0x9D, 0xC3, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//		0x00, 0x03, 0xF0, 255, 0x9D, 0xC3, 0x80, 0x3F, 0x00, 0x00, 0x00, 0x00,
//		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7C, 255, 0x18, 0xC3, 0x8E, 0x38,
//		0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x0E, 0x1C, 0xE0,
//		0x1F, 0xC3, 0x8E, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//		0x00, 0x0E, 0x1C, 0xE0, 0x1F, 0xC3, 0x8E, 0x38, 0x00, 0x00, 0x00, 0x00,
//		0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xFC, 0xE0, 0x38, 0xE3, 0xFC, 0x3F,
//		0x84, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xF0, 0xE0,
//		0x38, 0xE0, 0xF8, 0x3F, 0x80, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00,
//		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//		0x00, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x03, 0x87, 0x39, 0xC7, 0x1F,
//		0x07, 0xF8, 0x7E, 0x3F, 0x83, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03,
//		0x87, 0x39, 0xC7, 0x1F, 0x07, 0xFC, 0x7E, 0x3F, 0xC7, 0xFC, 0x00, 0x00,
//		0x00, 0x00, 0x00, 0x03, 0x87, 0xB9, 0xC7, 0x1B, 0x07, 0x1C, 0x70, 0x39,
//		0xC7, 0x1C, 0x00, 0x00, 0x00, 0x00, 0x08, 0x03, 0x87, 0xB8, 0xEE, 0x1B,
//		0x07, 0x1C, 0x70, 0x39, 0xC7, 0x9C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03,
//		0x87, 0xB8, 0xEE, 0x3B, 0x87, 0x1C, 0x7E, 0x3F, 0xC3, 0xE0, 0x00, 0x00,
//		0x00, 0x00, 0x00, 0x03, 0x87, 0xF8, 0xEE, 0x3B, 0x87, 0x1C, 0x7E, 0x3F,
//		0x81, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x87, 0x78, 0xEE, 0x31,
//		0x87, 0x1C, 0x70, 0x39, 0xC0, 0x3E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03,
//		0x87, 0x78, 0xEE, 0x3F, 0x87, 0x1C, 0x70, 0x39, 0xC7, 0x0E, 0x00, 0x00,
//		0x08, 0x08, 0x00, 0x03, 0x87, 0x78, 0xEE, 0x3F, 0x87, 0x1C, 0x70, 0x39,
//		0xC7, 0x0E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x87, 0x38, 0x7C, 0x71,
//		0xC7, 0xFC, 0x7F, 0x39, 0xC7, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03,
//		0x87, 0x38, 0x7C, 0x71, 0xC7, 0xF8, 0x7F, 0x39, 0xC1, 0xF8, 0x00, 0x00,
//		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x00, 0x00, 0x00,
//		0x00, 0x00, 0x00, 0x06, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01,
//		0xE0, 0x02, 0x08, 0x00, 0x00, 0x08, 0x20, 0x0F, 0x00, 0x00, 0x10, 0x00,
//		0x00, 0x00, 0x00, 0x03, 0xF0, 0x01, 0x10, 0x00, 0xE0, 0x04, 0x40, 0x1F,
//		0x80, 0x38, 0x00, 0x00, 0x00, 0x00, 0x80, 0x06, 0xD8, 0x03, 0xF8, 0x01,
//		0xF0, 0x0F, 0xE0, 0x36, 0xC0, 0x7C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07,
//		0xF8, 0x07, 0xFC, 0x02, 0x48, 0x1F, 0xF0, 0x3F, 0xC0, 0x92, 0x00, 0x00,
//		0x00, 0x00, 0x00, 0x07, 0xF8, 0x0E, 0xEE, 0x03, 0xF8, 0x3B, 0xB8, 0x3F,
//		0xC0, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x20, 0x0B, 0xFA, 0x01,
//		0x50, 0x2F, 0xE8, 0x09, 0x00, 0x54, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02,
//		0xD0, 0x09, 0x12, 0x02, 0xA8, 0x24, 0x48, 0x16, 0x80, 0xAA, 0x00, 0x00,
//		0x00, 0x00, 0x00, 0x05, 0x28, 0x03, 0x18, 0x00, 0x00, 0x0C, 0x60, 0x29,
//		0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//		0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//		0x00, 0x00, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40,
//		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
//		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

u8g2_t u8g2;
Oven oven = { };
Hotplace hotplace[3] = { };
relay_t relay = { };
uint32_t reg = 25; //25 degrees Celsius
//const char i0[] = "Parameter 0:";
//const char i1[] = "Parameter 1:";
//const char i2[] = "Parameter 2:";
//const char i3[] = "Parameter 3:";
//const char i4[] = "Parameter 4:";
//const char i5[] = "Parameter 5:";
//const char i6[] = "Parameter 6:";
//const char i7[] = "Parameter 7:";
//const char i8[] = "Parameter 8:";
//const char i9[] = "Parameter 9:";
//const char i10[] = "Parameter 10:";
//const char i11[] = "Parameter 11:";
//const char *const names[] = { i0, i1, i2, i3, i4, i5, i6, i7, i8, i9, i10, i11 };
SoundTypeDef Notes[MUSICSIZE] = { { C * 2, t4 }, { G, t4 }, { A_, t8 }, {
F, t8 }, { D_, t8 }, { F, t8 }, { G, t4 }, { C, t2 }, { C * 2, t4 }, {
G, t4 }, { A_, t8 }, { F, t8 }, { D_, t8 }, { F, t8 }, { G, t4 }, { C * 2, t4 },
		{ 0, t8 }, { D_, t8 }, { D_, t8 }, { D_, t8 }, { G, t8 }, {
		A_, t4 }, { D_ * 2, t8 }, { C_ * 2, t8 }, { C * 2, t8 }, { C * 2, t8 },
		{ C * 2, t8 }, { C * 2, t8 }, { A_, t8 }, { F, t8 }, { D_, t8 },
		{ F, t8 }, { G, t4 }, { C * 2, t2 }, { C * 2, t2 }, { A_, t8 },
		{ G_, t8 }, { G, t8 }, { G_, t8 }, { A_, t2 }, { A_, t4 },
		{ C * 2, t4 }, { A_, t8 }, { F, t8 }, { D_, t8 }, { F, t8 }, { G, t4 },
		{ C * 2, t2 } };
_Bool isSilentStop = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM3_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
uint8_t U8x8Stm32GPIOAndDelay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int,
		void *arg_ptr);
uint8_t U8x8ByteSTM32HWSPI(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int,
		void *arg_ptr);
static BTN_STATE EventFlag(Button *btn);
//static void RTC_TimeShow(uint8_t *showtime);
static void BtnCheck(void);
static void MainMenu(void);
static void HotplaceMenu(uint8_t arg);
static void OvenMenu(uint8_t arg);
static uint8_t TemperatureMenuH(uint8_t arg);
static uint16_t TemperatureMenuO(uint8_t arg);
static RTC_TimeTypeDef TimeMenu(uint8_t arg);
static uint16_t StateMenuO(uint8_t arg);
//static void CustomAlgorithm(uint8_t arg);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	RTC_TimeTypeDef *time = calloc(1, sizeof(RTC_TimeTypeDef));
	pointer = 0;
	btns = 0;
	pointer_max = 0xffff;
	update = 1;
	for (uint8_t i = 0; i < 5; i++) {
		btn[i].time_key = 0;
		btn[i].exti = (IRQn_Type) ((int) EXTI0_IRQn + i);
		btn[i].key_state = GPIO_PIN_SET;
		btn[i].short_state = 0;
		btn[i].long_state = 0;
	}
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	u8g2_Setup_st7920_s_128x64_f(&u8g2, U8G2_R0, U8x8ByteSTM32HWSPI,
			U8x8Stm32GPIOAndDelay);
	u8g2_InitDisplay(&u8g2); // send init sequence to the display, display is in sleep mode after this,
	u8g2_SetPowerSave(&u8g2, 0); // wake up display
	u8g2_SetFont(&u8g2, u8g2_font_6x12_t_cyrillic);
	u8g2_SetFontDirection(&u8g2, 0);

#ifdef Debug_WIFI
	esp_Init();
#endif
	TEMPLATE(SetCustomProfile, Oven)(&oven, 0, 0, time);
	free(time);
	cs_strob();
	SetState(&oven, hotplace);
	HAL_TIM_Base_Start_IT(&htim4);
	HAL_RTCEx_SetSecond_IT(&hrtc);
	MainMenu();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};

  /* USER CODE BEGIN RTC_Init 1 */
	//RTC_AlarmTypeDef sAlarm = {};
  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
  DateToUpdate.Month = RTC_MONTH_JANUARY;
  DateToUpdate.Date = 1;
  DateToUpdate.Year = 0;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

//	sAlarm.Alarm = RTC_ALARM_A;
//	sAlarm.AlarmTime.Hours = 0;
//	sAlarm.AlarmTime.Minutes = 0;
//	sAlarm.AlarmTime.Seconds = 30;
//
//	if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK) {
//		Error_Handler();
//	}
//
//	HAL_NVIC_SetPriority(RTC_Alarm_IRQn, 0x0F, 0);
//	HAL_NVIC_EnableIRQ(RTC_Alarm_IRQn);
  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 65535;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 548;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 14, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 14, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MAX6675_CS_GPIO_Port, MAX6675_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SR_GPIO_Port, SR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_RST_GPIO_Port, SPI2_RST_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : MAX6675_CS_Pin SPI2_RST_Pin */
  GPIO_InitStruct.Pin = MAX6675_CS_Pin|SPI2_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SR_Pin */
  GPIO_InitStruct.Pin = SR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_CS_Pin */
  GPIO_InitStruct.Pin = SPI2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SPI2_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 15, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
uint8_t U8x8Stm32GPIOAndDelay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int,
		void *arg_ptr) {
	/* STM32 supports HW SPI, Remove unused cases like U8X8_MSG_DELAY_XXX & U8X8_MSG_GPIO_XXX */
	switch (msg) {
	case U8X8_MSG_GPIO_AND_DELAY_INIT:
		/* Insert codes for initialization */
		break;
	case U8X8_MSG_DELAY_MILLI:
		/* ms Delay */
		//HAL_Delay(arg_int);
		break;
	case U8X8_MSG_GPIO_CS:
		/* Insert codes for SS pin control */
		HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, arg_int);
		break;
	case U8X8_MSG_GPIO_DC:
		/* Insert codes for DC pin control */
		//HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, arg_int);
		break;
	case U8X8_MSG_GPIO_RESET:
		/* Insert codes for RST pin control */
		HAL_GPIO_WritePin(SPI2_RST_GPIO_Port, SPI2_RST_Pin, arg_int);
		break;
	default:
		//u8x8_SetGPIOResult(u8x8, 1);
		return 1;
	}
	return 1;
}

uint8_t U8x8ByteSTM32HWSPI(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int,
		void *arg_ptr) {
	switch (msg) {
	case U8X8_MSG_BYTE_SEND:
		/* Insert codes to transmit data */
		if (HAL_SPI_Transmit(&hspi2, arg_ptr, arg_int, 1000) != HAL_OK)
			return 0;
		break;
	case U8X8_MSG_BYTE_INIT:
		/* Insert codes to begin SPI transmission */
		break;
	case U8X8_MSG_BYTE_SET_DC:
		/* Control DC pin, U8X8_MSG_GPIO_DC will be called */
		u8x8_gpio_SetDC(u8x8, arg_int);
		break;
	case U8X8_MSG_BYTE_START_TRANSFER:
		/* Select slave, U8X8_MSG_GPIO_CS will be called */
		u8x8_gpio_SetCS(u8x8, u8x8->display_info->chip_enable_level);
		__NOP();
		break;
	case U8X8_MSG_BYTE_END_TRANSFER:
		__NOP();
		/* Insert codes to end SPI transmission */
		u8x8_gpio_SetCS(u8x8, u8x8->display_info->chip_disable_level);
		break;
	default:
		return 0;
	}
	return 1;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	update = 1;
}

static uint32_t max4(uint32_t a, uint32_t b, uint32_t c, uint32_t d) {
	return  a > b ?
				a > c ?
					a > d ? a : d
				: c > d ? c : d
			: b > c ?
					b > d ? b : d
				: c > d ? c : d;
}

void HAL_RTCEx_RTCEventCallback(RTC_HandleTypeDef *hrtc) {
	if (RTC_IT_SEC) {
		switch (max4(TEMPLATE(isNewState, Hotplace)(&hotplace[0]),
		TEMPLATE(isNewState, Hotplace)(&hotplace[1]),
		TEMPLATE(isNewState, Hotplace)(&hotplace[2]),
		TEMPLATE(isNewState, Oven)(&oven))) {
		case 2:
			if (!isSilentStop)
				PlayMusic(&Music, Notes, &htim3, MUSICSIZE, PRESCALER,
				TIM_CHANNEL_3);
			isSilentStop = 0;
		case 1:
			SetState(&oven, hotplace);
#ifdef Debug_WIFI
			WIFIDebug(&oven);
#endif
			break;
		default:
			break;
		}
		update = 1;
	}
}

static BTN_STATE EventFlag(Button *btn) {
	if (btn->key_state == 0 && !btn->short_state
			&& (HAL_GetTick() - btn->time_key) > 50) {
		btn->short_state = 1;
		btn->long_state = 0;
		btn->time_key = HAL_GetTick();
	} else if (btn->key_state == 0 && !btn->long_state
			&& (HAL_GetTick() - btn->time_key) > 650) {
		btn->long_state = 1;
		return BTN_STATE_LONG;
	} else if (btn->key_state == 1 && btn->short_state
			&& (HAL_GetTick() - btn->time_key) > 50) {
		btn->short_state = 0;
		btn->time_key = HAL_GetTick();

		if (!btn->long_state) {
			return BTN_STATE_SHORT;
		}
	}
	__HAL_GPIO_EXTI_CLEAR_IT(btn->exti);
	NVIC_ClearPendingIRQ(btn->exti);
	HAL_NVIC_EnableIRQ(btn->exti);

	return BTN_STATE_NOTHING;
}

//static void RTC_TimeShow(uint8_t *showtime) {
//	RTC_DateTypeDef sdatestructureget;
//	RTC_TimeTypeDef stimestructureget;
//	UNUSED(stimestructureget);
//
//	/* Get the RTC current Time */
//	HAL_RTC_GetTime(&hrtc, &stimestructureget, RTC_FORMAT_BIN);
//	/* Get the RTC current Date */
//	HAL_RTC_GetDate(&hrtc, &sdatestructureget, RTC_FORMAT_BIN);
//	/* Display time Format : hh:mm:ss */
//	sprintf((char*) showtime, "%02d:%02d:%02d", stimestructureget.Hours,
//			stimestructureget.Minutes, stimestructureget.Seconds);
//}

static void BtnCheck(void) {
	static uint32_t ms = 0;
	static uint16_t temp_btns = 0;
	if (HAL_GetTick() - ms > 200) {
		if (btns != temp_btns) {
			update = 1;
			btns = temp_btns;
			StopMusic(&Music);
		}
		temp_btns = 0;
		ms = HAL_GetTick();
	} else
		for (int i = 0; i < 5; ++i) {
			temp_btns |= ((uint16_t) EventFlag(&btn[i])) << (i * 2);
		}
}

static inline void MainMenu(void) {
	RTC_TimeTypeDef time;
	pointer = 0;
	MainMenu: ;
	pointer_max = 3;
	btns = 0;
	uint8_t stime[17];
	while (1) {
		BtnCheck();
		if (btns) {
			switch (btns) {
			case 0x0001: // short right
				memset(&time, 0, sizeof(RTC_TimeTypeDef));
				time.Minutes = 30;
				if (pointer != 3)
					TEMPLATE(SetCustomProfile, Hotplace)(&hotplace[pointer], pointer, 6, &time);
				else
					TEMPLATE(SetCustomProfile, Oven)(&oven, 4, 300, &time);
				break;
			case 0x0004: // short up
				memset(&time, 0, sizeof(RTC_TimeTypeDef));
				if (pointer != 3)
					TEMPLATE(SetCustomProfile, Hotplace)(&hotplace[pointer], pointer, 0, &time);
				else
					TEMPLATE(SetCustomProfile, Oven)(&oven, 0, 0, &time);
				isSilentStop = 1;
				break;
			case 0x0100: // short center
				if (pointer != 3)
					HotplaceMenu(pointer);
				else
					OvenMenu(pointer);
				goto MainMenu;
			default:
				break;
			}
			btns = 0;
		}
		if (update) {
			update = 0;
			u8g2_FirstPage(&u8g2);
			do {
				u8g2_SetDrawColor(&u8g2, 1);
				u8g2_DrawFrame(&u8g2, 0, 1, 15, 15);
				u8g2_DrawFilledEllipse(&u8g2, 14, 1, 7, 7,
				U8G2_DRAW_LOWER_LEFT);
				u8g2_DrawFrame(&u8g2, 0, 17, 15, 15);
				u8g2_DrawFilledEllipse(&u8g2, 14, 31, 7, 7,
				U8G2_DRAW_UPPER_LEFT);
				u8g2_DrawFrame(&u8g2, 0, 33, 15, 15);
				u8g2_DrawFilledEllipse(&u8g2, 0, 47, 7, 7,
				U8G2_DRAW_UPPER_RIGHT);
				u8g2_DrawFrame(&u8g2, 0, 49, 15, 15);
				if (oven.algorithm->profile[oven.step].state == LOWER
						|| oven.algorithm->profile[oven.step].state == POWERFUL)
					u8g2_DrawFilledEllipse(&u8g2, 7, 63, 7, 3,
					U8G2_DRAW_UPPER_LEFT | U8G2_DRAW_UPPER_RIGHT);
				if (oven.algorithm->profile[oven.step].state == UPPER
						|| oven.algorithm->profile[oven.step].state == POWERFUL)
					u8g2_DrawFilledEllipse(&u8g2, 7, 49, 7, 3,
					U8G2_DRAW_LOWER_LEFT | U8G2_DRAW_LOWER_RIGHT);
				if (relay.output) {
					u8g2_DrawVLine(&u8g2, 5, 54, 5);
					u8g2_DrawVLine(&u8g2, 9, 54, 5);
					u8g2_SetDrawColor(&u8g2, 2);
					u8g2_DrawHLine(&u8g2, 5, 56, 2);
					u8g2_DrawHLine(&u8g2, 9, 56, 2);
					u8g2_SetDrawColor(&u8g2, 1);
				}
				//RTC_TimeShow(stime);
				sprintf((char*) stime, "%02d:%02d:%02d %1d",
						hotplace[0].profile.time.Hours,
						hotplace[0].profile.time.Minutes,
						hotplace[0].profile.time.Seconds,
						TEMPLATE(GetState, HOTPLACE_STATE)(hotplace[0].profile.state));
				u8g2_DrawUTF8(&u8g2, 17, 16 * 1 - 2, (const char*) stime);
				sprintf((char*) stime, "%02d:%02d:%02d %1d",
						hotplace[1].profile.time.Hours,
						hotplace[1].profile.time.Minutes,
						hotplace[1].profile.time.Seconds,
						TEMPLATE(GetState, HOTPLACE_STATE)(hotplace[1].profile.state));
				u8g2_DrawUTF8(&u8g2, 17, 16 * 2 - 2, (const char*) stime);
				sprintf((char*) stime, "%02d:%02d:%02d %1d %3d00",
						hotplace[2].profile.time.Hours,
						hotplace[2].profile.time.Minutes,
						hotplace[2].profile.time.Seconds,
						TEMPLATE(GetState, HOTPLACE_STATE)(hotplace[2].profile.state),
						oven.profile.targetTemp);
				u8g2_DrawUTF8(&u8g2, 17, 16 * 3 - 2, (const char*) stime);
				sprintf((char*) stime, "%02d:%02d:%02d   %5d",
						oven.profile.time.Hours, oven.profile.time.Minutes,
						oven.profile.time.Seconds, (int) reg);
				u8g2_DrawUTF8(&u8g2, 17, 16 * 4 - 2, (const char*) stime);
				u8g2_SetDrawColor(&u8g2, 2);
				u8g2_DrawBox(&u8g2, 16, 1 + 16 * (pointer), 112, 15);

				//HAL_Delay(1200);
				//
				//u8g2_SetBitmapMode(&u8g2,1);
				//u8g2_DrawBitmap(&u8g2, 0, 0, bitmap_height, bitmap_width,  bitmap);

			} while (u8g2_NextPage(&u8g2));
		}
	}
}

// arg - Hotplace Number
static void HotplaceMenu(uint8_t arg) {
	pointer = 0;
	uint8_t temp = TEMPLATE(GetState, HOTPLACE_STATE)(hotplace[arg].profile.state), stime[30];
	RTC_TimeTypeDef time = hotplace[arg].profile.time;
	HotplaceMenu: ;
	pointer_max = 2;
	btns = 0;
	while (1) {
		BtnCheck();
		if (btns) {
			switch (btns) {
			case 0x0001: // short right
				if (temp == 0)
					temp = 6;
				if (time.Hours == 0 && time.Minutes == 0 && time.Seconds == 0)
					time.Minutes = 30;
				TEMPLATE(SetCustomProfile, Hotplace)(&hotplace[arg], arg, temp, &time);
				pointer = arg;
				return;
				break;
			case 0x0100: // short center
				switch (pointer) {
				case 0:
					temp = TemperatureMenuH(arg);
					break;
				case 1:
					time = TimeMenu(arg);
					break;
				case 2:
					if (temp == 0)
						temp = 6;
					if (time.Hours == 0 && time.Minutes == 0
							&& time.Seconds == 0)
						time.Minutes = 30;
					TEMPLATE(SetCustomProfile, Hotplace)(&hotplace[arg], arg, temp, &time);
					pointer = arg;
					return;
				default:
					break;
				}
				goto HotplaceMenu;
				break;
			case 0x0200: // long center
				pointer = arg;
				return; //back to MainMenu
			default:
				break;
			}
			btns = 0;
		}
		if (update) {
			update = 0;
			u8g2_FirstPage(&u8g2);
			do {

				u8g2_SetDrawColor(&u8g2, 1);

				u8g2_DrawFrame(&u8g2, 0, 1, 15, 15);
				if (arg == 0) {
					u8g2_DrawFilledEllipse(&u8g2, 14, 1, 7, 7,
					U8G2_DRAW_LOWER_LEFT);
				} else if (arg == 1) {
					u8g2_DrawFilledEllipse(&u8g2, 14, 14, 7, 7,
					U8G2_DRAW_UPPER_LEFT);
				} else if (arg == 2) {
					u8g2_DrawFilledEllipse(&u8g2, 0, 14, 7, 7,
					U8G2_DRAW_UPPER_RIGHT);
				}
				u8g2_DrawUTF8(&u8g2, 17, 14, "Меню");
				u8g2_DrawHLine(&u8g2, 0, 15, 128);
				sprintf((char*) stime, "Режим: %d", temp);
				u8g2_DrawUTF8(&u8g2, 1, 30, (const char*) stime);
				sprintf((char*) stime, "%02d:%02d:%02d", time.Hours,
						time.Minutes, time.Seconds);
				u8g2_DrawUTF8(&u8g2, 1, 46, (const char*) stime);
				u8g2_DrawUTF8(&u8g2, 1, 62, "OK");
				u8g2_SetDrawColor(&u8g2, 2);
				u8g2_DrawBox(&u8g2, 0, 1 + 16 * (pointer + 1), 128, 15);
			} while (u8g2_NextPage(&u8g2));
		}
	}
}

static void OvenMenu(uint8_t arg) {
	pointer = 0;
	uint16_t temp = oven.algorithm->profile[oven.step].targetTemp;
	uint8_t oState = TEMPLATE(GetState, OVEN_STATE)(oven.algorithm->profile[oven.step].state);
	RTC_TimeTypeDef time = oven.profile.time;
	uint8_t stime[30];
	char c[13];
	OvenMenu: ;
	pointer_max = 2;
	btns = 0;
	while (1) {
		BtnCheck();
		if (btns) {
			switch (btns) {
			case 0x0001: // short right
				if (oState == 0)
					oState = 4;
				if (temp == 0)
					temp = 300;
				if (time.Hours == 0 && time.Minutes == 0 && time.Seconds == 0)
					time.Minutes = 30;
				TEMPLATE(SetCustomProfile, Oven)(&oven, oState, temp, &time);
				pointer = arg;
				return;
				break;
			case 0x0100: // short center
				switch (pointer) {
				case 0:
					oState = StateMenuO(arg);
					break;
				case 1:
					time = TimeMenu(arg);
					break;
				case 2:
					temp = TemperatureMenuO(arg);
				default:
					break;
				}
				goto OvenMenu;
				break;
			case 0x0200: // long center
				pointer = arg;
				return; //back to MainMenu
			default:
				break;
			}
			btns = 0;
		}
		if (update) {
			update = 0;
			u8g2_FirstPage(&u8g2);
			do {

				u8g2_SetDrawColor(&u8g2, 1);

				u8g2_DrawUTF8(&u8g2, 0, 14, "Меню духовки");
				u8g2_DrawHLine(&u8g2, 0, 15, 128);
				switch (oState) {
				case 1:
					sprintf((char*) c, "Cлабо");
					break;
				case 2:
					sprintf((char*) c, "Низ");
					break;
				case 3:
					sprintf((char*) c, "Верх");
					break;
				case 4:
					sprintf((char*) c, "Сильно");
					break;
				default:
					sprintf((char*) c, "Выкл");
					break;
				}
				sprintf((char*) stime, "Режим:%s", c);
				u8g2_DrawUTF8(&u8g2, 1, 30, (const char*) stime);
				sprintf((char*) stime, "%02d:%02d:%02d", time.Hours,
						time.Minutes, time.Seconds);
				u8g2_DrawUTF8(&u8g2, 1, 46, (const char*) stime);
				sprintf((char*) stime, "Т = %d", temp);
				u8g2_DrawUTF8(&u8g2, 1, 62, (const char*) stime);
				u8g2_SetDrawColor(&u8g2, 2);
				u8g2_DrawBox(&u8g2, 0, 1 + 16 * (pointer + 1), 128, 15);
			} while (u8g2_NextPage(&u8g2));
		}
	}
}

// arg - Hotplace Number
// return temperature
static uint8_t TemperatureMenuH(uint8_t arg) {
	pointer_max = 6;
	pointer = 0;
	uint16_t temp;
	uint8_t poin[14];
	btns = 0;
	while (1) {
		BtnCheck();
		if (btns) {
			switch (btns) {
			case 0x0100: // short center -- OK
				temp = pointer;
				pointer = 0;
				return temp;
			case 0x0200: // long center -- cancel
				pointer = 0;
				return 0;
			default:
				break;
			}
			btns = 0;
		}
		if (update) {
			update = 0;
			do {
				u8g2_SetDrawColor(&u8g2, 0);
				u8g2_DrawBox(&u8g2, 0, 17, 128, 15);
				u8g2_SetDrawColor(&u8g2, 1);
				sprintf((char*) poin, "Режим: %d", pointer);
				u8g2_DrawUTF8(&u8g2, 1, 30, (const char*) poin);
				u8g2_SetDrawColor(&u8g2, 2);
				u8g2_DrawHLine(&u8g2, 0, 31, 128);
			} while (u8g2_NextPage(&u8g2));
		}
	}
}
static uint16_t TemperatureMenuO(uint8_t arg) {
	pointer_max = 60;
	pointer = 0;
	uint16_t temp;
	uint8_t poin[6];
	btns = 0;
	while (1) {
		BtnCheck();
		if (btns) {
			switch (btns) {
			case 0x0100: // short center -- OK
				temp = pointer * 5;
				pointer = 2;
				return temp;
			case 0x0200: // long center -- cancel
				pointer = 2;
				return 0;
			default:
				break;
			}
			btns = 0;
		}
		if (update) {
			update = 0;
			do {
				u8g2_SetDrawColor(&u8g2, 0);
				u8g2_DrawBox(&u8g2, 0, 49, 128, 15);
				u8g2_SetDrawColor(&u8g2, 1);
				sprintf((char*) poin, "T = %d", pointer * 5);
				u8g2_DrawUTF8(&u8g2, 1, 62, (const char*) poin);
				u8g2_SetDrawColor(&u8g2, 2);
				u8g2_DrawHLine(&u8g2, 0, 63, 128);
			} while (u8g2_NextPage(&u8g2));
		}
	}
}
// arg - Hotplace Number
static RTC_TimeTypeDef TimeMenu(uint8_t arg) {
	pointer = 0;
	RTC_TimeTypeDef tempTime = { };
	uint8_t stime[9];
	WE_CHANGE_TIME state = WE_CHANGE_HOURS;
	pointer_max = 23;
	btns = 0;
	while (1) {
		BtnCheck();
		if (btns) {
			switch (btns) {
			case 0x0001: // short right -- back
				switch (state) {
				case WE_CHANGE_HOURS:
					pointer = 1;
					memset(&tempTime, 0, sizeof(RTC_TimeTypeDef));
					return tempTime;
				case WE_CHANGE_MINUTES:
					state = WE_CHANGE_HOURS;
					pointer_max = 23;
					tempTime.Minutes = pointer;
					pointer = tempTime.Hours;
					break;
				case WE_CHANGE_SECONDS:
					state = WE_CHANGE_MINUTES;
					tempTime.Seconds = pointer;
					pointer = tempTime.Minutes;
					break;
				default:
					break;
				}
				break;
			case 0x0100: // short center -- OK
				switch (state) {
				case WE_CHANGE_HOURS:
					state = WE_CHANGE_MINUTES;
					pointer_max = 59;
					tempTime.Hours = pointer;
					pointer = tempTime.Minutes;
					break;
				case WE_CHANGE_MINUTES:
					state = WE_CHANGE_SECONDS;
					tempTime.Minutes = pointer;
					pointer = tempTime.Seconds;
					break;
				case WE_CHANGE_SECONDS:
					tempTime.Seconds = pointer;
					pointer = 1;
					return tempTime;
				default:
					break;
				}
				break;
			case 0x0200: // long center -- cancel
				pointer = 1;
				memset(&tempTime, 0, sizeof(RTC_TimeTypeDef));
				return tempTime;
			default:
				break;
			}
			btns = 0;
		}
		if (update) {
			update = 0;
			do {
				u8g2_SetDrawColor(&u8g2, 0);
				u8g2_DrawBox(&u8g2, 0, 33, 128, 15);
				u8g2_SetDrawColor(&u8g2, 1);
				switch (state) {
				case WE_CHANGE_HOURS:
					sprintf((char*) stime, "%02d:%02d:%02d", pointer,
							tempTime.Minutes, tempTime.Seconds);
					break;
				case WE_CHANGE_MINUTES:
					sprintf((char*) stime, "%02d:%02d:%02d", tempTime.Hours,
							pointer, tempTime.Seconds);
					break;
				case WE_CHANGE_SECONDS:
					sprintf((char*) stime, "%02d:%02d:%02d", tempTime.Hours,
							tempTime.Minutes, pointer);
					break;
				default:
					break;
				}
				u8g2_DrawUTF8(&u8g2, 1, 46, (const char*) stime);
				u8g2_SetDrawColor(&u8g2, 2);
				u8g2_DrawBox(&u8g2, 18 * (uint8_t) state, 37, 13, 11);
			} while (u8g2_NextPage(&u8g2));
		}
	}
}

static uint16_t StateMenuO(uint8_t arg) {
	pointer = 0;
	uint16_t oState;
	uint8_t c[13], stime[24];
	pointer_max = 4;
	btns = 0;
	while (1) {
		BtnCheck();
		if (btns) {
			switch (btns) {
			case 0x0100: // short center -- OK
				oState = pointer;
				pointer = 0;
				return oState;
			case 0x0200: // long center -- cancel
				pointer = 0;
				return 0;
			default:
				break;
			}
			btns = 0;
		}
		if (update) {
			update = 0;
			do {
				u8g2_SetDrawColor(&u8g2, 0);
				u8g2_DrawBox(&u8g2, 0, 17, 128, 15);
				u8g2_SetDrawColor(&u8g2, 1);
				switch (pointer) {
				case 1:
					sprintf((char*) c, "Слабо");
					break;
				case 2:
					sprintf((char*) c, "Низ");
					break;
				case 3:
					sprintf((char*) c, "Верх");
					break;
				case 4:
					sprintf((char*) c, "Сильно");
					break;
				default:
					sprintf((char*) c, "Выкл");
					break;
				}
				sprintf((char*) stime, "Режим:%s", c);
				u8g2_DrawUTF8(&u8g2, 1, 30, (const char*) stime);
				u8g2_SetDrawColor(&u8g2, 2);
				u8g2_DrawHLine(&u8g2, 0, 31, 128);
			} while (u8g2_NextPage(&u8g2));
		}
	}
}

//static void CustomAlgorithm(uint8_t arg) {
//	pointer_max = 11;
//	pointer = 0;
//	btns = 0;
//	while (1) {
//		BtnCheck();
//		if (btns) {
//			switch (btns) {
//			case 0x0001: // short right
//				break;
//			case 0x0002: // long right
//				break;
//			case 0x0004: // short up
//				break;
//			case 0x0008: // long up
//				break;
//			case 0x0010: // short down
//				break;
//			case 0x0020: // long down
//				break;
//			case 0x0040: // short left
//				break;
//			case 0x0080: // long left
//				break;
//			case 0x0100: // short center
//				break;
//			case 0x0200: // long center
//				return;
//			default:
//				break;
//			}
//			btns = 0;
//		}
//		if (update) {
//			update = 0;
//			u8g2_FirstPage(&u8g2);
//			do {
//
//			} while (u8g2_NextPage(&u8g2));
//		}
//	}
//}

// TEMPLATE MENU
//void TEMPLATEMENU(uint8_t arg) {
//	pointer_max = ;
//	pointer = 0;
//  btns = 0;
//	while (1) {
//		BtnCheck();
//		if (btns) {
//			switch (btns) {
//			case 0x0001: // short right
//				break;
//			case 0x0002: // long right
//				break;
//			case 0x0004: // short up
//				break;
//			case 0x0008: // long up
//				break;
//			case 0x0010: // short down
//				break;
//			case 0x0020: // long down
//				break;
//			case 0x0040: // short left
//				break;
//			case 0x0080: // long left
//				break;
//			case 0x0100: // short center
//				break;
//			case 0x0200: // long center
//				break;
//			default:
//				break;
//			}
//			btns = 0;
//		}
//		if (update) {
//			update = 0;
//			u8g2_FirstPage(&u8g2);
//			do {
//
//			} while (u8g2_NextPage(&u8g2));
//		}
//	}
//}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
/*****END OF FILE****/