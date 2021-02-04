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
#include "u8g2/u8g2.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
TIM_Encoder_InitTypeDef sConfig = { 0 };
//unsigned char stAlarm[5] = "ABC";
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
uint32_t time_irq[5];
uint8_t activityOLD = 255, activity = 1, pointer = 0, flag_irq[5] = { 0 },
		update = 255;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM3_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
uint8_t U8x8Stm32GPIOAndDelay(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int,
		void *arg_ptr);
uint8_t U8x8ByteSTM32HWSPI(u8x8_t *u8x8, uint8_t msg, uint8_t arg_int,
		void *arg_ptr);
void MainActivity(void);
void OrderActivity(void);
void TIM1_ReConfig(uint32_t period, uint32_t cnt);
_Bool EventFlag(uint32_t *time_irq, uint8_t *flag_irq, IRQn_Type exti);
void ScreenUpdate(uint8_t button);
static void RTC_TimeShow(uint8_t *showtime);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define bitmap_width 64
#define bitmap_height 16

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_SPI2_Init();
  MX_TIM3_Init();
  MX_RTC_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	u8g2_Setup_st7920_s_128x64_f(&u8g2, U8G2_R0, U8x8ByteSTM32HWSPI,
			U8x8Stm32GPIOAndDelay);
	u8g2_InitDisplay(&u8g2); // send init sequence to the display, display is in sleep mode after this,
	u8g2_SetPowerSave(&u8g2, 0); // wake up display
	u8g2_SetFont(&u8g2, u8g2_font_6x12_t_cyrillic);
	u8g2_SetFontDirection(&u8g2, 0);

	ScreenUpdate(255);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		if (EventFlag(&time_irq[0], &flag_irq[0], EXTI0_IRQn)) {
			activity = 0;
			goto gotoUpdate;
		} else if (EventFlag(&time_irq[1], &flag_irq[1], EXTI1_IRQn)) {
			activity = 1;
			goto gotoUpdate;
		} else if (EventFlag(&time_irq[2], &flag_irq[2], EXTI2_IRQn)) {
			goto gotoUpdate;
		} else if (EventFlag(&time_irq[3], &flag_irq[3], EXTI3_IRQn)) {
			goto gotoUpdate;
		} else if (EventFlag(&time_irq[4], &flag_irq[4], EXTI4_IRQn)) {
			goto gotoUpdate;
		}
		if (pointer != __HAL_TIM_GET_COUNTER(&htim1) / 2) {
			pointer = __HAL_TIM_GET_COUNTER(&htim1) / 2;
			goto gotoUpdate;
		} else
			if (update != 255) {
			gotoUpdate: ScreenUpdate(update);
			update = 255;
		}
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
	RTC_AlarmTypeDef sAlarm = { 0 };
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

	sAlarm.Alarm = RTC_ALARM_A;
	sAlarm.AlarmTime.Hours = 0;
	sAlarm.AlarmTime.Minutes = 0;
	sAlarm.AlarmTime.Seconds = 30;

	if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK) {
		Error_Handler();
	}

	HAL_NVIC_SetPriority(RTC_Alarm_IRQn, 0x0F, 0);
	HAL_NVIC_EnableIRQ(RTC_Alarm_IRQn);
  /* USER CODE END RTC_Init 2 */

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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 5;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 5;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
  HAL_TIM_Encoder_Start_IT(htim1, TIM_CHANNEL_1);
  /* USER CODE END TIM1_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7199;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

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
		HAL_Delay(arg_int);
		break;
	case U8X8_MSG_GPIO_CS:
		/* Insert codes for SS pin control */
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, arg_int);
		break;
	case U8X8_MSG_GPIO_DC:
		/* Insert codes for DC pin control */
		//HAL_GPIO_WritePin(OLED_DC_GPIO_Port, OLED_DC_Pin, arg_int);
		break;
	case U8X8_MSG_GPIO_RESET:
		/* Insert codes for RST pin control */
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, arg_int);
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
		HAL_Delay(1);
		break;
	case U8X8_MSG_BYTE_END_TRANSFER:
		HAL_Delay(1);
		/* Insert codes to end SPI transmission */
		u8x8_gpio_SetCS(u8x8, u8x8->display_info->chip_disable_level);
		break;
	default:
		return 0;
	}
	return 1;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	switch (GPIO_Pin) {
	case GPIO_PIN_0:
		HAL_NVIC_DisableIRQ(EXTI0_IRQn);
		flag_irq[0]++;
		time_irq[0] = HAL_GetTick();
		update = 0;
		break;
	case GPIO_PIN_1:
		HAL_NVIC_DisableIRQ(EXTI1_IRQn);
		flag_irq[1]++;
		time_irq[1] = HAL_GetTick();
		update = 1;
		break;
	case GPIO_PIN_2:
		HAL_NVIC_DisableIRQ(EXTI2_IRQn);
		flag_irq[2]++;
		time_irq[2] = HAL_GetTick();
		update = 2;
		break;
	case GPIO_PIN_3:
		HAL_NVIC_DisableIRQ(EXTI3_IRQn);
		flag_irq[3]++;
		time_irq[3] = HAL_GetTick();
		update = 3;
		break;
	case GPIO_PIN_4:
		HAL_NVIC_DisableIRQ(EXTI4_IRQn);
		flag_irq[4]++;
		time_irq[4] = HAL_GetTick();
		update = 4;
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM1) {
		//update = 1;
	}
}

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc) {
	//sprintf((char*) stAlarm, "ZAZ");
}

void MainActivity(void) {
	u8g2_FirstPage(&u8g2);
	do {
		u8g2_ClearBuffer(&u8g2);
		u8g2_SetDrawColor(&u8g2, 1);
		u8g2_DrawFrame(&u8g2, 0, 1, 15, 15);
		u8g2_DrawFilledEllipse(&u8g2, 14, 1, 7, 7, U8G2_DRAW_LOWER_LEFT);
		u8g2_DrawFrame(&u8g2, 0, 17, 15, 15);
		u8g2_DrawFilledEllipse(&u8g2, 14, 31, 7, 7, U8G2_DRAW_UPPER_LEFT);
		u8g2_DrawFrame(&u8g2, 0, 33, 15, 15);
		u8g2_DrawFilledEllipse(&u8g2, 0, 47, 7, 7, U8G2_DRAW_UPPER_RIGHT);
		u8g2_DrawFrame(&u8g2, 0, 49, 15, 15);
		//if (lowerHeat)
		u8g2_DrawFilledEllipse(&u8g2, 7, 63, 7, 3,
		U8G2_DRAW_UPPER_LEFT | U8G2_DRAW_UPPER_RIGHT);
		//if (upperHeat)
		u8g2_DrawFilledEllipse(&u8g2, 7, 49, 7, 3,
		U8G2_DRAW_LOWER_LEFT | U8G2_DRAW_LOWER_RIGHT);
		//if (heatOn)
		//{
		u8g2_DrawVLine(&u8g2, 5, 54, 5);
		u8g2_DrawVLine(&u8g2, 9, 54, 5);
		u8g2_SetDrawColor(&u8g2, 2);
		u8g2_DrawHLine(&u8g2, 5, 56, 2);
		u8g2_DrawHLine(&u8g2, 9, 56, 2);
		u8g2_SetDrawColor(&u8g2, 1);
		//}
		uint8_t stime[9];
		RTC_TimeShow(stime);
		u8g2_DrawUTF8(&u8g2, 17, 16 * 1 - 2, stime);
		//u8g2_DrawUTF8(&u8g2, 17, 16 * 2 - 2, stAlarm);
		//u8g2_DrawUTF8(&u8g2, 17, 16 * 3 - 2, time);
		//u8g2_DrawUTF8(&u8g2, 17, 16 * 4 - 2, time);
		u8g2_SetDrawColor(&u8g2, 2);
		u8g2_DrawBox(&u8g2, 16, 1 + 16 * (pointer), 112, 15);

		//HAL_Delay(1200);
		//u8g2_ClearBuffer(&u8g2);
		//u8g2_SetBitmapMode(&u8g2,1);
		//u8g2_DrawBitmap(&u8g2, 0, 0, bitmap_height, bitmap_width,  bitmap);

	} while (u8g2_NextPage(&u8g2));
}

void OrderActivity(void) {
	u8g2_FirstPage(&u8g2);
	do {
		u8g2_ClearBuffer(&u8g2);
		u8g2_SetDrawColor(&u8g2, 1);

		u8g2_DrawFrame(&u8g2, 0, 1, 15, 15);
		u8g2_DrawFilledEllipse(&u8g2, 14, 1, 7, 7, U8G2_DRAW_LOWER_LEFT);

		u8g2_DrawUTF8(&u8g2, 17, 14, "Меню");
		u8g2_DrawHLine(&u8g2, 0, 15, 128);
		u8g2_DrawUTF8(&u8g2, 1, 30, "Режим: 3");
		//u8g2_DrawUTF8(&u8g2, 1, 46, "Время: 00:12:49");
		uint8_t stime[9];
		RTC_TimeShow(stime);
		u8g2_DrawUTF8(&u8g2, 1, 46, stime);
		u8g2_DrawUTF8(&u8g2, 1, 62, "3 pointer");
		u8g2_SetDrawColor(&u8g2, 2);
		u8g2_DrawBox(&u8g2, 0, 1 + 16 * (pointer + 1), 128, 15);
	} while (u8g2_NextPage(&u8g2));
}

void ScreenUpdate(uint8_t button) {
	if (activityOLD != activity) {
		activityOLD = activity;
		switch (activity) {
		case 0:
			TIM1_ReConfig(7, 1);
			break;
		case 1:
			TIM1_ReConfig(5, 1);
			break;
		default:
			__NOP();
		}
	}
	switch (activity) {
	case 0:
		MainActivity();
		break;
	case 1:
		OrderActivity();
		break;
	default:
		__NOP();
	}
}

void TIM1_ReConfig(uint32_t period, uint32_t cnt) {
	if (HAL_TIM_Encoder_DeInit(&htim1) != HAL_OK) {
		Error_Handler();
	}
	htim1.Init.Period = period;
	if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	htim1.Instance->CNT = cnt;
}

_Bool EventFlag(uint32_t *time_irq, uint8_t *flag_irq, IRQn_Type exti) {
	if ((*flag_irq == 1)
			&& (HAL_GetTick() - *time_irq) > 100) {
		__HAL_GPIO_EXTI_CLEAR_IT(exti);
		NVIC_ClearPendingIRQ(exti);
		HAL_NVIC_EnableIRQ(exti);
//		if (*flag_irq == 1) {
//			(*flag_irq)++;
//			return (_Bool) 0;
//		} else {
			*flag_irq = 0;
			return (_Bool) 1;
//		}
	}
	return (_Bool) 0;
}

static void RTC_TimeShow(uint8_t *showtime) {
	RTC_DateTypeDef sdatestructureget;
	RTC_TimeTypeDef stimestructureget;

	/* Get the RTC current Time */
	HAL_RTC_GetTime(&hrtc, &stimestructureget, RTC_FORMAT_BIN);
	/* Get the RTC current Date */
	HAL_RTC_GetDate(&hrtc, &sdatestructureget, RTC_FORMAT_BIN);
	/* Display time Format : hh:mm:ss */
	sprintf((char*) showtime, "%02d:%02d:%02d", stimestructureget.Hours,
			stimestructureget.Minutes, stimestructureget.Seconds);
}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
