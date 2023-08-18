/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "rtc.h"
#include "spi.h"
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "../App/ILI9341.h"
#include "../App/W25qxx.h"
#include "../App/key.h"
#include "../App/tab_temp.h"
#include "../App/power_ctrl.h"
#include "../lvgl/lvgl.h"
#include "../App/screen.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define DEBOUNCE_SW 	50

uint16_t adcBuffer[2]; 					// Buffer ADC conversion
uint8_t buf_tft[320 * 10 * 2];
//static lv_obj_t * Tela_Principal;
//static lv_obj_t * Text_Header;

uint32_t ADC_iron = 0, ADC_air = 0;
uint32_t timer_led1 = 0, timer_led2 = 0, timer_led3 = 0, timer_led4 = 0;
uint32_t timer_lvgl = 0;

GPIO_PinState pin_sw_air;
uint8_t sw_air_low = 0;
uint8_t sw_air_high = 0;
volatile uint8_t sw_air = 0;

GPIO_PinState pin_sw_iron;
uint8_t sw_iron_low = 0;
uint8_t sw_iron_high = 0;
volatile uint8_t sw_iron = 0;

volatile uint32_t timer_key = 0;

double adc_ch8, adc_ch9;

volatile uint32_t enc1_cnt=0, enc1_dir=0, enc1_btn=0;
volatile uint32_t enc2_cnt=0, enc2_dir=0, enc2_btn=0;
volatile uint32_t enc3_cnt=0, enc3_dir=0, enc3_btn=0;

uint32_t target_iron, target_air, target_speed;
uint32_t flag_iron, flag_air;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_USB_DEVICE_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM9_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
  __HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_1, 0);		// PWM_CH1 = 0 IRON
  // TIM4 Dimmer

  // Start ADC
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcBuffer, 2);	// Start ADC in DMA

  // Init Flash
  W25qxx_Init();

  // Apaga LEDS
  // LED-1 - HEATER GUN
  // LED-2 - REPOUSO
  // LED-3 - OPERATE
  // LED-4 - HEATER IRON
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);

  Evt_InitQueue();
  KeyboardInit(0x01);

  target_iron = 0;
  target_air = 0;
  target_speed = 0;
  flag_iron = 0;
  flag_air = 0;

  ILI9341_Init();
  ILI9341_Set_Address(0, 0, ILI9341_SCREEN_WIDTH-1, ILI9341_SCREEN_HEIGHT-1);
  ILI9341_Set_Rotation(1);
  ILI9341_Fill_Screen(0x0000);

  AIRTEMP_TABLE_Interpolation();
  IRON_TABLE_Interpolation();

  lv_init();

  static lv_disp_draw_buf_t draw_buf;
  static lv_color_t buf1[(320 * 10)];                        		// Declare a buffer for 1/10 screen size
  static lv_color_t buf2[(320 * 10)];                        		// Declare a buffer for 1/10 screen size
  lv_disp_draw_buf_init(&draw_buf, buf1, buf2, (320 * 10) );		// Initialize the display buffer.

  static lv_disp_drv_t disp_drv;        // Descriptor of a display driver
  lv_disp_drv_init(&disp_drv);          // Basic initialization

  disp_drv.flush_cb = ILI9341_Flush;    // Set your driver function
  disp_drv.draw_buf = &draw_buf;        // Assign the buffer to the display
  disp_drv.hor_res = 320;   			// Set the horizontal resolution of the display
  disp_drv.ver_res = 240;   			// Set the vertical resolution of the display
  disp_drv.rotated    = LV_DISP_ROT_90;
  disp_drv.sw_rotate  = 1;
  lv_disp_drv_register(&disp_drv);      // Finally register the driver

/*  Tela_Principal = lv_obj_create(NULL);
  lv_obj_clear_flag(Tela_Principal, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_style_bg_color(Tela_Principal, lv_color_hex(0x000000), 0);
  lv_obj_set_style_bg_grad_color(Tela_Principal, lv_color_hex(0x000000), 0);
  //
  Text_Header = lv_label_create(Tela_Principal);
  lv_obj_set_style_text_font(Text_Header, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_text_color(Text_Header, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_text_opa(Text_Header, LV_OPA_COVER, LV_PART_MAIN | LV_STATE_DEFAULT);
  lv_obj_set_style_text_letter_space(Text_Header, 1, 0);
  lv_obj_set_style_text_line_space(Text_Header, 1, 0);

  lv_label_set_long_mode(Text_Header, LV_LABEL_LONG_WRAP);               // Break the long lines
  lv_label_set_recolor(Text_Header, true);                               // Enable re-coloring by commands in the text
  lv_label_set_text(Text_Header, "TFT LVGL STM32-F411 240x320");
  lv_obj_center(Text_Header);

  lv_scr_load(Tela_Principal);
*/
  //screen_main();
  screen_debug();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(HAL_GetTick() - timer_lvgl > 5) {
		  timer_lvgl = HAL_GetTick();
		  lv_timer_handler();
	  }

	  // Encoder 1
	  enc1_cnt = htim1.Instance->CNT;
	  enc1_dir = !(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim1));

	  // Encoder 2
	  enc2_cnt = htim2.Instance->CNT;
	  enc2_dir = !(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2));

	  // Encoder 3
	  enc3_cnt = htim3.Instance->CNT;
	  enc3_dir = !(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3));

	  // Buttons Encoders
	  KeyboardEvent();

	  // Read ADC
	  adc_ch8 = ADC_iron * ((float)3300.0/4095.0);
	  adc_ch9 = ADC_air * ((float)3300.0/4095.0);
	  //ADC_MeasurementCorrection();

	  if(HAL_GetTick() - timer_led1 > 100) {
		  timer_led1 = HAL_GetTick();
		  HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	  }
	  if(HAL_GetTick() - timer_led2 > 150) {
		  timer_led2 = HAL_GetTick();
		  HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
	  }
	  if(HAL_GetTick() - timer_led3 > 200) {
		  timer_led3 = HAL_GetTick();
		  HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
	  }
	  if(HAL_GetTick() - timer_led4 > 250) {
		  timer_led4 = HAL_GetTick();
		  HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
	  }

	  // Controle de Temperatura
//	  control_temperature_iron();
//	  control_temperature_air();
//	  control_speed_air();
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/* USER CODE BEGIN 4 */
void debounce_input(void)
{
	// Debounce SW_AIR PIN
	pin_sw_air = HAL_GPIO_ReadPin(SW_AIR_GPIO_Port, SW_AIR_Pin);
	if(pin_sw_air == 0) {
		sw_air_low++;
		sw_air_high = 0;
		if(sw_air_low >= DEBOUNCE_SW) {
			sw_air_low = DEBOUNCE_SW + 1;
			sw_air = 0;
		}
	}
	else {
		sw_air_high++;
		sw_air_low = 0;
		if(sw_air_high >= DEBOUNCE_SW) {
			sw_air_high = DEBOUNCE_SW + 1;
			sw_air = 1;
		}
	}
	//
	// Debounce SW_IRON PIN
	pin_sw_iron = HAL_GPIO_ReadPin(SW_AIR_GPIO_Port, SW_AIR_Pin);
	if(pin_sw_iron == 0) {
		sw_iron_low++;
		sw_iron_high = 0;
		if(sw_iron_low >= DEBOUNCE_SW) {
			sw_iron_low = DEBOUNCE_SW + 1;
			sw_iron = 0;
		}
	}
	else {
		sw_iron_high++;
		sw_iron_low = 0;
		if(sw_iron_high >= DEBOUNCE_SW) {
			sw_iron_high = DEBOUNCE_SW + 1;
			sw_iron = 1;
		}
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	ADC_iron = (uint32_t)adcBuffer[0];
	ADC_air  = (uint32_t)adcBuffer[1];
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM11 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM11) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM11) {
	  debounce_input();
	  lv_tick_inc(1);
	  timer_key++;
	  if(timer_key >= PUSHBTN_TMR_PERIOD) {
		  timer_key = 0;
		  Key_Read();
	  }
  }
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
