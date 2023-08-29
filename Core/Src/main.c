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
#include "string.h"
#include "stdio.h"
#include "../App/ILI9341.h"
#include "../App/ILI9488.h"
#include "../App/MAX6675.h"
#include "../App/W25qxx.h"
#include "../App/key.h"
#include "../App/tab_temp.h"
#include "../App/power_ctrl.h"
#include "../lvgl/lvgl.h"
#include "../App/screen.h"
#include "../App/log_usb.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define DEBOUNCE_SW 		50
#define NUM_DIMMERS 		2
#define MAX_DIMMER_VALUE	832
#define MAX_TEMPERATURE		525

uint16_t adcBuffer[4]; 					// Buffer ADC conversion
uint8_t buf_tft[ILI9341_SCREEN_WIDTH * 10 * 2];
//uint8_t buf_tft[ILI9488_SCREEN_WIDTH * 10 * 3];

uint32_t ADC_iron = 0, ADC_air = 0, ADC_temp = 0, ADC_vref = 0;
uint32_t flt_adc_8[8] ={0};
uint32_t flt_adc_9[8] ={0};
uint32_t flt_adc_t[8] ={0};
uint32_t flt_adc_v[8] ={0};
uint32_t idx_flt = 0;
uint32_t flt_flag = 0;
uint32_t timer_led1 = 0, timer_led2 = 0, timer_led3 = 0, timer_led4 = 0;
uint32_t timer_lvgl = 0, timer_max = 0, timer_rtc = 0, timer_debug = 0;

GPIO_PinState pin_sw_air;
uint8_t sw_air_low = 0;
uint8_t sw_air_high = 0;
volatile uint8_t sw_air = 0;

GPIO_PinState pin_sw_iron;
uint8_t sw_iron_low = 0;
uint8_t sw_iron_high = 0;
volatile uint8_t sw_iron = 0;

volatile uint32_t timer_key = 0;

float adc_ch8, adc_ch9;

volatile uint32_t enc1_cnt=0, enc1_dir=0, enc1_btn=0;
volatile uint32_t enc2_cnt=0, enc2_dir=0, enc2_btn=0;
volatile uint32_t enc3_cnt=0, enc3_dir=0, enc3_btn=0;

uint32_t target_speed;
float target_iron, target_air;
uint32_t flag_iron, flag_air;

uint16_t pwm_iron = 0;
float temperature_air_K, temperature_K, temp_iron_K, temp_air_K;

extern _Bool TCF_IRON, TCF_AIR;
char str_termopar_iron[32] = {0};
char str_termopar_air[32] = {0};
float vdda = 0; // Result of VDDA calculation
float vref = 0; // Result of vref calculation
float temp_stm, ta, tb; // transfer function using calibration data

RTC_TimeTypeDef RTC_Time = {0};
RTC_DateTypeDef RTC_Date = {0};

int NumActiveChannels = NUM_DIMMERS;
volatile bool zero_cross = 0;
volatile int NumHandled = 0;
volatile bool isHandled[NUM_DIMMERS] = { 0, 0 };
int State [NUM_DIMMERS] = { 1, 1 };
bool pLampState[2]={ false, false };
volatile int dimmer_Counter[NUM_DIMMERS] = { 0, 0 };
int dimmer_value[NUM_DIMMERS] = { 0, 0 };

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
void filter_adc(void);
void calculate_calibration(void);
uint32_t map_uint32(uint32_t var, uint32_t x_min, uint32_t x_max, uint32_t y_min, uint32_t y_max);
int map_dimmer(int var, int x_min, int x_max, int y_min, int y_max);
void Zero_Crossing_Int(void);
void dimTimerISR(void);
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
  MX_TIM9_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim5);
  HAL_TIM_Base_Start_IT(&htim10);
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  // PWM
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
  pwm_iron = 0;
  __HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_1, pwm_iron);		// PWM_CH1 = 0 IRON
  // Start ADC
  idx_flt = 0;
  flt_flag = 0;
  calculate_calibration();
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adcBuffer, 4);	// Start ADC in DMA

  // Init Flash
  W25qxx_Init();

  // Apaga LEDS
  // LED-1 - PWM GUN
  // LED-2 - REPOUSO
  // LED-3 - OPERATE
  // LED-4 - PWM IRON
  HAL_GPIO_WritePin(DIMMER_1_GPIO_Port, DIMMER_1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(DIMMER_2_GPIO_Port, DIMMER_2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(RELAY_GPIO_Port,RELAY_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);

  Evt_InitQueue();
  KeyboardInit(0x01);

  target_iron = 0.0f;
  target_air = 0.0f;
  target_speed = 0;
  flag_iron = 0;
  flag_air = 0;

  ILI9341_Init();
  ILI9341_Set_Address(0, 0, ILI9341_SCREEN_WIDTH-1, ILI9341_SCREEN_HEIGHT-1);
  ILI9341_Set_Rotation(1);
  ILI9341_Fill_Screen(0x0000);

  //ILI9488_Init();
  //ILI9488_Set_Address(0, 0, ILI9488_SCREEN_WIDTH-1, ILI9488_SCREEN_HEIGHT-1);
  //ILI9488_Set_Rotation(1);
  //ILI9488_Fill_Screen(0x0000);

  //IRON_TABLE_Interpolation();
  //AIR_TABLE_Interpolation();

  lv_init();

  static lv_disp_draw_buf_t draw_buf;
  static lv_color_t buf1[(ILI9341_SCREEN_WIDTH * 10)];                        		// Declare a buffer for 1/10 screen size
  static lv_color_t buf2[(ILI9341_SCREEN_WIDTH * 10)];                        		// Declare a buffer for 1/10 screen size
  lv_disp_draw_buf_init(&draw_buf, buf1, buf2, (ILI9341_SCREEN_WIDTH * 10) );		// Initialize the display buffer.

  static lv_disp_drv_t disp_drv;        // Descriptor of a display driver
  lv_disp_drv_init(&disp_drv);          // Basic initialization

  disp_drv.flush_cb = ILI9341_Flush;    // Set your driver function
  disp_drv.hor_res = ILI9341_SCREEN_WIDTH;   			// Set the horizontal resolution of the display
  disp_drv.ver_res = ILI9341_SCREEN_HEIGHT;   			// Set the vertical resolution of the display

  //disp_drv.flush_cb = ILI9488_Flush;	//Set your driver function
  //disp_drv.hor_res = ILI9488_SCREEN_WIDTH;   			// Set the horizontal resolution of the display
  //disp_drv.ver_res = ILI9488_SCREEN_HEIGHT;   			// Set the vertical resolution of the display

  disp_drv.draw_buf   = &draw_buf;        // Assign the buffer to the display
  disp_drv.rotated    = LV_DISP_ROT_90;
  disp_drv.sw_rotate  = 1;
  lv_disp_drv_register(&disp_drv);      // Finally register the driver

  screen_main();
  screen_debug();
  load_screen(0);

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

	  // Debug Serial Text
	  if(HAL_GetTick() - timer_debug > 1000) {
	  		timer_debug = HAL_GetTick();
	  		Log_temp_iron();
	  }

	  if(HAL_GetTick() - timer_max > 250) {
		  timer_max = HAL_GetTick();
		  // Temperatura Iron
		  temp_iron_K = Max6675_Read_Temp(0);
		  if(TCF_IRON == 0) {
			  temperature_K = temp_iron_K;
			  sprintf(str_termopar_iron, "Temp(K): %0.2f°C", temperature_K);
		  }
		  else {
			  sprintf(str_termopar_iron, "Temp(K): Not Connected");
		  }
		  // Temperatura Air
		  temp_air_K = Max6675_Read_Temp(1);
		  if(TCF_AIR == 0) {
			  temperature_air_K = temp_air_K;
			  sprintf(str_termopar_air, "Temp(K): %0.2f°C", temperature_air_K);
		  }
		  else {
			  sprintf(str_termopar_air, "Temp(K): Not Connected");
		  }
	  }

	  if(HAL_GetTick() - timer_rtc > 1000) {
		  timer_rtc = HAL_GetTick();
		  HAL_RTC_GetTime(&hrtc, &RTC_Time, RTC_FORMAT_BIN);
		  HAL_RTC_GetDate(&hrtc, &RTC_Date, RTC_FORMAT_BIN);
	  }

	  // Encoder 1
	  enc1_cnt = htim1.Instance->CNT >> 2;
	  enc1_dir = !(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim1));

	  if(enc1_cnt != target_speed) {
		  target_speed = map_uint32(enc1_cnt, 0, 1023, 0, 100);
		  dimmer_value[1] = map_dimmer(enc1_cnt, 0, 1023, 0, MAX_DIMMER_VALUE);
	  }

	  // Encoder 2
	  enc2_cnt = htim3.Instance->CNT >> 2;
	  enc2_dir = !(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3));

	  if(enc2_cnt != FLOAT_TO_INT(target_air)) {
		  target_air = (float)map_uint32(enc2_cnt, 0, 1023, 0, MAX_TEMPERATURE);
		  dimmer_value[0] = map_dimmer(enc2_cnt, 0, 1023, 0, MAX_DIMMER_VALUE);
	  }

	  // Encoder 3
	  enc3_cnt = htim2.Instance->CNT >> 2;
	  enc3_dir = !(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2));

	  if(enc3_cnt != FLOAT_TO_INT(target_iron)) {
	  	  target_iron = (float)map_uint32(enc3_cnt, 0, 4095, 0, MAX_TEMPERATURE);
	  }

	  if(enc3_cnt != pwm_iron) {
		  pwm_iron = (uint16_t) enc3_cnt;
		  __HAL_TIM_SetCompare(&htim9, TIM_CHANNEL_1, pwm_iron);	// PWM_CH1 = 0 IRON
	  }

	  // Buttons Encoders
	  KeyboardEvent();

	  // Read ADC
	  filter_adc();
	  //adc_ch8 = ADC_iron * ((float)3300.0/4095.0);
	  //adc_ch9 = ADC_air * ((float)3300.0/4095.0);
	  //ADC_MeasurementCorrection();

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
	//ADC_iron = (uint32_t)adcBuffer[0];
	//ADC_air  = (uint32_t)adcBuffer[1];

	flt_adc_8[idx_flt] = (uint32_t)adcBuffer[0];
	flt_adc_9[idx_flt] = (uint32_t)adcBuffer[1];
	flt_adc_t[idx_flt] = (uint32_t)adcBuffer[2];
	flt_adc_v[idx_flt] = (uint32_t)adcBuffer[3];
	idx_flt++;
	if(idx_flt >= 8) {
		idx_flt = 0;
		flt_flag = 1;
	}
}

void filter_adc(void)
{
	if(flt_flag == 1) {
		ADC_iron = ( ( flt_adc_8[0]+flt_adc_8[1]+flt_adc_8[2]+flt_adc_8[3]+flt_adc_8[4]+flt_adc_8[5]+flt_adc_8[6]+flt_adc_8[7] ) / 8 );
		ADC_air  = ( ( flt_adc_9[0]+flt_adc_9[1]+flt_adc_9[2]+flt_adc_9[3]+flt_adc_9[4]+flt_adc_9[5]+flt_adc_9[6]+flt_adc_9[7] ) / 8 );
        ADC_temp = ( ( flt_adc_t[0]+flt_adc_t[1]+flt_adc_t[2]+flt_adc_t[3]+flt_adc_t[4]+flt_adc_t[5]+flt_adc_t[6]+flt_adc_t[7] ) / 8 );
		ADC_vref = ( ( flt_adc_v[0]+flt_adc_v[1]+flt_adc_v[2]+flt_adc_v[3]+flt_adc_v[4]+flt_adc_v[5]+flt_adc_v[6]+flt_adc_v[7] ) / 8 );

		// VDDA can be calculated based on the measured vref and the calibration data
	    vdda = (float)VREFINT_CAL_VREF * (float)*VREFINT_CAL_ADDR / ADC_vref / 1000;

	    // Knowing vdda and the resolution of adc - the actual voltage can be calculated
	    vref = (float) vdda / 4095 * ADC_vref;

	    temp_stm = (float) (ta * (float) (ADC_temp) + tb);
		flt_flag = 0;
	}
}

void calculate_calibration(void)
{
    float x1 = (float) *TEMPSENSOR_CAL1_ADDR;
    float x2 = (float) *TEMPSENSOR_CAL2_ADDR;
    float y1 = (float) TEMPSENSOR_CAL1_TEMP;
    float y2 = (float) TEMPSENSOR_CAL2_TEMP;

    // Simple linear equation y = ax + b based on two points
    ta = (float) ((y2 - y1) / (x2 - x1));
    tb = (float) ((x2 * y1 - x1 * y2) / (x2 - x1));

}

uint32_t map_uint32(uint32_t var, uint32_t x_min, uint32_t x_max, uint32_t y_min, uint32_t y_max)
{
	uint32_t value = var;

	if(value >= x_max) value = x_max;
	return (uint32_t)(value - x_min) * (y_max - y_min) / (x_max - x_min) + x_min;
}

int map_dimmer(int var, int x_min, int x_max, int y_min, int y_max)
{
	int value = var;

	if(value >= x_max) value = x_max;
	return (int)(value - x_min) * (y_max - y_min) / (x_max - x_min) + x_min;
}

// EXTI Line9 External Interrupt ISR Handler CallBackFun
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == ZERO_CROSS_Pin) // If The INT Source Is EXTI Line7 (B7 Pin)
    {
		HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
		Zero_Crossing_Int();
		//HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
    }
}

void Zero_Crossing_Int(void)
{
	NumHandled = 0;

	isHandled[0] = 0;
	isHandled[1] = 0;
	//HAL_GPIO_WritePin(DIMMER_1_GPIO_Port, DIMMER_1_Pin, GPIO_PIN_RESET);
	//HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
	//HAL_GPIO_WritePin(DIMMER_2_GPIO_Port, DIMMER_2_Pin, GPIO_PIN_RESET);
	//htim10.Instance->CNT = 0;
	//dimmer_Counter[0] = 0;
	//dimmer_Counter[1] = 0;

	zero_cross = 1;
}

void dimTimerISR(void)
{
/*	for(int i = 0; i < NUM_DIMMERS; i++) {
		if(pLampState[i] == 1) {
			if(i == 0){
				HAL_GPIO_WritePin(DIMMER_1_GPIO_Port, DIMMER_1_Pin, GPIO_PIN_RESET);
				HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
				pLampState[i] = false;
			}else if(i  == 1){
				HAL_GPIO_WritePin(DIMMER_2_GPIO_Port, DIMMER_2_Pin, GPIO_PIN_RESET);
				pLampState[i] = false;
			}
		}
	}
*/
	if(zero_cross == 1) {
		for(int i = 0; i < NUM_DIMMERS; i++) {
			if(State[i] == 1) {
				if(dimmer_Counter[i] > dimmer_value[i] ) {
					if(i == 0){
						//HAL_GPIO_WritePin(DIMMER_1_GPIO_Port, DIMMER_1_Pin, GPIO_PIN_SET);
						//HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
						pLampState[i] = true;
					}else if(i  == 1){
						//HAL_GPIO_WritePin(DIMMER_2_GPIO_Port, DIMMER_2_Pin, GPIO_PIN_SET);
						pLampState[i] = true;
					}
					dimmer_Counter[i] = 0;
					isHandled[i] = 1;

					NumHandled++;
					if(NumHandled == NumActiveChannels) {
						zero_cross = 0;
					}
				}
				else if(isHandled[i] == 0) {
					dimmer_Counter[i]++;
				}
			}
		}
	}
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
  if(htim->Instance == TIM5) {			// 100Khz - 10us
	  dimTimerISR();
  }
  if(htim->Instance == TIM10) {			// 120Hz - 8.3334ms
	  HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_SET);
	  __NOP();
	  __NOP();
	  __NOP();
	  __NOP();
	  __NOP();
	  __NOP();
	  HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_RESET);
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
