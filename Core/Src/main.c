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
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ltc2944.h"
#include "ssd1306.h"
#include "ssd1306_tests.h"
#include <stdbool.h>
#include <stdio.h>

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

/* USER CODE BEGIN PV */
uint32_t tick, prev_tick = 0, i2c_timeout = 10;
uint16_t blink_delay = 10;
ltc2944_configuration_t ltc2944_struct = {0};
extern ltc2944_data_t ltc2944_data;

float voltage, current, charge, temperature;
uint16_t volt;

uint8_t buf[2], status;
uint32_t voltage_;
uint16_t temp;
uint16_t qbat = 300, rsns = 5123;
uint16_t sec_prev = 0, seconds = 0;
//uint32_t temp;
//uint8_t ctrl_reg[2];
//float prescalar_value;
uint8_t state;

char stringTick[10];  // Adjust the buffer size as needed
char stringTest1[12], stringTest2[12];
char string_voltage[10], string_current[10], string_charge[10], string_temperature[10];

bool battery_detect = false;
bool is_ltc2944_config = false;
//float prescaler_value, psc_temp;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
HAL_StatusTypeDef Device_Config(void);
uint16_t Get_Voltage(void);
void myOLED_char(uint16_t cursorX, uint16_t cursorY, char* data);
void myOLED_float(uint16_t cursorX, uint16_t cursorY, float data);
void myOLED_int(uint16_t cursorX, uint16_t cursorY, uint16_t data);
void myOLED_int8(uint16_t cursorX, uint16_t cursorY, uint8_t data);
void SCL_Reconfig();

typedef enum
{
	IDLE 		= 0,
	BATT_CONN	= 1,
	RUN			= 2,
	STUCK		= 3
}state_t;

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}
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
  SystemClock_Config();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C2_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  ssd1306_Init();
  HAL_ADC_Start(&hadc1);
//  state_t state = IDLE;

//  while(Device_Config()){
//  }
  HAL_Delay(100);
  myOLED_char(1, 12, "Volt = ");
  myOLED_char(1, 24, "Curr = ");
  myOLED_char(1, 36, "Chg  = ");
  myOLED_char(1, 48, "Temp = ");
  ssd1306_UpdateScreen();
  HAL_Delay(500);

//  ssd1306_Fill(White);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  tick = HAL_GetTick();
	  myOLED_int(1, 2, tick);
//	  LTC2944_Init(ltc2944_struct);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  HAL_ADC_PollForConversion(&hadc1, 10);
  	  if(HAL_ADC_GetValue(&hadc1) >= 200){
  		  battery_detect = true;
  	  }else if(HAL_ADC_GetValue(&hadc1) < 200){
  		  battery_detect = false;
  	  }

	  switch(state){
	  case IDLE:
		  if(battery_detect){
			  state = BATT_CONN;
		  }else{
			  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, RESET);
			  myOLED_char(50, 24, "        ");	// print empty spaces in curr
			  myOLED_char(50, 36, "       ");	// print empty spaces in chg
			  myOLED_char(50, 48, "  ");		// print empty spaces in temp
			  myOLED_int(50, 2, 0);
			  // Resets the seconds count every time battery is removed
			  if(seconds > 1){
				  seconds = 0;
			  }
		  }
		  break;

	  case BATT_CONN:
		  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, SET); 		// Turn on RED led for indication
		  Device_Config();
		  state = RUN;
		  break;

	  case RUN:
		  /*
		  * test timer for run condition
		  */
		  if(tick - sec_prev >= 1000){		// 1000ms = 1 sec
			  sec_prev = tick;
			  myOLED_int(50, 2, seconds++);
		  }
		  if(battery_detect){
			  uint32_t tickstart = HAL_GetTick();
			  status = LTC2944_Get_Battery_Data(&ltc2944_struct);
			  if((HAL_GetTick() - tickstart) > i2c_timeout){
				  state = STUCK;
				  break;
			  }
			  // print the battery values on oled screen
			  myOLED_float(50, 12, ltc2944_data.voltage);
			  myOLED_float(50, 24, ltc2944_data.current);
			  myOLED_int(50, 36, ltc2944_data.acc_charge);
			  myOLED_int(50, 48, ltc2944_data.temperature);
//			  if(status != HAL_OK){
//				  state = STUCK;
//				  break;
//			  }
		  }else{
			  state = IDLE;
		  }
		  break;

	  case STUCK:		// state to handle stuck LTC2944
		  if(battery_detect){
//			  HAL_I2C_DeInit(&hi2c2);
//			  SCL_Reconfig();
		  }else{
			  state = IDLE;
		  }

		  break;

	  default:
	  }

	  if(tick - prev_tick >= blink_delay){
		  prev_tick = tick;
		  HAL_GPIO_TogglePin(LED_BLU_GPIO_Port, LED_BLU_Pin);
		  myOLED_int(75, 48, state);
		  myOLED_int(95, 48, status);
		  ssd1306_UpdateScreen();
	  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
HAL_StatusTypeDef Device_Config(void){
	ltc2944_struct.adc_mode = 			Automatic_Mode;
	ltc2944_struct.alcc_mode = 			ALCC_Disable;
	ltc2944_struct.sense_resistor = 	5;
	ltc2944_struct.batt_capacity =		3500;
	ltc2944_struct.i2c_handle = 		hi2c2;

	return LTC2944_Init(ltc2944_struct);
}

void myOLED_char(uint16_t cursorX, uint16_t cursorY, char* data){

	ssd1306_SetCursor(cursorX, cursorY);
	ssd1306_WriteString(data, Font_7x10, White);
}

void myOLED_float(uint16_t cursorX, uint16_t cursorY, float data){
	char str_data[10];

	sprintf(str_data, "%.3f", data);
	ssd1306_SetCursor(cursorX, cursorY);
	ssd1306_WriteString(str_data, Font_7x10, White);
}

void myOLED_int(uint16_t cursorX, uint16_t cursorY, uint16_t data){
	char str_data[10];

	sprintf(str_data, "%u", data);
	ssd1306_SetCursor(cursorX, cursorY);
	ssd1306_WriteString(str_data, Font_7x10, White);
}

void myOLED_int8(uint16_t cursorX, uint16_t cursorY, uint8_t data){
	char str_data[10];

	sprintf(str_data, "%d", data);
	ssd1306_SetCursor(cursorX, cursorY);
	ssd1306_WriteString(str_data, Font_7x10, White);
}

void SCL_Reconfig(){
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);

	/*Configure GPIO pins : PCPin PCPin PCPin */
	GPIO_InitStruct.Pin = GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

//uint16_t Get_Voltage(void){
//
//	HAL_I2C_Mem_Read(&hi2c2, LTC2944_ADDRESS, ACCUMULATED_CHARGE_MSB, 1, buf, 2, 1000);
//	voltage_ = buf[0] << 8 | buf[1];
//	return voltage_;
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
