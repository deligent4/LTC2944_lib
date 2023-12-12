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
uint32_t tick, prev_tick = 0;
uint8_t blink_delay = 100;
ltc2944_configuration_t ltc2944_struct = {0};

float voltage, current;
uint16_t volt;

//ltc2944_data_t ltc2944_data;
//float Perscaler_Table[] = {1.0, 4.0, 16.0, 64.0, 256.0, 1024.0, 4096.0};


uint8_t buf[2];
uint32_t voltage_;
uint32_t temp = 3676;
uint16_t qbat = 300, rsns = 5123;
//uint32_t temp;
//uint8_t ctrl_reg[2];
//float prescalar_value;
//uint8_t status;

char stringValue[10];  // Adjust the buffer size as needed
char string_voltage[10], string_current[10];

uint16_t battery_detect = 0;
bool is_ltc2944_config = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Device_Config(void);
uint16_t Get_Voltage(void);

typedef enum
{
	IDLE 		= 0,
	BATT_CONN	= 1,
	RUN			= 3
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
  MX_I2C2_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  ssd1306_Init();
  HAL_ADC_Start(&hadc1);
  state_t state = IDLE;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


	  tick = HAL_GetTick();
	  printf(string_current);


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  HAL_ADC_PollForConversion(&hadc1, 10);
  	  battery_detect = HAL_ADC_GetValue(&hadc1);

	  switch(state){
	  case IDLE:
		  if(battery_detect >= 200){
			  state = BATT_CONN;
		  }else{
			  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, RESET);
			  voltage = 0.0;
			  current = 0.0;
		  }
		  break;

	  case BATT_CONN:
		  Device_Config();
		  state = RUN;
		  break;

	  case RUN:
		  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, SET);
		  if(battery_detect < 200){
			  state = IDLE;
		  }
		  break;
	  default:
	  }

//	  sprintf(string_voltage, "%f", voltage);

//	  sprintf(string_voltage, "%ld", volt);
//	  ssd1306_SetCursor(10, 20);
//	  ssd1306_WriteString(string_voltage, Font_7x10, Black);

//	  sprintf(string_voltage, "%c",v);
	  if(tick - prev_tick >= blink_delay){
		  prev_tick = tick;
		  voltage = LTC2944_Get_Voltage(&ltc2944_struct);
		  current = LTC2944_Get_Current(&ltc2944_struct);

		  sprintf(stringValue, "%ld", tick);
		  ssd1306_SetCursor(10, 10);
		  ssd1306_WriteString(stringValue, Font_7x10, Black);

		  ssd1306_SetCursor(5, 20);
		  ssd1306_WriteString("V=", Font_7x10, Black);
		  sprintf(string_voltage, "%f", voltage);
		  ssd1306_SetCursor(20, 20);
		  ssd1306_WriteString(string_voltage, Font_7x10, Black);

		  ssd1306_SetCursor(5, 30);
		  ssd1306_WriteString("I=", Font_7x10, Black);
		  sprintf(string_current, "%f", current);
		  ssd1306_SetCursor(20, 30);
		  ssd1306_WriteString(string_current, Font_7x10, Black);

		  HAL_GPIO_TogglePin(LED_BLU_GPIO_Port, LED_BLU_Pin);
		  ssd1306_UpdateScreen();
		  ssd1306_Fill(White);
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
void Device_Config(void){
	ltc2944_struct.adc_mode = 			Automatic_Mode;
	ltc2944_struct.prescalar_mode = 	Factor_256;
	ltc2944_struct.alcc_mode = 			ALCC_Disable;
	ltc2944_struct.sense_resistor = 	0.0051;
	ltc2944_struct.i2c_handle = 		hi2c2;

	LTC2944_Init(ltc2944_struct);
}


//static uint8_t LTC2944_Write_Reg(ltc2944_configuration_t *ltc2944, uint8_t data, uint8_t mem_add){
//	uint8_t status;
//	uint8_t buf[1];
//	buf[0] = data;
////	HAL_I2C_Mem_Write(hi2c, DevAddress, MemAddress, MemAddSize, pData, Size, HAL_MAX_DELAY);
//	status = HAL_I2C_Mem_Write(&(ltc2944->i2c_handle), LTC2944_ADDRESS, mem_add, 1, buf, 1, HAL_MAX_DELAY);
//	return status;
//}








uint16_t Get_Voltage(void){

	HAL_I2C_Mem_Read(&hi2c2, LTC2944_ADDRESS, VOLTAGE_MSB, 1, buf, 2, 1000);
	voltage_ = buf[0] << 8 | buf[1];
	return voltage_;
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
