/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int value = 10;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == CLK_Pin)
	{
		uint8_t dt = HAL_GPIO_ReadPin(DT_GPIO_Port, DT_Pin);

		int increment = dt ? 1 : -1;

		value += increment;

		if (value < 0)
		{
			value = 0;
		}
	}
}

// Pin 3
#define CLOCK_PIN GPIO_PIN_3
// Pin 4
#define CS_PIN GPIO_PIN_4
// Pin 5
#define DATA_PIN GPIO_PIN_5

// No-decode mode data bits and corresponding segment lines for standard 7-segment LED
#define SEG_DOT 0x80
#define SEG_A 0x40
#define SEG_B 0x20
#define SEG_C 0x10
#define SEG_D 0x08
#define SEG_E 0x04
#define SEG_F 0x02
#define SEG_G 0x01
#define SEG_BLANK 0x0

// Segment display data
#define CHAR_0 (SEG_A|SEG_B|SEG_C|SEG_D|SEG_E|SEG_F)
#define CHAR_1 (SEG_B|SEG_C)
#define CHAR_2 (SEG_A|SEG_B|SEG_D|SEG_E|SEG_G)
#define CHAR_3 (SEG_A|SEG_B|SEG_C|SEG_D|SEG_G)
#define CHAR_4 (SEG_B|SEG_C|SEG_F|SEG_G)
#define CHAR_5 (SEG_A|SEG_C|SEG_D|SEG_F|SEG_G)
#define CHAR_6 (SEG_A|SEG_D|SEG_C|SEG_E|SEG_F|SEG_G)
#define CHAR_7 (SEG_A|SEG_B|SEG_C)
#define CHAR_8 (SEG_A|SEG_B|SEG_C|SEG_D|SEG_E|SEG_F|SEG_G)
#define CHAR_9 (SEG_A|SEG_B|SEG_C|SEG_D|SEG_G|SEG_F)
#define CHAR_SPACE (SEG_BLANK)
#define CHAR_H (SEG_B|SEG_C|SEG_E|SEG_F|SEG_G)
#define CHAR_E (SEG_A|SEG_D|SEG_E|SEG_F|SEG_G)
#define CHAR_L (SEG_D|SEG_E|SEG_F)
#define CHAR_O (SEG_A|SEG_B|SEG_C|SEG_D|SEG_E|SEG_F)
#define CHAR_EXCLAMATION_MARK (SEG_B|SEG_C|SEG_DOT)
#define CHAR_UNDERSCORE (SEG_D)



void cs_disable()
{
  HAL_GPIO_WritePin(GPIOA, CS_PIN, GPIO_PIN_SET);
  // HAL_Delay(10);
}

void cs_enable()
{
  HAL_GPIO_WritePin(GPIOA, CS_PIN, GPIO_PIN_RESET);
}

void clock_high()
{
  HAL_GPIO_WritePin(GPIOA, CLOCK_PIN, GPIO_PIN_SET);
}

void clock_low()
{
  HAL_GPIO_WritePin(GPIOA, CLOCK_PIN, GPIO_PIN_RESET);
}

void send_word(uint16_t word)
{
  uint16_t mask = 1 << 15;

  // HAL_Delay(1);

  for (int i = 0; i < 16; i++)
  {
    uint16_t bit = word & mask;
    mask >>= 1;

    clock_low();

    HAL_GPIO_WritePin(GPIOA, DATA_PIN, bit ? GPIO_PIN_SET : GPIO_PIN_RESET);

    clock_high();
  }

  clock_low();
  // HAL_Delay(1);
  HAL_GPIO_WritePin(GPIOA, DATA_PIN, GPIO_PIN_RESET);
}

void send_command(uint16_t command)
{
  cs_enable();
  send_word(command);
  cs_disable();
}

void set_no_decode_mode()
{
  send_command(0x0900);
}

void switch_on(){
  send_command(0x0B0F); // Set scan limit to display all 8 segments
  set_no_decode_mode();
  send_command(0x0C01); // Switch to normal operation mode
}

void switch_off(){
  send_command(0x0C00); // Switch to shutdown mode
}

void brightness(uint8_t level)
{
  send_command(0x0A00 + level);
}

void test_on()
{
  send_command(0x0F01);
}

void test_off()
{
  send_command(0x0F00); // Switch to shutdown mode
}

void send_character(uint8_t position, uint8_t character)
{
  uint16_t address = (position+1)<<8;
  send_command(address | character);
}

// Data is 8-byte array
void fill(uint8_t data[])
{
  for (uint16_t i=0; i<8; i++)
  {
    // Swap the segments positioned from right to left on the PCB
    send_character(7-i, data[i]);
  }
}

void clear()
{
  for (int i=0; i<8; i++)
  {
    send_character(i, 0);
  }
}

// Duration is set in ms
void internal_test(uint32_t display_duration)
{
  test_on();
  HAL_Delay(display_duration);
  test_off();
}

void display_custom_test_data()
{
  uint8_t data[] = {CHAR_0|SEG_DOT, CHAR_1, CHAR_2, CHAR_3, CHAR_4, CHAR_5, CHAR_6, CHAR_7};
  //uint8_t data[] = {CHAR_0, CHAR_0, CHAR_0, CHAR_0, CHAR_0, CHAR_0, CHAR_8, CHAR_9};
  fill(data);
}

// Duration is set in ms
void custom_test(uint32_t display_duration)
{
  display_custom_test_data();
  HAL_Delay(display_duration);
  clear();
}

void display_hello(){
  uint8_t data[] = {CHAR_H, CHAR_E, CHAR_L, CHAR_L, CHAR_O, CHAR_SPACE, CHAR_SPACE, CHAR_SPACE};
  fill(data);
}

void convert_int_to_8_digits(int n, uint8_t *buffer)
{
	const uint8_t size = 8;
	const uint8_t arithmetic_base = 10;

	for (uint8_t i=0; i<size; i++)
	{
		uint8_t digit = n % arithmetic_base;
		n /= arithmetic_base;

		buffer[size - 1 - i] = digit;
	}
}

uint8_t convert_digit_to_symbol(uint8_t digit)
{
	const uint8_t arithmetic_base = 10;
	const uint8_t symbols[] = {CHAR_0, CHAR_1, CHAR_2, CHAR_3, CHAR_4, CHAR_5, CHAR_6, CHAR_7, CHAR_8, CHAR_9};

	uint8_t symbol = CHAR_UNDERSCORE; // For digits greater than 9

	if (digit < arithmetic_base)
	{
		symbol = symbols[digit];
	}

	return symbol;
}

void convert_8_digits_to_symbols(uint8_t *digits_buffer, uint8_t *symbol_buffer)
{
	for (uint8_t i=0; i<8; i++)
	{
		symbol_buffer[i] = convert_digit_to_symbol(digits_buffer[i]);
	}
}

void convert_int_to_8_symbols(int n, uint8_t *symbol_buffer)
{
	uint8_t digits_buffer[8];

	convert_int_to_8_digits(n, digits_buffer);

	convert_8_digits_to_symbols(digits_buffer, symbol_buffer);
}

void replace_leading_zeroes(uint8_t *symbol_buffer, int size)
{
	for (int i=0; i<size-1; i++)
	{
		if (symbol_buffer[i] == CHAR_0)
		{
			symbol_buffer[i] = CHAR_SPACE;
		} else
		{
			break;
		}
	}
}

void display_int(int n)
{
	uint8_t symbol_buffer[8];

	convert_int_to_8_symbols(n, symbol_buffer);

	replace_leading_zeroes(symbol_buffer, 8);

	fill(symbol_buffer);
}

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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  internal_test(500);
  clear();

  brightness(0);
  switch_on();

  custom_test(500);
  clear();
  HAL_Delay(500);

  display_hello();
  HAL_Delay(500);
  clear();

  while (1)
  {
//	  HAL_Delay(delay);
//	  HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

	  display_int(value);

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Clock_Pin|CS_Pin|Data_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Clock_Pin CS_Pin Data_Pin */
  GPIO_InitStruct.Pin = Clock_Pin|CS_Pin|Data_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_Pin */
  GPIO_InitStruct.Pin = CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CLK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DT_Pin */
  GPIO_InitStruct.Pin = DT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
