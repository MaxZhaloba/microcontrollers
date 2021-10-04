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

  internal_test(500);
  clear();
  HAL_Delay(1000);

  brightness(0);
  switch_on();

  //send_command(0x0101);

  //uint8_t data[] = {1, 2, 3, 4, 5, 6, 7, 8};

  custom_test(2000);
  clear();
  HAL_Delay(1000);

  display_hello();
  HAL_Delay(3000);
  clear();

  //send_character(5, 0x80);
  //send_character(6, 0x80);

  // HAL_Delay(5000);
  // switch_off();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      

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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Clock_Pin | CS_Pin | Data_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Clock_Pin Data_Pin */
  GPIO_InitStruct.Pin = Clock_Pin | Data_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_Pin */
  GPIO_InitStruct.Pin = CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);
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

#ifdef USE_FULL_ASSERT
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
