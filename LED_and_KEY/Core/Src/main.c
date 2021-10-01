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

// GPIO_PIN_2
#define CLOCK_PIN ((uint16_t)0x0004)
// GPIO_PIN_3
#define DATA_PIN ((uint16_t)0x0008)
// GPIO_PIN_4
#define CS_PIN ((uint16_t)0x0010)
 
void cs_high(){
  HAL_GPIO_WritePin(GPIOA, CS_PIN, GPIO_PIN_SET);
  HAL_Delay(10);
}

void cs_low(){
  HAL_GPIO_WritePin(GPIOA, CS_PIN, GPIO_PIN_RESET);
}

void clock_high(){
  HAL_GPIO_WritePin(GPIOA, CLOCK_PIN, GPIO_PIN_SET);
}

void clock_low(){
  HAL_GPIO_WritePin(GPIOA, CLOCK_PIN, GPIO_PIN_RESET);
}

// void send_byte(char data)
// {
//   for (int i = 0; i < 8; i++) {
//     digitalWrite(clockPin, LOW);
//     digitalWrite(dataPin, data & 1 ? HIGH : LOW);
//     data >>= 1;
//     digitalWrite(clockPin, HIGH);
//   }
// }

void send_byte(char data)
{
  for (int i = 0; i < 8; i++) {
    clock_low();

    HAL_GPIO_WritePin(GPIOA, DATA_PIN, data & 1 ? GPIO_PIN_SET : GPIO_PIN_RESET);

    data >>= 1;
    clock_high();
  }

  cs_low();
}

void send_byte_old(unsigned char b)
{
  unsigned char mask = 1;

  HAL_Delay(1);

  for (int i=0; i<8; i++){
    unsigned char d = b & mask;
    mask = mask << 1;

  
    // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

    //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
    clock_low();

    if (d)
    {
      HAL_GPIO_WritePin(GPIOA, DATA_PIN, GPIO_PIN_SET);
    } else 
    {
       HAL_GPIO_WritePin(GPIOA, DATA_PIN, GPIO_PIN_RESET);
    }

    clock_high();

    // HAL_Delay(1);
    //HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
    // HAL_Delay(1);
  }
  
  HAL_Delay(1);

  clock_low();
  HAL_GPIO_WritePin(GPIOA, DATA_PIN, GPIO_PIN_SET);
}

void fill(unsigned char data){
  cs_low();

  send_byte(0x40);
  send_byte(0xC0);

  for (int i=0; i<8; i++){
    send_byte(data);
    send_byte(0x0); //LED OFF
  }

  cs_high();
}

// void fill(char data){
//   send_byte(0x44);

//   for (int i=0; i<16; i++){
//     send_byte(0xC0 + i);
//     send_byte(data);
//   }
// }

void reset(){
  fill(0x0);
}

// 8-byte array
// void display(unsigned char data[]){
  
  
//   for (unsigned char i=0; i<8; i++){
//     unsigned char *pointer = data;
//     pointer += i;

//     send_byte(0x44);
//     send_byte(0xC0 + 2*i);
//     send_byte(*pointer);
//   }
// }

// 8-byte array
void display(unsigned char data[]){
  cs_low();

  send_byte(0x40);
  send_byte(0xC0);

  for (int i=0; i<8; i++){
    send_byte(data[i]);
    send_byte(0x0); //LED OFF
  } 

  cs_high();
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
  // char n = 0;

//  char message[] = "Hello World!";

  // for (int i=0; i<sizeof(message); i++)
  // {
  //   send_byte(message[i]);
  // }

    HAL_Delay(100);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET); 

    send_byte(0x88); // 1/16 brightness

    // reset();

    // fill(0x7F);

    // send_byte(0x44); //Fixed address mode

    // send_byte(0xC0);
    // send_byte(0x00);
    //send_byte(0xFF);

    // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

    char n=0;

// unsigned char values[] = {0, 1, 2, 4, 8, 16, 32, 64, 128 };

unsigned char values[] = {0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F};

// HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET); 
    // send_byte(0x44);
    // send_byte(0xC0);
    // send_byte(values[0]);

    // send_byte(0x44);
    // send_byte(0xC2);
    // send_byte(values[1]);

  cs_low();
  clock_low();

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // send_byte(n);
    // unsigned char value = values[n];

    // display(values);
    // n++;

      // command_delay();

      // send_byte(0x40);
      // send_byte(0xCC);
      
      // send_byte(0x4F);
      // send_byte(0xFF);
      

      // command_delay();

      // send_byte(0x40);
      // send_byte(0xC2);
      // send_byte(0xFF);
      // send_byte(0x66);

      // command_delay();

      reset();
      // command_delay();

      HAL_Delay(500);

      display(values);

      // command_delay();

      HAL_Delay(500);

      // send_byte(0x40);
      // send_byte(0xC0);

      // send_byte(0x01);
      // send_byte(0xFF);

      // send_byte(0x3F);
      // send_byte(0x00);
      
      // send_byte(0x06);
      // send_byte(0x00);
      
      // send_byte(0x5B);
      // send_byte(0x00);
      
      // send_byte(0x4F);
      // send_byte(0x00);

      // send_byte(0x66);
      // send_byte(0x00);
      
      // send_byte(0x6D);
      // send_byte(0x00);

      // send_byte(0x7D);
      // send_byte(0x00);

      // send_byte(0x07);
      // send_byte(0x00);

      // fill(0xFF);
      // fill(0x3F);

      // display(values);

      // command_delay();

    if (n>sizeof(values))
    {
      n=0;
    }
    // reset();

    // HAL_Delay(500);

    // send_byte(0x00);
    // send_byte(0x89);

    // send_byte(0xC0);
    // send_byte(0xFF);
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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Clock_Pin|GPIO_PIN_3|Strobe_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Clock_Pin PA3 */
  GPIO_InitStruct.Pin = Clock_Pin|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Strobe_Pin */
  GPIO_InitStruct.Pin = Strobe_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Strobe_GPIO_Port, &GPIO_InitStruct);

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
