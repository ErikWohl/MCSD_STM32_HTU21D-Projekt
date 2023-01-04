/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HTU21D_ADDR                 0x40

#define HTU21D_CLEAR_BIT            0x40    // Clears any pending interrupt (write 1 to clear)
#define HTU21D_WORD_BIT             0x20    // 1 = read/write word (rather than byte)
#define HTU21D_BLOCK_BIT            0x10    // 1 = using block read/write

#define TRIGGER_TEMP_MEASURE_HOLD    0xE3
#define TRIGGER_HUMD_MEASURE_HOLD    0xE5
#define TRIGGER_TEMP_MEASURE_NOHOLD  0xF3
#define TRIGGER_HUMD_MEASURE_NOHOLD  0xF5
#define WRITE_USER_REG               0xE6
#define READ_USER_REG                0xE7
#define SOFT_RESET                   0xFE

#define I2C_TIMEOUT 50
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
uint8_t addr = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
char HTU21D_Soft_Reset(){
char reg_command[1];
  reg_command[0] = SOFT_RESET;
  //I2C1_Start();                                                // issue I2C start signal
  //I2C1_Write(HTU21D_ADDR,reg_command, 1, END_MODE_STOP);
  HAL_I2C_Master_Transmit(&hi2c1, HTU21D_ADDR << 1, reg_command, sizeof(reg_command), I2C_TIMEOUT);
  HAL_Delay(15);
}

void HTU21D_REG_read(){
  char reg_data[1], txt[12];
  int reg_value = 0;
  reg_data[0] = READ_USER_REG;
  //I2C1_Start();
  //I2C1_Write(HTU21D_ADDR, reg_data, 1, END_MODE_RESTART);
  //I2C1_Read (HTU21D_ADDR, reg_data, 1, END_MODE_STOP);
  HAL_I2C_Master_Transmit(&hi2c1, HTU21D_ADDR << 1, reg_data, sizeof(reg_data), I2C_TIMEOUT);
  HAL_I2C_Master_Receive(&hi2c1, HTU21D_ADDR << 1, reg_data, sizeof(reg_data), I2C_TIMEOUT);

  reg_value=reg_data[0];

  IntToStr(reg_value, txt);
  UART1_Write_Text(txt);
}

void HTU21D_REG_write(){
  char reg_data[2], txt[12];
  int reg_value=0;
  reg_data[0]=WRITE_USER_REG;
  reg_data[1]=0xC1;
  HAL_I2C_Master_Transmit(&hi2c1, HTU21D_ADDR << 1, reg_data, sizeof(reg_data), I2C_TIMEOUT);
  //I2C1_Start();
  //I2C1_Write(HTU21D_ADDR, reg_data, 2, END_MODE_STOP);
  reg_value = reg_data[0];
}

unsigned HTU21D_ReadValue(char regSelect){
unsigned value = 0;
char reg_data[3];

  reg_data[0] = regSelect;
  //I2C1_Start();
  //I2C1_Write(HTU21D_ADDR, reg_data, 1, END_MODE_RESTART);
  //I2C1_Read (HTU21D_ADDR, reg_data, 3, END_MODE_STOP);
  HAL_I2C_Master_Transmit(&hi2c1, HTU21D_ADDR << 1, reg_data, 1, I2C_TIMEOUT);
  HAL_I2C_Master_Receive(&hi2c1, HTU21D_ADDR << 1, reg_data, sizeof(reg_data), I2C_TIMEOUT);

  /*
  char output[12];

	char *beginMsg = "BITs: ";
	char *spaceMsg = " ";
	if(HAL_UART_Transmit(&huart2, (uint8_t *)beginMsg, strlen(beginMsg), 1000)==HAL_ERROR)Error_Handler();

	itoa(reg_data[0], output, 2);
	if(HAL_UART_Transmit(&huart2, (uint8_t*) output, strlen(output), 1000)==HAL_ERROR)Error_Handler();
	if(HAL_UART_Transmit(&huart2, (uint8_t *)spaceMsg, strlen(spaceMsg), 1000)==HAL_ERROR)Error_Handler();

	itoa(reg_data[1], output, 2);
	if(HAL_UART_Transmit(&huart2, (uint8_t*) output, strlen(output), 1000)==HAL_ERROR)Error_Handler();
	if(HAL_UART_Transmit(&huart2, (uint8_t *)spaceMsg, strlen(spaceMsg), 1000)==HAL_ERROR)Error_Handler();

	itoa(reg_data[2], output, 2);
	if(HAL_UART_Transmit(&huart2, (uint8_t*) output, strlen(output), 1000)==HAL_ERROR)Error_Handler();
	if(HAL_UART_Transmit(&huart2, (uint8_t *)spaceMsg, strlen(spaceMsg), 1000)==HAL_ERROR)Error_Handler();
*/

  value = ((unsigned)reg_data[0] << 8) | reg_data[1] ;
  return value & 0xFFFC;            // Clear status bits
  //return value;
}

float procTemperatureValue(unsigned ValueTemp){
  float calc;
  calc = -46.85 + 175.72 * ValueTemp / 65536.0;
  return calc;
}

float procHumidityValue(unsigned ValueTemp){
  char txt[12];
  float calc;
  calc = -6.0 + 125.0 * ValueTemp / 65536.0;
  return calc;
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HTU21D_Soft_Reset();
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, 1);
  HAL_Delay(50);
  while (1)
  {
	char buf[10];
	float x = 0;
	x = procTemperatureValue(HTU21D_ReadValue(TRIGGER_TEMP_MEASURE_HOLD));
	gcvt(x, 7, buf);

	char bufHum[10];
	float hum = 0;
	hum = procHumidityValue(HTU21D_ReadValue(TRIGGER_HUMD_MEASURE_HOLD));
	gcvt(hum, 7, bufHum);

	char *beginMsg = "Temp: ";
	if(HAL_UART_Transmit(&huart2, (uint8_t *)beginMsg, strlen(beginMsg), 1000)==HAL_ERROR)Error_Handler();

	if(HAL_UART_Transmit(&huart2, (uint8_t*) buf, strlen(buf), 1000)==HAL_ERROR)Error_Handler();

	char *midMsg = "°C Humid: ";
	if(HAL_UART_Transmit(&huart2, (uint8_t *)midMsg, strlen(midMsg), 1000)==HAL_ERROR)Error_Handler();

	if(HAL_UART_Transmit(&huart2, (uint8_t*) bufHum, strlen(bufHum), 1000)==HAL_ERROR)Error_Handler();

	char *receiveMsg = "%\n\r";
	if(HAL_UART_Transmit(&huart2, (uint8_t *)receiveMsg, strlen(receiveMsg), 1000)==HAL_ERROR)Error_Handler();
	HAL_Delay(1000);
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

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 16;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00707CBB;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

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
