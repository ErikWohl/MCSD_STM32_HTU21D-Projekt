/*
 * htu21d.c
 *
 *  Created on: 4 Jan 2023
 *      Author: eriky
 */

/*--- COMMON LIBRARIES ---*/
#include <string.h>
#include <stdio.h>
#include <math.h>

/*--- CUSTOM LIBRARIES ---*/
#include "htu21d.h"
/*--- GLOBAL VARIABLES ---*/
I2C_HandleTypeDef i2cDef;
/* -------- METHODS -------- */


/**
  * @brief  Sets the i2c variable for communication.
  * @param  i2cVal i2c address
  * @retval none
  */
void HTU21D_Init(I2C_HandleTypeDef i2cVal) {
	i2cDef = i2cVal;
}

/**
  * @brief  Reboots the HTU21D sensor switching the power off and on again.
  * @retval none
  */
void HTU21D_Soft_Reset(){
char reg_command[1];
  reg_command[0] = SOFT_RESET;
  HAL_I2C_Master_Transmit(&i2cDef, HTU21D_ADDR << 1, (uint8_t *)reg_command, sizeof(reg_command), I2C_TIMEOUT);
  HAL_Delay(15);
}

/**
  * @brief  Receives an amount of data from the specified register.
  * @param  regSelect register adress.
  * @retval register value cleared of the status bit.
  */
unsigned HTU21D_ReadValue(char regSelect){
unsigned value = 0;
char reg_data[3];

  reg_data[0] = regSelect;
  HAL_I2C_Master_Transmit(&i2cDef, HTU21D_ADDR << 1, (uint8_t *)reg_data, 1, I2C_TIMEOUT);
  HAL_I2C_Master_Receive(&i2cDef, HTU21D_ADDR << 1, (uint8_t *)reg_data, sizeof(reg_data), I2C_TIMEOUT);

  /* FOR TESTING PURPOSES, ENABLE TO SEE THE RECEIVED BITS
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

/**
  * @brief  Calculates the temperature from the raw data
  * @param valueTemp data value from sensor
  * @retval temperature
  */
float procTemperatureValue(unsigned valueTemp){
  float calc;
  calc = -46.85 + 175.72 * valueTemp / 65536.0;
  return calc;
}

/**
  * @brief  Calculates the humidity from the raw data
  * @param valueTemp data value from sensor
  * @retval humidity
  */
float procHumidityValue(unsigned valueTemp){
  float calc;
  calc = -6.0 + 125.0 * valueTemp / 65536.0;
  return calc;
}

/**
  * @brief  Calculates the partial pressure from the temperature
  * @param temperature value from sensor after calculation
  * @retval partial pressure
  */
float calculatePartialPressure(float temperature) {
	double power = Variable_A - (Variable_B / (temperature + Variable_C));
	return (float)pow(10, power);
}

/**
  * @brief  Calculates the dew point temperature from the humidity and partial pressure
  * @param humidity value from sensor after calculation
  * @param partial pressure calculated value
  * @retval dew point temperature
  */
float calculateDewPointTemperature(float humidity, float partialPressure) {
	double result = (Variable_B / (log10(humidity * (partialPressure / 100))- Variable_A)) + Variable_C;
	return (float)result * (-1);
}
