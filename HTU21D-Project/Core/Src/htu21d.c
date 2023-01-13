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
#include <stdbool.h>
/*--- CUSTOM LIBRARIES ---*/
#include "htu21d.h"
/*--- GLOBAL VARIABLES ---*/
I2C_HandleTypeDef i2cDef;
HTU21D_WARNING currentWarning;
/* -------- METHODS -------- */


/**
  * @brief  Sets the i2c variable for communication.
  * @param  i2cVal i2c address
  * @retval none
  */
void HTU21D_Init(I2C_HandleTypeDef i2cVal) {
	i2cDef = i2cVal;
	currentWarning.check_value_type = NONE;
	currentWarning.check_trigger = BELOW;
	currentWarning.check_threshold = 0;
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
  * @param  sensorValue data value from sensor
  * @retval temperature
  */
float procTemperatureValue(unsigned sensorValue){
  float calc;
  calc = -46.85 + 175.72 * sensorValue / 65536.0;
  return calc;
}

/**
  * @brief  Calculates the humidity from the raw data
  * @param  sensorValue data value from sensor
  * @retval humidity
  */
float procHumidityValue(unsigned sensorValue){
  float calc;
  calc = -6.0 + 125.0 * sensorValue / 65536.0;
  return calc;
}

/**
  * @brief  Converts the temperature from the celsius to fahrenheit
  * @param  tempInCelsius temperature in °C
  * @retval temmperature in °F
  */
float calcCelsiusToFahrenheit(float tempInCelsius) {
	return tempInCelsius * (9.0/5.0) + 32.0;
}

/**
  * @brief  Calculates the partial pressure from the temperature
  * @param  temperature value from sensor after calculation
  * @retval partial pressure
  */
float calculatePartialPressure(float temperature) {
	double power = Variable_A - (Variable_B / (temperature + Variable_C));
	return (float)pow(10, power);
}

/**
  * @brief  Calculates the dew point temperature from the humidity and partial pressure
  * @param  humidity value from sensor after calculation
  * @param  partial pressure calculated value
  * @retval dew point temperature
  */
float calculateDewPointTemperature(float humidity, float partialPressure) {
	double result = (Variable_B / (log10(humidity * (partialPressure / 100))- Variable_A)) + Variable_C;
	return (float)result * (-1);
}

/**
  * @brief  Set the value type and threshold of the warning that should be checked
  * @param  value_type type of value that should be checked
  * @param  trigger if the warning should be triggered, when the value is below or above a certain threshold
  * @param  threshold the threshold value that needs to be checked
  * @retval none
  */
void setWarning(CHECK_VALUE_TYPE value_type, CHECK_TRIGGER trigger, float threshold) {
	currentWarning.check_value_type = value_type;
	currentWarning.check_trigger = trigger;
	currentWarning.check_threshold = threshold;
}

/**
  * @brief  Returns the value depending on what value type was inputted
  * @param  value_type type of value that should be pulled
  * @retval sensor value of the input type
  */
float getValue(CHECK_VALUE_TYPE value_type) {
	float value = 0;

	switch(value_type) {
		case TEMPERATURE: {
			value = procTemperatureValue(HTU21D_ReadValue(TRIGGER_TEMP_MEASURE_HOLD));
			break;
		}

		case HUMIDITY: {
			value = procHumidityValue(HTU21D_ReadValue(TRIGGER_HUMD_MEASURE_HOLD));
			break;
		}

		case PARTIAL_PRESSURE: {
			float temp = procTemperatureValue(HTU21D_ReadValue(TRIGGER_TEMP_MEASURE_HOLD));
			value = calculatePartialPressure(temp);
			break;
		}

		case DEW_POINT_TEMPERATURE: {
			float temp = procTemperatureValue(HTU21D_ReadValue(TRIGGER_TEMP_MEASURE_HOLD));
			float hum = procHumidityValue(HTU21D_ReadValue(TRIGGER_HUMD_MEASURE_HOLD));
			float pressure = calculatePartialPressure(temp);
			value = calculateDewPointTemperature(hum, pressure);
			break;
		}

		default: {
			value = 0;
		}
	}

	return value;
}

/**
  * @brief  Checks if a warning is set and if the warning is triggered
  * @param  value that needs to be checked
  * @param  value type that needs to be checked
  * @retval if the warning is triggered
  */
bool isTriggered(float value, CHECK_VALUE_TYPE value_type) {
	if(currentWarning.check_value_type == NONE || currentWarning.check_value_type != value_type) {
		return false;
	}

	if(currentWarning.check_trigger == BELOW && value < currentWarning.check_threshold) {
		return true;
	}

	if(currentWarning.check_trigger == ABOVE && value > currentWarning.check_threshold) {
		return true;
	}

	return false;
}


