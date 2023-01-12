/*
 * htu21d.h
 *
 *  Created on: 4 Jan 2023
 *      Author: eriky
 */

#pragma once
#ifndef INC_HTU21D_H_
#define INC_HTU21D_H_

/* -------- IMPORTS -------- */

#include "main.h"

/* ########################## ADDRESSES ############################## */

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

/* ########################## STRUCTURES ############################## */

typedef enum {
	NONE = 0x00,
	TEMPERATURE,
	HUMIDITY,
	PARTIAL_PRESSURE,
	DEW_POINT_TEMPERATURE
} CHECK_VALUE_TYPE;

typedef enum {
	BELOW = 0x00,
	ABOVE
} CHECK_TRIGGER;

typedef struct {
	CHECK_VALUE_TYPE check_value_type;
	CHECK_TRIGGER check_trigger;
	float check_threshold;
} HTU21D_WARNING;

/* ########################## OTHER DECLARATIONS ############################## */

// Constant values needed to calculate the partial pressure and dew point temperature
#define Variable_A 8.1332
#define Variable_B 1762.39
#define Variable_C 235.66

extern I2C_HandleTypeDef i2cDef;
extern HTU21D_WARNING currentWarning;
/* ########################## METHODS ############################## */

/**
  * @brief  Sets the i2c variable for communication.
  * @param  i2cVal i2c address
  * @retval none
  */
extern void HTU21D_Init(I2C_HandleTypeDef i2cVal);

/**
  * @brief  Reboots the HTU21D sensor switching the power off and on again.
  * @retval none
  */
extern void HTU21D_Soft_Reset();

/**
  * @brief  Receives an amount of data from the specified register.
  * @param  regSelect register adress.
  * @retval register value cleared of the status bit.
  */
extern unsigned HTU21D_ReadValue(char regSelect);

/**
  * @brief  Calculates the temperature from the raw data
  * @param  valueTemp data value from sensor
  * @retval temperature
  */
extern float procTemperatureValue(unsigned valueTemp);

/**
  * @brief  Calculates the humidity from the raw data
  * @param  valueTemp data value from sensor
  * @retval humidity
  */
extern float procHumidityValue(unsigned valueTemp);

/**
  * @brief  Converts the temperature from the celsius to fahrenheit
  * @param  tempInCelsius temperature in °C
  * @retval temmperature in °F
  */
float calcCelsiusToFahrenheit(float tempInCelsius);

/**
  * @brief  Calculates the partial pressure from the temperature
  * @param  temperature value from sensor after calculation
  * @retval partial pressure
  */
extern float calculatePartialPressure(float temperature);

/**
  * @brief  Calculates the dew point temperature from the humidity and partial pressure
  * @param  humidity value from sensor after calculation
  * @param  partial pressure calculated value
  * @retval dew point temperature
  */
extern float calculateDewPointTemperature(float humidity, float partialPressure);

/**
  * @brief  Set the value type and threshold of the warning that should be checked
  * @param  value_type type of value that should be checked
  * @param  trigger if the warning should be triggered, when the value is below or above a certain threshold
  * @param  threshold the threshold value that needs to be checked
  * @retval none
  */
extern void setWarning(CHECK_VALUE_TYPE value_type, CHECK_TRIGGER trigger, float threshold);

/**
  * @brief  Returns the value depending on what value type was inputted
  * @param  value_type type of value that should be pulled
  * @retval sensor value of the input type
  */
extern float getValue(CHECK_VALUE_TYPE value_type);

/**
  * @brief  Checks if a warning is set and if the warning is triggered
  * @param  value that needs to be checked
  * @retval if the warning is triggered
  */
extern bool isTriggered(float value);

#endif /* INC_HTU21D_H_ */
