/*
 * temp_NTC.c
 *
 *  Created on: Sep 11, 2023
 *      Author: bulbu
 */
#include "temp_NTC.h"
#include "math.h"

const float R1 = 2000.0;
const float R0 = 10000.0;     // Resistance at 25°C in ohms
const float T0 = 25.0;        // Temperature at R0 in Celsius
const float B = 3570.0;       // Beta value of the thermistor

void temperature_NTC(void)
{
	float voltage = (adc_store[NTC_INSIDE_INDEX] / 4096.0) * 3.3; // 5.0V is the Arduino reference voltage

	// Calculate the resistance of the NTC thermistor using the voltage divider formula
	float resistance = (R1 * voltage) / (3.4 - voltage);

	// Calculate the temperature using the Steinhart-Hart equation
	float steinhart = 1.0
			/ (1.0 / (T0 + 273.15) + (1.0 / B) * log(resistance / R0));
	powerSide_data.tempSensors.T1 = (steinhart - 273.15 - 4);
}
