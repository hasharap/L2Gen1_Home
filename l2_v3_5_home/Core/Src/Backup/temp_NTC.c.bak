/**
 * @file temp_NTC.c
 * @brief Implements temperature measurement using an NTC thermistor.
 *
 * @details This module calculates the temperature based on the resistance of an
 *          NTC thermistor using the Steinhart-Hart equation and a low-pass filter
 *          for smoothing.
 *
 * @date September 11, 2023
 * @version 1.0
 * @author
 * Nisal Bulathsinghalage
 */

#include "temp_NTC.h"
#include "math.h"

/** @brief Reference resistor value in the voltage divider (ohms). */
const float R1 = 2000.0;

/** @brief Resistance of the thermistor at 25°C (ohms). */
const float R0 = 10000.0;

/** @brief Temperature in Celsius corresponding to R0. */
const float T0 = 25.0;

/** @brief Beta value of the thermistor (Kelvin). */
const float B = 3570.0;

/** @brief Smoothing factor for the low-pass filter. */
#define ALPHA 0.05

/**
 * @brief Calculates the temperature using an NTC thermistor.
 *
 * @details Reads the ADC value corresponding to the NTC thermistor, calculates
 *          the resistance using a voltage divider formula, and then determines
 *          the temperature using the Steinhart-Hart equation. The calculated
 *          temperature is smoothed using a low-pass filter before updating the
 *          system data.
 */
void temperature_NTC(void)
{
    // Calculate the voltage from the ADC value (12-bit ADC, 3.4V reference).
    float voltage = (adc_store[NTC_INSIDE_INDEX] / 4096.0) * 3.4;

    // Calculate the resistance of the NTC thermistor using the voltage divider formula.
    float resistance = ((R1 * voltage) / (3.4 - voltage));

    // Calculate the temperature using the Steinhart-Hart equation.
    float steinhart = (1.0 / (1.0 / (T0 + 273.15) + (1.0 / B) * log(resistance / R0)));

    // Apply a low-pass filter to smooth the temperature reading.
    powerSide_data.tempSensors.T1 = (ALPHA * (steinhart - 273.15 - 4)) +
                                    ((1 - ALPHA) * powerSide_data.tempSensors.T1);
}
