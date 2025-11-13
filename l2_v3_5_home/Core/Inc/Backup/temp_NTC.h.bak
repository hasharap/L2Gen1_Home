/**
 * @file temp_NTC.h
 * @brief Header file for NTC thermistor temperature measurement.
 *
 * @details This file provides the declaration for functions and variables used
 *          to calculate the temperature using an NTC thermistor and the Steinhart-Hart equation.
 *
 * @date September 11, 2023
 * @version 1.0
 * @author
 * bulbu
 */

#ifndef INC_TEMP_NTC_H_
#define INC_TEMP_NTC_H_

#include "main.h"

/**
 * @brief Stores the temperature measured inside the system.
 */
extern uint16_t temp_inside;

/**
 * @brief Calculates the temperature using an NTC thermistor.
 *
 * @details This function reads the ADC value corresponding to the thermistor,
 *          calculates the resistance, and determines the temperature using the
 *          Steinhart-Hart equation. It applies a low-pass filter to smooth the output.
 */
void temperature_NTC(void);

#endif /* INC_TEMP_NTC_H_ */
