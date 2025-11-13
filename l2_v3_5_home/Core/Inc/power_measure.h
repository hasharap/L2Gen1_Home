/**
 * @file power_measure.h
 * @brief Header file for power and energy measurement functionality.
 *
 * @details This file declares the variables and functions used for calculating
 *          energy consumption and retrieving energy meter data.
 *
 * @date September 7, 2022
 * @version 1.0
 * @author
 * Nisal Bulathsinghala
 */

#ifndef INC_POWER_MEASURE_H_
#define INC_POWER_MEASURE_H_

#include "main.h"

/**
 * @brief Indicates whether energy measurement is active.
 */
extern volatile bool measure_energy;

/**
 * @brief Starting energy value for energy calculation.
 */
extern volatile uint32_t start_energy;

/**
 * @brief Counter for energy measurement errors.
 */
extern volatile uint16_t energy_error_count;

/**
 * @brief Calculates energy consumption.
 *
 * @details This function computes the energy consumed during a charging session
 *          based on the difference between the current energy reading and the
 *          starting energy value.
 */
void calculate_Energy(void);

/**
 * @brief Retrieves energy meter data and updates system parameters.
 *
 * @details Reads voltage, current, power, and frequency data from the energy
 *          meter and updates the relevant structures in the system.
 */
void get_EnergyMeterData(void);

#endif /* INC_POWER_MEASURE_H_ */
