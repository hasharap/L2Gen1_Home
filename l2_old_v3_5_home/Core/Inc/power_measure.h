/*
 * power_measure.h
 *
 *  Created on: Sep 7, 2022
 *      Author: Nisal Bulathsinghala
 */

#ifndef INC_POWER_MEASURE_H_
#define INC_POWER_MEASURE_H_

#include "main.h"
//#include "power_meas_sine_analyzer.h"
//
//extern POWER_MEAS_SINE_ANALYZER power_meas_in;
//extern POWER_MEAS_SINE_ANALYZER power_meas_out;

extern volatile bool measure_energy;
extern volatile uint32_t start_energy;

extern volatile uint16_t energy_error_count;

void calculate_Energy(void);
void get_EnergyMeterData(void);

#endif /* INC_POWER_MEASURE_H_ */
