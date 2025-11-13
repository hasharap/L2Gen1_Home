/*
 * power_measure.c
 *
 *  Created on: Sep 7, 2022
 *      Author: Nisal Bulathsinghala
 */

#include "power_measure.h"

uint16_t adc_max_c = 0;
uint16_t pre_adc_max_c = 0;
uint16_t adc_min_c = 0;
uint16_t pre_adc_min_c = 4096;
uint16_t energy_counter = 0;
uint32_t cal_counter = 0;
uint32_t adc_min_sum = 0;
uint32_t adc_max_sum = 0;
uint32_t av_adc_min_sum = 0;
uint32_t av_adc_max_sum = 0;
uint16_t zero_voltage_ref = 0;
uint16_t voltage_test = 0;
float power = 0;
float energy = 0;

//POWER_MEAS_SINE_ANALYZER power_meas_in;
//POWER_MEAS_SINE_ANALYZER power_meas_out;

//Nisal 25/02/2023
#define FILTER_SIZE 30
int values[FILTER_SIZE];
int valueIndex = 0;
int sum = 0;
int val = 0;
int avg = 0;

bool avg_init = false;

float alpha = 0.01; // smoothing factor (0 < alpha < 1)
float smoothedValue = 0;

volatile bool measure_energy = false;
volatile uint32_t start_energy = 0;
uint32_t start_energy2 = 0;

uint8_t modbusData_counter = 0;

volatile uint16_t energy_error_count = 0;

void calculate_Energy(void)
{
	static uint32_t temp;

    if ((currentState == STATE_C2) &&
        (controlSide_data.status.bit.charging_active == 1))
    {
        if (measure_energy == true)
        {
            powerSide_data.powerEnergy.kWh = 0;
            start_energy = (uint32_t)ADL.data.activeEnergy;

            if ((start_energy == ADL.data.activeEnergy) && (start_energy != 0))
            {
                measure_energy = false;
            }
            else
            {
                energy_error_count++;
            }
        }
        else
        {
            if (start_energy != 0)
            {
                temp = (uint32_t)(ADL.data.activeEnergy - start_energy);

                if (temp >= (uint32_t)ADL.data.activeEnergy)
                {
                    // Handle overflow or invalid energy reading.
                }
                else
                {
                    powerSide_data.powerEnergy.kWh = temp;
                }
            }
            else
            {
                energy_error_count++;
            }
        }
    }
}

void get_EnergyMeterData(void)
{
	powerSide_data.voltage.VA = (uint16_t) ADL.data.voltage_PhaseA;
	powerSide_data.voltage.VB = (uint16_t) ADL.data.voltage_PhaseB;
	powerSide_data.voltage.VC = (uint16_t) ADL.data.voltage_PhaseC;
	powerSide_data.current.IA = (uint16_t) ADL.data.current_PhaseA;
	powerSide_data.current.IB = (uint16_t) ADL.data.current_PhaseB;
	powerSide_data.current.IC = (uint16_t) ADL.data.current_PhaseC;
	powerSide_data.powerEnergy.power = (uint8_t) ((ADL.data.activePower) / 100);
	powerSide_data.frequency = (uint16_t) (ADL.data.freqeuncy / 100);
}
