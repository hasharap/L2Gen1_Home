/**
 * @file power_measure.c
 * @brief Implements power and energy measurement functionality.
 *
 * @details This module calculates energy consumption and retrieves energy meter
 *          data such as voltage, current, power, and frequency.
 *
 * @date September 7, 2022
 * @version 1.0
 * @author
 * Nisal Bulathsinghala
 */

#include "power_measure.h"

/** @brief Stores the measured power value. */
float power = 0;

/** @brief Stores the measured energy value in kWh. */
float energy = 0;

/** @brief Indicates whether energy measurement is active. */
volatile bool measure_energy = false;

/** @brief Starting energy value for energy calculation. */
volatile uint32_t start_energy = 0;

/** @brief Secondary starting energy value for alternate logic. */
uint32_t start_energy2 = 0;

/** @brief Counter for Modbus data. */
uint8_t modbusData_counter = 0;

/** @brief Counter for energy measurement errors. */
volatile uint16_t energy_error_count = 0;

/**
 * @brief Calculates energy consumption.
 *
 * @details This function calculates the energy consumed during the charging
 *          process by comparing the current energy reading with the starting
 *          value. Updates the `powerSide_data.powerEnergy.kWh` with the result.
 */
void calculate_Energy(void)
{
    static uint32_t temp;

    if ((sm.currentState == STATE_C2) &&
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

/**
 * @brief Retrieves energy meter data and updates system parameters.
 *
 * @details Reads voltage, current, power, and frequency data from the energy meter.
 *          Updates the `powerSide_data` structure with these parameters.
 */
void get_EnergyMeterData(void)
{
    powerSide_data.voltage.VA = (uint16_t)ADL.data.voltage_PhaseA;
    powerSide_data.voltage.VB = (uint16_t)ADL.data.voltage_PhaseB;
    powerSide_data.voltage.VC = (uint16_t)ADL.data.voltage_PhaseC;
    powerSide_data.current.IA = (uint16_t)ADL.data.current_PhaseA;
    powerSide_data.current.IB = (uint16_t)ADL.data.current_PhaseB;
    powerSide_data.current.IC = (uint16_t)ADL.data.current_PhaseC;
    powerSide_data.powerEnergy.power = (uint8_t)((ADL.data.activePower) / 100);
    powerSide_data.frequency = (uint16_t)(ADL.data.freqeuncy / 100);
}
