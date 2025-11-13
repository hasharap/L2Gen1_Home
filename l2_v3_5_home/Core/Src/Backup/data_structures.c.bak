/*
 * data_structures.c
 *
 *  Created on: Jul 28, 2022
 *      Author: Nisal Bulathsinghala
 */

#include "data_structures.h"

volatile struct controlSide_data_struct controlSide_data;
volatile struct powerSide_data_struct powerSide_data;
volatile struct networkSide_data_struct networkSide_data;

volatile uint16_t adc_store[ADC_CHANNEL];
volatile uint32_t counter = 0;
volatile uint32_t counter2 = 0;

volatile float rms_vin = 0.0;
volatile float rms_vout = 0.0;

volatile bool cpMin_reached = false;

volatile bool networkSide_bootup = true;

volatile bool charger_init_flag = false;
volatile bool charger_setup_flag = false;

volatile struct cp_data_struct cpdata;

volatile float vrefint = 1.2;
volatile float VREF = 0.0;
volatile float VREF_SUM = 0.0;
volatile float VREF_AVG = 0.0;
volatile uint32_t vref_count = 0;
volatile float Temperature = 0.0;

void init_dataStructures(void)
{
	controlSide_data.controlPilot.cpDuty = PWMFULLON;
	controlSide_data.controlPilot.cp_Vmax = 0;
	controlSide_data.controlPilot.cp_Vmin = 3800;
	controlSide_data.controlPilot.cp_max = 0;
	controlSide_data.controlPilot.cp_min = 0;
	controlSide_data.controlPilot.cp_enable = 1;
	controlSide_data.controlPilot.cp_duty_enable = 0;
	controlSide_data.errorStatus.all = 0;
	controlSide_data.errorStatus.all = 0;
	controlSide_data.errorStatus.all = 0;
	controlSide_data.inputs.all = 0;
	controlSide_data.networkSide_request.all = 0;
	controlSide_data.outputs.all = 0;
	controlSide_data.powerSide_request.all = 0;
	controlSide_data.status.all = 0;
	controlSide_data.warnings.all = 0;
	controlSide_data.mcuTemp = 0.0;

	powerSide_data.voltage.VA = 0;
	powerSide_data.voltage.VB = 0;
	powerSide_data.voltage.VC = 0;
	powerSide_data.current.IA = 0;
	powerSide_data.current.IB = 0;
	powerSide_data.current.IC = 0;
	powerSide_data.powerEnergy.power = 0;
	powerSide_data.powerEnergy.kWh = 0;
	powerSide_data.frequency = 0;
	powerSide_data.errorStatus.all = 0;
	powerSide_data.status.all = 0;
	powerSide_data.tripStatus.all = 0;
	powerSide_data.tempSensors.T1 = 0.0;
	powerSide_data.tempSensors.T2 = 0.0;
	powerSide_data.tempSensors.T3 = 0.0;
	powerSide_data.tempSensors.T4 = 0.0;

	networkSide_data.status = 0;
	networkSide_data.maxCurrent_req = 53;
	networkSide_data.errors = 0;
	networkSide_data.ledCommnad = 0;
	networkSide_data.scheduleCharge = 0;
	networkSide_data.timeReady = 0;
	networkSide_data.chargerLock = 0;
	networkSide_data.isInternet_available = 0;
	networkSide_data.loadBalancing_en = 0;
	networkSide_data.ledOnOff_command = 0;
	networkSide_data.setTime.Hours = 0;
	networkSide_data.setTime.Minutes = 0;
	networkSide_data.weekdayOn.Hours = 0;
	networkSide_data.weekdayOff.Minutes = 0;
	networkSide_data.weekendOn.Hours = 0;
	networkSide_data.weekendOff.Minutes = 0;

	charger_configGet.en_1.all = 0;
	charger_configGet.en_2.all = 0;
	charger_configGet.uv_upper = 0;
	charger_configGet.uv_upper = 0;
	charger_configGet.ov_upper = 0;
	charger_configGet.ov_lower = 0;
	charger_configGet.freq_upper = 0;
	charger_configGet.freq_lower = 0;
	charger_configGet.max_current = 0;
	charger_configGet.config_counter = 0;
	charger_configGet.config_enable = false;

	charger_configSet.en_1.all = 0;
	charger_configSet.en_2.all = 0;
	charger_configSet.uv_upper = 0;
	charger_configSet.uv_upper = 0;
	charger_configSet.ov_upper = 0;
	charger_configSet.ov_lower = 0;
	charger_configSet.freq_upper = 0;
	charger_configSet.freq_lower = 0;
	charger_configSet.max_current = 0;
	charger_configSet.config_counter = 0;
	charger_configSet.config_enable = false;

	hw_version.v1 = 0;
	hw_version.v2 = 0;
	hw_version.v3 = 0;
}

volatile bool buzzer_en = false;

void buzzer_on(void)
{
	BUZZER_ON();
	HAL_Delay(60);
	BUZZER_OFF();
	HAL_Delay(60);
	BUZZER_ON();
	HAL_Delay(60);
	BUZZER_OFF();
	HAL_Delay(60);
}

void buzzer_poweron(void)
{
	BUZZER_ON();
	HAL_Delay(60);
	BUZZER_OFF();
	HAL_Delay(60);
	BUZZER_ON();
	HAL_Delay(60);
	BUZZER_OFF();
	HAL_Delay(60);
	BUZZER_ON();
	HAL_Delay(60);
	BUZZER_OFF();
	HAL_Delay(60);
	BUZZER_ON();
	HAL_Delay(60);
	BUZZER_OFF();
	HAL_Delay(60);
	BUZZER_ON();
	HAL_Delay(60);
	BUZZER_OFF();
	HAL_Delay(60);
}
