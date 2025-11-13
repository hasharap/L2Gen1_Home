/*
 * data_structures.c
 *
 *  Created on: Jul 28, 2022
 *      Author: Nisal Bulathsinghala
 */

#include "data_structures.h"

volatile StateType currentState = STATE_A1;
volatile StateType previousState = STATE_A1;
volatile bool stateEntry_flag = true;

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

volatile struct charger_config_struct charger_configGet;
volatile struct charger_config_struct charger_configSet;
volatile uint16_t config_get[9] = {};
volatile uint16_t config_set[9] = {};

volatile struct hw_version_struct hw_version;
volatile uint16_t hw_verion_get[2] = {};

volatile float vrefint = 1.2;
volatile float VREF = 0.0;
volatile float VREF_SUM = 0.0;
volatile float VREF_AVG = 0.0;
volatile uint8_t vref_count = 0;
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

void get_chargerConfig(void)
{
	EEPROM_ReadData(CONFIG_LOCATION, (uint16_t *)config_get, sizeof(config_get)/sizeof(config_get[0]));
	HAL_Delay(100);

	charger_configGet.en_1.all = config_get[0];
	charger_configGet.en_2.all = config_get[1];
	charger_configGet.uv_upper = config_get[2];
	charger_configGet.uv_lower = config_get[3];
	charger_configGet.ov_upper = config_get[4];
	charger_configGet.ov_lower = config_get[5];
	charger_configGet.freq_upper = config_get[6];
	charger_configGet.freq_lower = config_get[7];
	charger_configGet.max_current = (config_get[8] & 0xFF);
}

void set_chargerConfig(void)
{
	config_set[0] = charger_configSet.en_1.all;
	config_set[1] = charger_configSet.en_2.all;
	config_set[2] = charger_configSet.uv_upper;
	config_set[3] = charger_configSet.uv_lower;
	config_set[4] = charger_configSet.ov_upper;
	config_set[5] = charger_configSet.ov_lower;
	config_set[6] = charger_configSet.freq_upper;
	config_set[7] = charger_configSet.freq_lower;
	config_set[8] = (uint16_t)charger_configSet.max_current;

	EEPROM_Erase(CONFIG_LOCATION, EEPROM_SIZE);
	EEPROM_WriteData(CONFIG_LOCATION, (uint16_t *)config_set, sizeof(config_get)/sizeof(config_get[0]));
}

void get_HWVersion(void)
{
	EEPROM_ReadData(HW_VERSION_LOCATION, (uint16_t *)hw_verion_get, sizeof(hw_verion_get)/sizeof(hw_verion_get[0]));
	HAL_Delay(100);
	hw_version.v1 = (uint8_t)(hw_verion_get[0] & 0xFF);
	hw_version.v2 = (uint8_t)(hw_verion_get[0] >> 8);
	hw_version.v3 = (uint8_t)(hw_verion_get[1] & 0xFF);
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
