/**
 ******************************************************************************
 * @file           : control_pilot.c
 * @brief          : Control Pilot Functions
 ******************************************************************************
 * @attention
 *
 * Author: Nisal Bulathsinghala, Jul 27, 2022
 * Copyright (c) 2022 Vega Innovations, Sri Lanka.
 * All rights reserved.
 ******************************************************************************
 */

#include "control_pilot.h"

#define OLD_CP		0

uint16_t stateA_Vmin, stateA_Vmax;
uint16_t stateB_Vmin, stateB_Vmax;
uint16_t stateC_Vmin, stateC_Vmax;
uint16_t stateD_Vmin, stateD_Vmax;
uint16_t stateE_Vmin, stateE_Vmax;
uint16_t stateF_Vmin, stateF_Vmax;

int min_count = 0;

#define samples     50
#define samples2    25
#define samples3    5

uint32_t adc_min = 4000;
uint16_t adc_min_buf[samples] = { 0 };

uint16_t adc_max = 0;
uint16_t adc_max_buf[samples] = { 0 };

uint32_t cp_min_sum = 0;
uint16_t cp_min_avg = 0;
uint32_t cp_max_sum = 0;
uint16_t cp_max_avg = 0;

uint32_t cp_min_count = 0;
uint32_t cp_max_count = 0;
uint32_t cp_min_count2 = 0;
uint32_t cp_max_count2 = 0;

uint16_t adc_high[samples] = {};
uint16_t adc_low[samples] = {};

bool dataCollected1 = false;
bool dataCollected2 = false;

bool max_done = false;
bool min_done = false;

uint16_t adcSample_buf1[samples];
uint16_t adcSample_buf2[samples];
uint16_t adcSample_buf1_count = 0;
uint16_t adcSample_buf2_count = 0;

uint16_t temp_adc = 0;

const float ADC_supply_val = 3.3;
float ADC_supply_calibrate = 0.0;

float actual_CP = 0.0;

uint16_t vehicleCheck_adc = 0;
uint16_t vehicleCheck_avg = 0;

uint32_t diodeCheck_adc = 0;
uint16_t diodeCheck_avg = 0;
uint16_t diodeCheck_count = 0;
volatile bool diodeCheck_flag = false;
volatile bool diodeCheck_passed = false;

#define PWMActive_count	(50 / PROTECTION_TICK_TIME)
uint16_t PWMActive_InCounter = 0;
uint16_t PWMActive_OutCounter = 0;

#define CPFault_THRESHOLD	200
//#define CPFault_errorCount	50
#define CPFault_errorCount	(500 / PROTECTION_TICK_TIME)	//1s
uint16_t CPFault_errorInCounter = 0;
uint16_t CPFault_errorOutCounter = 0;

float CP_A_V = 12.00, CP_B_V = 9.00, CP_C_V = 6.00; //Relative voltage values
float V_upper = 11.97, V_zero = 0.105, V_lower = -12.20; //Real voltage values
float CP_voltage_val = 0; //To get the rela voltage from the ADC value

uint16_t CP_max_val =0, CP_min_val=0, CP_zero_val=0; //maximum CP reading value = CP_max, Minimum CP reading value = CP_min
bool vehile_pluged = false; //To set a flag for calibration

bool cp_readings_done = false;

uint16_t dataToWrite[8] = {0};
uint16_t saveCP[3] = {0};
uint16_t readCP[3] = {0};

#define DC_errorCount	50
uint8_t DC_errorInCounter = 0;
uint8_t DC_errorOutCounter = 0;

#define vehicle_checkCount 25
uint8_t vehicle_checkInCounter = 0;
uint8_t vehicle_checkOutCounter = 0;

void readCP_stored(void)
{
	EEPROM_ReadData(CP_LOCATION, readCP, sizeof(readCP)/sizeof(readCP[0]));

	cpdata.cpmax = (uint16_t)readCP[0];
	cpdata.cpzero = (uint16_t)readCP[1];
	cpdata.cpmin = (uint16_t)readCP[2];
}

void saveCP_store(void)
{
	saveCP[0] = cpdata.cpmax;
	saveCP[1] = cpdata.cpzero;
	saveCP[2] = cpdata.cpmin;

	EEPROM_Erase(CP_LOCATION, EEPROM_SIZE);
	EEPROM_WriteData(CP_LOCATION, saveCP, (sizeof(saveCP)/sizeof(saveCP[0])));
}

void cp_map(bool cp_calib)
{
	if (cp_calib == true)
	{ //If calibration done then this will execute
#if OLD_CP
		uint16_t y1 = ((cpdata.cpmax - cpdata.cpzero) /*+ (cpdata.cpzero - cpdata.cpmin)*/);
//		y1 = y1 / 2;
		//uint16_t y3 = (cpcalibration.cpzero - cpcalibration.cpmin);
		uint16_t val1 = 0;
		float hist_val = 0.5;

		float A_V1 = ((CP_A_V - V_zero) / (V_upper - V_zero));
		float B_V1 = ((CP_B_V - V_zero) / (V_upper - V_zero));
		float C_V1 = ((CP_C_V - V_zero) / (V_upper - V_zero));
		float F_V1 = ((-12 - V_zero) / (V_upper - V_zero));
		float hist_add = ((hist_val / V_upper) * y1);

		//float A_V2 = (A-V_lower)/(V_zero-V_lower), B_V2=(B-V_lower)/(V_zero-V_lower), C_V2=(C-V_lower)/(V_zero-V_lower);

		val1 = (uint16_t)((A_V1 * y1) + cpdata.cpzero);
		stateA_Vmax = (uint16_t)(val1 + hist_add);
		dataToWrite[0] = stateA_Vmax;
		stateA_Vmin = (uint16_t)(val1 - hist_add);
		dataToWrite[1] = stateA_Vmin;
		val1 = 0;

		val1 = (uint16_t)((B_V1 * y1) + cpdata.cpzero);
		stateB_Vmax = (uint16_t)(val1 + (hist_add * 1.2));
		dataToWrite[2] = stateB_Vmax;
		stateB_Vmin = (uint16_t)(val1 - (hist_add * 0.8));
		dataToWrite[3] = stateB_Vmin;
		val1 = 0;

		val1 = (uint16_t)((C_V1 * y1) + cpdata.cpzero);
		stateC_Vmax = (uint16_t)(val1 + hist_add);
		dataToWrite[4] = stateC_Vmax;
		stateC_Vmin = (uint16_t)(val1 - (hist_add * 2.0));
		dataToWrite[5] = stateC_Vmin;
		val1 = 0;

		val1 = (uint16_t)((F_V1 * y1) + cpdata.cpzero);
		stateF_Vmax = (uint16_t)(val1 + (hist_add * 1.8));
		dataToWrite[6] = stateF_Vmax;
		stateF_Vmin = (uint16_t)(val1 - hist_add);
		dataToWrite[7] = stateF_Vmin;
#else
//		uint16_t y1 = ((cpdata.cpmax - cpdata.cpzero) /*+ (cpdata.cpzero - cpdata.cpmin)*/);
//		uint16_t val1 = 0;
//		float hist_val = 0.5;
//
//		float A_V1 = ((CP_A_V - V_zero) / (V_upper - V_zero));
//		float B_V1 = ((CP_B_V - V_zero) / (V_upper - V_zero));
//		float C_V1 = ((CP_C_V - V_zero) / (V_upper - V_zero));
//		float F_V1 = ((-12 - V_zero) / (V_upper - V_zero));
//		float hist_add = ((hist_val / V_upper) * y1);
//
//		val1 = (uint16_t)((A_V1 * y1) + cpdata.cpzero);
//		stateA_Vmax = (uint16_t)(val1 + (hist_add * 1.2));
//		dataToWrite[0] = stateA_Vmax;
//		stateA_Vmin = (uint16_t)(val1 - (hist_add * 1.5));
//		dataToWrite[1] = stateA_Vmin;
//		val1 = 0;
//
//		val1 = (uint16_t)((B_V1 * y1) + cpdata.cpzero);
//		stateB_Vmax = (uint16_t)(val1 + (hist_add * 1.0));
//		dataToWrite[2] = stateB_Vmax;
//		stateB_Vmin = (uint16_t)(val1 - (hist_add * 2.0));
//		dataToWrite[3] = stateB_Vmin;
//		val1 = 0;
//
//		val1 = (uint16_t)((C_V1 * y1) + cpdata.cpzero);
//		stateC_Vmax = (uint16_t)(val1 + hist_add);
//		dataToWrite[4] = stateC_Vmax;
//		stateC_Vmin = (uint16_t)(val1 - (hist_add * 4.0));
//		dataToWrite[5] = stateC_Vmin;
//		val1 = 0;
//
//		val1 = (uint16_t)((F_V1 * y1) + cpdata.cpzero);
//		stateF_Vmax = (uint16_t)(val1 + (hist_add * 3.0));
//		dataToWrite[6] = stateF_Vmax;
//		stateF_Vmin = (uint16_t)(val1 - hist_add * 1.0);
//		dataToWrite[7] = stateF_Vmin;

//		stateA_Vmax = (uint16_t)(val1 + (hist_add * 1.2));
//		stateA_Vmin = (uint16_t)(val1 - (hist_add * 1.5));
//
//		stateB_Vmax = (uint16_t)(val1 + (hist_add * 1.0));
//		stateB_Vmin = (uint16_t)(val1 - (hist_add * 2.0));
//
//		stateC_Vmax = (uint16_t)(val1 + hist_add);
//		stateC_Vmin = (uint16_t)(val1 - (hist_add * 4.0));
//
//		stateF_Vmax = (uint16_t)(val1 + (hist_add * 3.0));
//		stateF_Vmin = (uint16_t)(val1 - hist_add * 1.0);

		stateA_Vmax = 4000;
		stateA_Vmin = 3500;

		stateB_Vmax = 3400;
		stateB_Vmin = 3000;

		stateC_Vmax = 2900;
		stateC_Vmin = 2400;

		stateF_Vmax = 600;
		stateF_Vmin = 100;
#endif
		//EEPROM_Erase(EEROM_START_ADDRESS, EEPROM_SIZE);
		//HAL_Delay(1000);
		//EEPROM_WriteData(EEROM_START_ADDRESS,dataToWrite, 8);
		//HAL_Delay(1000);
		//EEPROM_ReadData(EEROM_START_ADDRESS, readdata, EEPROM_SIZE);
	}
}

uint16_t calculateCMA(uint16_t current_value, uint16_t previous_cma, int count) {
    return (previous_cma * count + current_value) / (count + 1);
}

void getCP_voltage(void)
{
	temp_adc = adc_store[MCU_CP_READ_INDEX];

	//V3
	if (dataCollected1 == false)
	{
		adcSample_buf1[adcSample_buf1_count] = temp_adc;
		adc_high[adcSample_buf1_count] = 0;
		adc_low[adcSample_buf1_count] = 0;

		adcSample_buf1_count++;

		if (adcSample_buf1_count >= samples)
		{
			adcSample_buf1_count = 0;
			dataCollected1 = true;
		}
	}
	else
	{
		if (dataCollected2 == false)
		{
			if (adcSample_buf1[adcSample_buf1_count] >= 1800)
			{
				if (cp_max_count < samples)
				{
					adc_high[cp_max_count] =
							adcSample_buf1[adcSample_buf1_count];
					cp_max_count++;
				}
			}
			else
			{
				if (cp_min_count < samples)
				{
					adc_low[cp_min_count] =
							adcSample_buf1[adcSample_buf1_count];
					cp_min_count++;
				}
			}

			adcSample_buf1_count++;

			if (adcSample_buf1_count >= samples)
			{
				adcSample_buf1_count = 0;
				dataCollected2 = true;
			}
		}
		else
		{
			if (cp_max_count > 0)
			{
				if (cp_max_count2 < cp_max_count)
				{
					cp_max_sum = cp_max_sum + adc_high[cp_max_count2];
					cp_max_count2++;
				}
				else
				{
					cp_max_avg = (uint16_t) (cp_max_sum / cp_max_count);

//					controlSide_data.controlPilot.cp_max = cp_max_avg;

					controlSide_data.controlPilot.cp_max = (0.2 * cp_max_avg)
							+ ((1 - 0.2) * controlSide_data.controlPilot.cp_max);

					float temp2 =
							((7.6807
									* ((float) (VREF / 4096.0)
											* (float) controlSide_data.controlPilot.cp_max))
									+ 0 - (3.6707 * VREF));
					controlSide_data.controlPilot.cp_Vmax =
							(int) (temp2 * 10.0);

					max_done = true;
				}
			}
			else
			{
				max_done = true;
			}

			if (cp_min_count > 0)
			{
				if (cp_min_count2 < cp_min_count)
				{
					cp_min_sum = cp_min_sum + adc_low[cp_min_count2];
					cp_min_count2++;
				}
				else
				{
					cp_min_avg = (uint16_t) (cp_min_sum / cp_min_count);

//					controlSide_data.controlPilot.cp_min = cp_min_avg;

					controlSide_data.controlPilot.cp_min = (0.2 * cp_min_avg)
							+ ((1 - 0.2) * controlSide_data.controlPilot.cp_min);

					float temp1 =
							((7.6807
									* ((float) (VREF / 4096.0)
											* (float) controlSide_data.controlPilot.cp_min))
									- 1.005 - (3.6707 * VREF));
					controlSide_data.controlPilot.cp_Vmin =
							(int) (temp1 * 10.0);

					min_done = true;
				}
			}
			else
			{
				min_done = true;
			}

			if ((max_done == true) && (min_done == true))
			{
				adcSample_buf1_count = 0;
				cp_max_count = 0;
				cp_min_count = 0;
				cp_max_count2 = 0;
				cp_min_count2 = 0;
				max_done = false;
				min_done = false;

				cp_min_sum = 0;
				cp_min_avg = 0;
				cp_max_sum = 0;
				cp_max_avg = 0;

				adc_min = 4000;
				adc_max = 0;

				dataCollected1 = false;
				dataCollected2 = false;
			}
		}
	}

//	if (dataCollected1 == false)
//	{
//		adc_min_buf[min_count] = temp_adc;
//		adc_max_buf[min_count] = temp_adc;
//		min_count++;
//
//		if (min_count == samples)
//		{
//			dataCollected1 = true;
//			min_count = 0;
//			adc_min = adc_min_buf[0];
//			adc_max = adc_max_buf[0];
//		}
//	}
//	else
//	{
//		if (adc_min_buf[a] < adc_min)
//		{
//			adc_min = adc_min_buf[a];
//		}
//		if (adc_max_buf[a] >= adc_max)
//		{
//			adc_max = adc_max_buf[a];
//		}
//
//		a++;
//
//		if (a >= samples)
//		{
//			a = 0;
//			dataCollected2 = true;
//		}
//	}
//
//	if (dataCollected2 == true)
//	{
//		adc_min_buf2[min_count2] = adc_min;
//		adc_max_buf2[min_count2] = adc_max;
//		min_count2++;
//
//		if (min_count2 >= samples2)
//		{
//			min_count2 = 0;
//			dataCollected3 = true;
//			adc_min2 = adc_min_buf2[0];
//			adc_max2 = adc_max_buf2[0];
//		}
//		else
//		{
//			a = 0;
//			dataCollected1 = false;
//			dataCollected2 = false;
//		}
//	}
//
//	if (dataCollected3 == true)
//	{
//		if (adc_min_buf2[a] < adc_min2)
//		{
//			adc_min2 = adc_min_buf2[a];
//		}
//		if (adc_max_buf2[a] >= adc_max2)
//		{
//			adc_max2 = adc_max_buf2[a];
//		}
//
//		a++;
//
//		if (a >= samples2)
//		{
//			a = 0;
//			dataCollected1 = false;
//			dataCollected2 = false;
//			dataCollected3 = false;
//
//			cp_min_sum = (cp_min_sum + adc_max2);
//			b++;
//			if (b >= samples3)
//			{
//				b = 0;
//				cp_min_avg = (cp_min_sum / samples3);
//
//				controlSide_data.controlPilot.cp_min = (uint16_t) (cp_min_avg);
//				controlSide_data.controlPilot.cp_Vmin = (float) ((3.3 / 4096.0)
//						* (float) controlSide_data.controlPilot.cp_min);
//
//				controlSide_data.controlPilot.cp_max = (uint16_t) (adc_min2);
//				controlSide_data.controlPilot.cp_Vmax = (float) ((3.3 / 4096.0)
//						* (float) controlSide_data.controlPilot.cp_max);
//
//				cp_min_sum = 0;
//				cp_min_avg = 0;
//			}
//		}
//	}
}

void calibrateCP(int action)
{
	readCP_stored();

	//-------------------------------------------+12V genaration
	TIM1->CCR1 = (uint32_t) (PWMFULLON * 10);
	TIM1->CCR4 = (uint32_t) (PWMFULLON * 10);

	cp_readings_done = false;

	HAL_Delay(1000);

	vehile_pluged = (controlSide_data.controlPilot.cp_max <= 3400); //Condition should be change
//	vehile_pluged = (controlSide_data.controlPilot.cp_max <= 3000);

	if (vehile_pluged == 0)
	{
#if OLD_CP
		TIM1->CCR1 = (uint32_t) (PWMFULLON * 10); //+12V genaration
		TIM1->CCR4 = (uint32_t) (PWMFULLON * 10);
		HAL_Delay(100); //Give some time to settle
		cp_readings_done = false;
		HAL_Delay(delay_2);

		CP_max_val = (controlSide_data.controlPilot.cp_min
				+ controlSide_data.controlPilot.cp_max); //Take the average value
		CP_max_val = (uint16_t) (CP_max_val / 2);
		cpdata.cpmax = CP_max_val;
		//cpcalibration.cpmax = CP_max_val;

		TIM1->CCR1 = (uint32_t) (PWMFULLOFF * 10); //-12V genaration
		TIM1->CCR4 = (uint32_t) (PWMFULLOFF * 10);
		HAL_Delay(100); //Give some time to settle
		cp_readings_done = false;
		HAL_Delay(delay_2);

		CP_min_val = (controlSide_data.controlPilot.cp_min
				/*+ controlSide_data.controlPilot.cp_max*/);
//		CP_min_val = (uint16_t) (CP_min_val / 2);
		cpdata.cpmin = CP_min_val;
		//cpcalibration.cpmin = CP_min_val;

		TIM1->CCR1 = (uint32_t) (PWMFULLON * 10); //0V genaration
		TIM1->CCR4 = (uint32_t) (PWMFULLOFF * 10);
		HAL_Delay(100); //Give some time to settle
		cp_readings_done = false;
		HAL_Delay(delay_2);

		CP_zero_val = (controlSide_data.controlPilot.cp_min
				+ controlSide_data.controlPilot.cp_max);
		CP_zero_val = (uint16_t) (CP_zero_val / 2);
		cpdata.cpzero = CP_zero_val;
		//cpcalibration.cpzero=CP_zero_val;
		saveCP_store();

		TIM1->CCR1 = (uint32_t) (PWMFULLON * 10); //-12V genaration
		TIM1->CCR4 = (uint32_t) (PWMFULLON * 10);
		HAL_Delay(500); //Give some time to settle
#else
		uint16_t CP_val2 = 0;

		//-------------------------------------------(+12V) Generation
		TIM1->CCR1 = (uint32_t) (PWMFULLON * 10);
		TIM1->CCR4 = (uint32_t) (PWMFULLON * 10);
		HAL_Delay(1000); //Give some time to settle

		CP_val2 = (controlSide_data.controlPilot.cp_max
				+ controlSide_data.controlPilot.cp_min); //Take the average value
		cpdata.cpmax = (CP_val2 / 2);

		//-------------------------------------------(-12V) Generation
		TIM1->CCR1 = (uint32_t) (PWMFULLOFF * 10);
		TIM1->CCR4 = (uint32_t) (PWMFULLOFF * 10);
		HAL_Delay(2000);

		CP_val2 = (controlSide_data.controlPilot.cp_max
				+ controlSide_data.controlPilot.cp_min);
		cpdata.cpmin = (CP_val2 / 2);

		//-------------------------------------------(0V) Calculation
		cpdata.cpzero = ((cpdata.cpmax) / 2); //To get the zeroth position

		saveCP_store();

		//-------------------------------------------(+12V) Generation
		TIM1->CCR1 = (uint32_t) (PWMFULLON * 10);
		TIM1->CCR4 = (uint32_t) (PWMFULLON * 10);
		HAL_Delay(1000); //Give some time to settle
#endif
	}

	/*
	 else{
	 buzzer_on();
	 buzzer_cp_error();
	 HAL_Delay(1000);
	 HAL_NVIC_SystemReset(); //If charger is not calibrated then the system will not work.
	 }
	 */
}

static uint8_t uv_handle_state = 0;
static uint8_t uv_stable_count = 0;
static uint16_t lowest_voltage = 0;

static inline uint16_t find_lowest(uint16_t a, uint16_t b, uint16_t c)
{
	uint16_t lowest = a; // Assume a is the lowest initially

	if (b < lowest)
	{
		lowest = b;
	}

	if (c < lowest)
	{
		lowest = c;
	}

	return lowest;
}

void load_balance(bool uv_en)
{
	if (load_balance6s.timeout_6s == false)
	{
		return;
	}

	tick_clear(&load_balance6s);

#if UV_dynamic
	if (controlSide_data.warnings.bits.UV_warn == 1)
	{
		switch (uv_handle_state)
		{
		case 0:
			if (powerSide_data.errorStatus.bit.error_UV == 1)
			{
				controlSide_data.controlPilot.PWMSET = 15;
				uv_handle_state = 3;
				uv_stable_count = 0;
			}
			break;
		case 1:
			if (powerSide_data.errorStatus.bit.error_UV == 0)
			{
				controlSide_data.controlPilot.PWMSET =
						(controlSide_data.controlPilot.PWMSET + 5);
				if (controlSide_data.controlPilot.PWMSET >= CPMAX)
				{
					controlSide_data.controlPilot.PWMSET = CPMAX;
				}
				uv_handle_state = 3;
				uv_stable_count = 0;
			}
			break;
		case 2:
			controlSide_data.controlPilot.PWMSET =
					(controlSide_data.controlPilot.PWMSET - 5);
			if (controlSide_data.controlPilot.PWMSET <= CPMIN)
			{
				controlSide_data.controlPilot.PWMSET = CPMIN;
			}
			uv_handle_state = 3;
			uv_stable_count = 0;
			break;
		case 3:
			lowest_voltage = find_lowest(powerSide_data.voltage.VA,
					powerSide_data.voltage.VB, powerSide_data.voltage.VC);

			if ((lowest_voltage - UV_UPPER_THRESHOLD) > 0)
			{
				if ((lowest_voltage - UV_UPPER_THRESHOLD) <= 50)
				{
					uv_handle_state = 3;
					uv_stable_count++;

					if (uv_stable_count >= 10)
					{
						controlSide_data.warnings.bits.UV_warn = 0;
					}
				}
				else
				{
					uv_handle_state = 1;
				}
			}
			else
			{
				uv_handle_state = 2;
			}
			break;
		default:
			break;
		}
		return;
	}
#endif

	controlSide_data.controlPilot.PWMSET = CPDUTY_DEFAULT;

//	if (controlSide_data.errorStatus.bit.serialAError == 1)
//	{
//		return;
//	}

	uint16_t temp = (uint16_t) (networkSide_data.maxCurrent_req / 0.6);

	if (temp >= CPMIN)
	{
		controlSide_data.controlPilot.PWMSET = temp;
	}
}

void monitor_cp(void)
{
//#if CP_FAULT_CHECK
//	if ((controlSide_data.controlPilot.cp_max <= (cpdata.cpzero + 200))
//			&& (currentState != STATE_F))
//	{
//		CPFault_errorOutCounter = 0;
//		CPFault_errorInCounter++;
//		if (CPFault_errorInCounter >= CPFault_errorCount)
//		{
//			controlSide_data.errorStatus.bit.CPFault = 1;
//		}
//	}
//	else
//	{
//		CPFault_errorInCounter = 0;
//		CPFault_errorOutCounter++;
//		if (CPFault_errorOutCounter >= CPFault_errorCount)
//		{
//			CPFault_errorOutCounter = 0;
//			controlSide_data.errorStatus.bit.CPFault = 0;
//		}
//	}
//#endif
	if (charger_configGet.en_1.bit.cpf_en == 1)
	{
		if ((controlSide_data.controlPilot.cp_Vmax > STATEF_MIN)
				&& (controlSide_data.controlPilot.cp_Vmax < STATEC_MIN))
		{
			CPFault_errorOutCounter = 0;
			CPFault_errorInCounter++;
			if (CPFault_errorInCounter >= CPFault_errorCount)
			{
				controlSide_data.errorStatus.bit.CPFault = 1;
			}
		}
		else
		{
			if (controlSide_data.status.bit.connector_state == 0)
			{
				CPFault_errorInCounter = 0;
				CPFault_errorOutCounter++;
				if (CPFault_errorOutCounter >= CPFault_errorCount)
				{
					CPFault_errorOutCounter = 0;
					controlSide_data.errorStatus.bit.CPFault = 0;
				}
			}
		}
	}

	if (controlSide_data.controlPilot.PWMSET < CPMIN)
	{
		cpMin_reached = true;
		return;
	}

	cpMin_reached = false;

	if (controlSide_data.controlPilot.PWMSET >= CPMAX)
	{
		controlSide_data.controlPilot.PWMSET = CPMAX;
	}
}

void setCP_duty(void)
{
//	if (currentState == STATE_F)
//	{
//		if (controlSide_data.status.bit.cpPWM_active == 1)
//		{
//			controlSide_data.controlPilot.cpDuty = PWMFULLON;
//		}
//		else
//		{
//			controlSide_data.controlPilot.cpDuty = PWMFULLOFF;
//		}
//	}
//	else
//	{
//		if (controlSide_data.status.bit.cpPWM_active == 1)
//		{
//			controlSide_data.controlPilot.cpDuty =
//					controlSide_data.controlPilot.PWMSET;
//		}
//		else
//		{
//			controlSide_data.controlPilot.cpDuty = PWMFULLON;
//		}
//	}
	if (currentState == STATE_F)
	{
		if (controlSide_data.controlPilot.cp_enable == 1)
		{
			controlSide_data.controlPilot.cpDuty = PWMFULLON;
		}
		else
		{
			controlSide_data.controlPilot.cpDuty = PWMFULLOFF;
		}
	}
	else
	{
		if (controlSide_data.controlPilot.cp_enable == 1)
		{
			if (controlSide_data.controlPilot.cp_duty_enable == 1)
			{
				controlSide_data.controlPilot.cpDuty =
						controlSide_data.controlPilot.PWMSET;
			}
			else
			{
				controlSide_data.controlPilot.cpDuty = PWMFULLON;
			}

		}
		else
		{
			controlSide_data.controlPilot.cpDuty = PWMFULLOFF;
		}
	}

	TIM1->CCR1 = (uint32_t) (controlSide_data.controlPilot.cpDuty * 10);
	TIM1->CCR4 = (uint32_t) (controlSide_data.controlPilot.cpDuty * 10);
}

void diodeCheck(void)
{
	if ((controlSide_data.controlPilot.cp_Vmin > STATEF_MIN)
			&& (currentState == STATE_B2)
			&& (controlSide_data.status.bit.cpPWM_active == 1))
	{
		DC_errorOutCounter = 0;
		DC_errorInCounter++;
		if (DC_errorInCounter >= DC_errorCount)
		{
			DC_errorInCounter = 0;
			controlSide_data.errorStatus.bit.diodeCheck_failed = 1;
			diodeCheck_passed = false;
		}
	}
	else
	{
		DC_errorInCounter = 0;
		DC_errorOutCounter++;
		if (DC_errorOutCounter >= DC_errorCount)
		{
			DC_errorOutCounter = 0;
			controlSide_data.errorStatus.bit.diodeCheck_failed = 0;
			diodeCheck_passed = true;
		}
	}
}

void bootup_vehicleCheck(void)
{
	if (controlSide_data.controlPilot.cp_Vmax > STATEA_MIN)
	{
		controlSide_data.networkSide_request.bit.vehicle_Check = 0;
	}
	else
	{
		controlSide_data.networkSide_request.bit.vehicle_Check = 1;

		if ((controlSide_data.controlPilot.cp_Vmax > STATEB_MIN) //Jump to STATE B1
		&& (controlSide_data.controlPilot.cp_Vmax < STATEB_MAX))
		{
			stateEntry_flag = true;
			currentState = STATE_B1;
		}

		if ((controlSide_data.controlPilot.cp_Vmax > STATEC_MIN) //Jump to STATE C1
		&& (controlSide_data.controlPilot.cp_Vmax < STATEC_MAX))
		{
			stateEntry_flag = true;
			currentState = STATE_C1;
		}
	}
}

void check_vehicleConnector(void)
{
	if (controlSide_data.controlPilot.cp_enable)
	{
		if ((controlSide_data.controlPilot.cp_Vmax > 0)
				&& (controlSide_data.controlPilot.cp_Vmax < STATEA_MIN))
		{
			vehicle_checkInCounter = 0;
			vehicle_checkOutCounter++;

			if (vehicle_checkOutCounter >= vehicle_checkCount)
			{
				controlSide_data.status.bit.connector_state = 0;
			}
		}
		else
		{
			vehicle_checkOutCounter = 0;
			vehicle_checkInCounter++;

			if (vehicle_checkInCounter >= vehicle_checkCount)
			{
				controlSide_data.status.bit.connector_state = 1;
			}
		}
	}
}

void check_PWMActive(void)
{
	if (charger_init_flag == true)
	{
		static uint16_t diff;

		if (controlSide_data.controlPilot.cp_duty_enable == 1)
		{
			diff = (controlSide_data.controlPilot.cp_max
					- controlSide_data.controlPilot.cp_min);

			if (diff > 1000)
			{
				PWMActive_OutCounter = 0;
				PWMActive_InCounter++;
				if (PWMActive_InCounter >= PWMActive_count)
				{
					PWMActive_InCounter = 0;
					controlSide_data.status.bit.cpPWM_active = 1;
				}
			}
			else
			{
				PWMActive_InCounter = 0;
				PWMActive_OutCounter++;
				if (PWMActive_OutCounter >= PWMActive_count)
				{
					PWMActive_OutCounter = 0;
					controlSide_data.status.bit.cpPWM_active = 0;
				}
			}
		}
		else
		{
			controlSide_data.status.bit.cpPWM_active = 0;
		}
	}
}
