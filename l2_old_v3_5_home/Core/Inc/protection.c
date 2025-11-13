/**
 ******************************************************************************
 * @file           : protection.c
 * @brief          : Protection Functions
 ******************************************************************************
 * @attention
 *
 * Author: Nisal Bulathsinghala, Nov 21, 2022
 * Copyright (c) 2022 Vega Innovations, Sri Lanka.
 * All rights reserved.
 ******************************************************************************
 */

#include "protection.h"

bool stuck_relay_live = false;
bool stuck_relay_nutral = false;
bool stuck_rleay_one_time = true;
bool gfic_test_one_time = true;
bool stuck_relay_test = false;
bool gfic_test_run = false;
bool gfic_reset = false;
bool ocp_reset = false;

volatile bool run_GFITest = false;
uint8_t GFCI_errorCounter = 0;
static bool GFI_triggered = false;

#define GFI_errorCount	300
uint16_t GFI_errorInCounter = 0;
uint16_t GFI_errorOutCounter = 0;

#define OC_THRESHOLD	3500
#define OC_errorCount	(1000 / PROTECTION_TICK_TIME)	//1s
uint16_t OC_errorInCounter = 0;
uint16_t OC_errorOutCounter = 0;

#define UV_LOWER_THRESHOLD	2000
#define UV_UPPER_THRESHOLD	2050
#define UV_errorCount	(5000 / PROTECTION_TICK_TIME)	//2s
uint16_t UV_errorInCounter = 0;
uint16_t UV_errorOutCounter = 0;

#define OV_LOWER_THRESHOLD	2500
#define OV_UPPER_THRESHOLD	2600
#define OV_errorCount	(2000 / PROTECTION_TICK_TIME)	//2s
uint16_t OV_errorInCounter = 0;
uint16_t OV_errorOutCounter = 0;

#define PL_errorCount	(2000 / PROTECTION_TICK_TIME)	//2s
uint16_t PL_errorInCounter = 0;
uint16_t PL_errorOutCounter = 0;

#define FREQ_LOWER_THRESHOLD	4940
#define FREQ_UPPER_THRESHOLD	5060
#define Freq_errorCount	(2000 / PROTECTION_TICK_TIME)	//2s
#define FREQ_PERIOD		1000
uint16_t FREQ_period = 0;
uint16_t Freq_errorInCounter = 0;
uint16_t Freq_errorOutCounter = 0;

#define SC_errorCount	(1000 / PROTECTION_TICK_TIME)	//1s
uint16_t SC_errorInCounter = 0;
uint16_t SC_errorOutCounter = 0;

#define OT_LOWER_THRESHOLD	55
#define OT_UPPER_THRESHOLD	60
#define OT_errorCount	(20000 / PROTECTION_TICK_TIME)	//10s
uint16_t OT_errorInCounter = 0;
uint16_t OT_errorOutCounter = 0;

#define SERIAL_TIMEOUT serial_error.timeout_10m

uint8_t pro_1 = 0;
uint8_t pro_2 = 0;

volatile uint8_t error_buffer[ERROR_BUFFER_SIZE] =
{ 0 };

uint8_t currentError = 0;
uint8_t previousError = 0;
uint8_t errorLog_count = 0;

volatile bool GFI_bypasss_flag = false;

static void check_GFITest(void)
{
	if (run_GFITest == true)
	{
		static uint8_t state = 0;

		switch (state)
		{
		case 0:
			tick_clear(&gfi_test);
			GFIC_TEST_ON();
			state = 1;
			break;
		case 1:
			if (gfi_test.timeout_0_5s == true)
			{
				if (GFIC_TEST_SENSE() == 1)
				{
					powerSide_data.errorStatus.bit.error_GFI_test = 0;
				}
				else
				{
					powerSide_data.errorStatus.bit.error_GFI_test = 1;
				}

				GFIC_TEST_OFF();
				run_GFITest = false;
				state = 0;
			}
			break;
		default:
			break;
		}
	}
}

/*---------------------------------------------------
 	 Check GFI Trip
 ---------------------------------------------------*/
static void check_GFI(void)
{
	if (GFIC_SENSE() == 1)
	{
		GFI_errorOutCounter = 0;
		GFI_errorInCounter++;
		if (GFI_errorInCounter >= GFI_errorCount)
		{
			GFI_errorInCounter = 0;
			powerSide_data.tripStatus.bit.trip_GFI = 1;
		}
	}
	else
	{
		GFI_errorInCounter = 0;
		GFI_errorOutCounter++;
		if (GFI_errorOutCounter >= GFI_errorCount)
		{
			GFI_errorOutCounter = 0;
			powerSide_data.tripStatus.bit.trip_GFI = 0;
		}
	}
}

/*---------------------------------------------------
 	Check Over Current
 ---------------------------------------------------*/
static void check_OC(void)
{
#if THREE_PHASE
	if (((powerSide_data.current.IA >= OC_THRESHOLD)
			&& (powerSide_data.current.IA < 10000))
			|| ((powerSide_data.current.IB >= OC_THRESHOLD)
					&& (powerSide_data.current.IA < 10000))
			|| ((powerSide_data.current.IC >= OC_THRESHOLD)
					&& (powerSide_data.current.IA < 10000)))
	{
		OC_errorInCounter++;
		if (OC_errorInCounter >= OC_errorCount)
		{
			powerSide_data.tripStatus.bit.trip_OC = 1;
		}
	}
	else
	{
		OC_errorInCounter = 0;
	}
#else
	if ((powerSide_data.current.IA >= OC_THRESHOLD)
			&& (powerSide_data.current.IA < 10000))
	{
		OC_errorOutCounter = 0;
		OC_errorInCounter++;
		if (OC_errorInCounter >= OC_errorCount)
		{
			OC_errorInCounter = 0;
			powerSide_data.tripStatus.bit.trip_OC = 1;
		}
	}
	else
	{
		OC_errorInCounter = 0;
		OC_errorOutCounter++;
		if (OC_errorOutCounter >= OC_errorCount)
		{
			OC_errorOutCounter = 0;
			powerSide_data.tripStatus.bit.trip_OC = 0;
		}
	}
#endif
}

/*---------------------------------------------------
 	 Check Under Voltage
 ---------------------------------------------------*/
static void check_UV(void)
{
#if THREE_PHASE
	if ((powerSide_data.voltage.VA < charger_configGet.uv_lower)
			|| (powerSide_data.voltage.VB < charger_configGet.uv_lower)
			|| (powerSide_data.voltage.VC < charger_configGet.uv_lower))
	{
		UV_errorOutCounter = 0;
		UV_errorInCounter++;
		if (UV_errorInCounter >= UV_errorCount)
		{
			UV_errorInCounter = 0;
			powerSide_data.errorStatus.bit.error_UV = 1;
#if UV_dynamic
			controlSide_data.warnings.bits.UV_warn = 1;
#endif
		}
	}
	else
	{
		UV_errorInCounter = 0;
		if ((powerSide_data.voltage.VA >= charger_configGet.uv_upper)
				&& (powerSide_data.voltage.VB >= charger_configGet.uv_upper)
				&& (powerSide_data.voltage.VC >= charger_configGet.uv_upper))
		{
			UV_errorOutCounter++;
			if (UV_errorOutCounter >= UV_errorCount)
			{
				UV_errorOutCounter = 0;
				powerSide_data.errorStatus.bit.error_UV = 0;
			}
		}
	}
#else
	if (powerSide_data.voltage.VA < charger_configGet.uv_lower)
	{
		UV_errorOutCounter = 0;
		UV_errorInCounter++;
		if (UV_errorInCounter >= UV_errorCount)
		{
			powerSide_data.errorStatus.bit.error_UV = 1;
#if UV_dynamic
			controlSide_data.warnings.bits.UV_warn = 1;
#endif
		}
	}
	else
	{
		UV_errorInCounter = 0;
		if (powerSide_data.voltage.VA >= charger_configGet.uv_upper)
		{
			UV_errorOutCounter++;
			if (UV_errorOutCounter >= UV_errorCount)
			{
				UV_errorOutCounter = 0;
				powerSide_data.errorStatus.bit.error_UV = 0;
			}
		}
		else
		{
			UV_errorOutCounter = 0;
		}
	}
#endif
}

/*---------------------------------------------------
 Check Over Voltage
 ---------------------------------------------------*/
static void check_OV(void)
{
#if THREE_PHASE
	if ((powerSide_data.voltage.VA >= charger_configGet.ov_upper)
			|| (powerSide_data.voltage.VB >= charger_configGet.ov_upper)
			|| (powerSide_data.voltage.VC >= charger_configGet.ov_upper))
	{
		OV_errorOutCounter = 0;
		OV_errorInCounter++;
		if (OV_errorInCounter >= OV_errorCount)
		{
			powerSide_data.errorStatus.bit.error_OV = 1;
		}
	}
	else
	{
		OV_errorInCounter = 0;

		if ((powerSide_data.voltage.VA < charger_configGet.ov_lower)
				&& (powerSide_data.voltage.VB < charger_configGet.ov_lower)
				&& (powerSide_data.voltage.VC < charger_configGet.ov_lower))
		{
			OV_errorOutCounter++;
			if (OV_errorOutCounter >= OV_errorCount)
			{
				OV_errorOutCounter = 0;
				powerSide_data.errorStatus.bit.error_OV = 0;
			}
		}
	}
#else
	if (powerSide_data.voltage.VA >= charger_configGet.ov_upper)
	{
		OV_errorInCounter++;
		if (OV_errorInCounter >= OV_errorCount)
		{
			powerSide_data.errorStatus.bit.error_OV = 1;
		}
	}
	else
	{
		OV_errorInCounter = 0;

		if (powerSide_data.voltage.VA < charger_configGet.ov_lower)
		{
			OV_errorOutCounter++;
			if (OV_errorOutCounter >= OV_errorCount)
			{
				OV_errorOutCounter = 0;
				powerSide_data.errorStatus.bit.error_OV = 0;
			}
		}
		else
		{
			OV_errorOutCounter = 0;
		}
	}
#endif
}

/*---------------------------------------------------
 Check Phase Loss
 ---------------------------------------------------*/
static void check_PL(void)
{
	if ((powerSide_data.voltage.VA < 1000)
			|| (powerSide_data.voltage.VB < 1000)
			|| (powerSide_data.voltage.VB < 1000))
	{
		PL_errorInCounter++;
		PL_errorOutCounter = 0;

		if (PL_errorInCounter >= PL_errorCount)
		{
			powerSide_data.errorStatus.bit.error_PL = 1;
		}
	}
	else
	{
		PL_errorInCounter = 0;

		if (controlSide_data.status.bit.connector_state == 0)
		{
			if ((powerSide_data.voltage.VA >= 1500)
					&& (powerSide_data.voltage.VB >= 1500)
					&& (powerSide_data.voltage.VC >= 1500))
			{
				PL_errorOutCounter++;

				if (PL_errorOutCounter >= UV_errorCount)
				{
					PL_errorOutCounter = 0;
					powerSide_data.errorStatus.bit.error_PL = 0;
				}
			}
		}
	}
}

/*---------------------------------------------------
 Check Frequency
 ---------------------------------------------------*/
static void check_Freq(void)
{
	if ((powerSide_data.frequency >= charger_configGet.freq_lower)
			&& (powerSide_data.frequency <= charger_configGet.freq_upper))
	{
		Freq_errorInCounter = 0;
		Freq_errorOutCounter++;
		if (Freq_errorOutCounter >= Freq_errorCount)
		{
			Freq_errorOutCounter = 0;
			powerSide_data.errorStatus.bit.error_Freq = 0;
		}
	}
	else
	{
		Freq_errorOutCounter = 0;
		Freq_errorInCounter++;
		if (Freq_errorInCounter >= Freq_errorCount)
		{
			Freq_errorInCounter = 0;
			powerSide_data.errorStatus.bit.error_Freq = 1;
		}
	}
}


/*---------------------------------------------------
 Check Stuck contactor
 ---------------------------------------------------*/
static void check_SC(void)
{
	//=========================== Stuck contactor condition check
	if ((currentState != STATE_C2)
			&& (powerSide_data.status.bit.contactor_state == ON))
	{
		SC_errorInCounter++;
		if (SC_errorInCounter >= SC_errorCount)
		{
			SC_errorInCounter = 0;
			powerSide_data.errorStatus.bit.error_SR_C = 1;
		}
	}
	else
	{
		SC_errorInCounter = 0;
		powerSide_data.errorStatus.bit.error_SR_C = 0;
	}
}

/*---------------------------------------------------
 Check over temperature
 ---------------------------------------------------*/
static void check_OT(void)
{
	if (((uint8_t)powerSide_data.tempSensors.T1) >= OT_UPPER_THRESHOLD)
	{
		OT_errorOutCounter = 0;
		OT_errorInCounter++;
		if (OT_errorInCounter >= OT_errorCount)
		{
			OT_errorInCounter = 0;
			powerSide_data.errorStatus.bit.error_OT = 1;
		}
	}
	else
	{
		OT_errorInCounter = 0;

		if (((uint8_t)powerSide_data.tempSensors.T1) < OT_LOWER_THRESHOLD)
		{
			OT_errorOutCounter++;
			if (OT_errorOutCounter >= OT_errorCount)
			{
				OT_errorOutCounter = 0;
				powerSide_data.errorStatus.bit.error_OT = 0;
			}
		}
	}
}

/*---------------------------------------------------
 Error detector function
 ---------------------------------------------------*/
void errorDetector(void)
{
	if (charger_configGet.en_1.bit.sc_en == 1)
	{
		check_SC();
	}

	if ((charger_configGet.en_1.bit.gfi_en == 1) && (GFI_bypasss_flag == false))
	{
		GFIC_RESET_OFF();
		check_GFI();

		if (charger_configGet.en_1.bit.gfit_en == 1)
		{
			check_GFITest();
		}
	}
	else
	{
		GFIC_RESET_ON();

		if (timeout.timeout_3s == true)
		{
			GFI_bypasss_flag = false;
		}
	}

	check_OC();

	if (charger_configGet.en_1.bit.pl_en == 1)
	{
		check_PL();
	}

	if (charger_configGet.en_2.bit.ot_en == 1)
	{
		check_OT();
	}

	if (charger_configGet.en_1.bit.ov_en == 1)
	{
		check_OV();
	}

	if (charger_configGet.en_1.bit.uv_en == 1)
	{
		check_UV();
	}

	if (charger_configGet.en_1.bit.freq_en == 1)
	{
		check_Freq();
	}
}

/*---------------------------------------------------
 Error handler function
 ---------------------------------------------------*/
void errorHandler(void)
{
	/*Stuck contactor handler*/
	if (powerSide_data.errorStatus.bit.error_SR_C == 1)
	{
		controlSide_data.errorStatus.bit.fault_level = PRIORITY_2;
		currentError = SC;
	}
	/*GFI Test failure handler*/
	if (powerSide_data.errorStatus.bit.error_GFI_test == 1)
	{
		controlSide_data.errorStatus.bit.fault_level = PRIORITY_2;
		currentError = GFI_Test;
	}
	/*GFI handler*/
	if (powerSide_data.tripStatus.bit.trip_GFI == 1)
	{
		if (GFI_triggered == false)
		{
			GFI_triggered = true;
			GFCI_errorCounter++;
			gfic_reset = true;
		}

		if (GFCI_errorCounter >= GFCI_errorCount)
		{
			controlSide_data.errorStatus.bit.fault_level = PRIORITY_2;
			currentError = GFI;
		}
		else
		{
			controlSide_data.errorStatus.bit.fault_level = PRIORITY_1;
		}
	}
	/*Modbus error handler*/
	if (controlSide_data.errorStatus.bit.serialAError == 1)
	{
		controlSide_data.errorStatus.bit.fault_level = PRIORITY_2;
		currentError = modbus_error;
	}
	/*Over current handler*/
	if (powerSide_data.tripStatus.bit.trip_OC == 1)
	{
		controlSide_data.errorStatus.bit.fault_level = PRIORITY_2;
		currentError = OC;
	}
	/*Phase loss handler*/
	if (powerSide_data.errorStatus.bit.error_PL == 1)
	{
		controlSide_data.errorStatus.bit.fault_level = PRIORITY_2;
		currentError = PL;
	}
	/*Over temperature handler*/
	if (powerSide_data.errorStatus.bit.error_OT == 1)
	{
		controlSide_data.errorStatus.bit.fault_level = PRIORITY_1;
		currentError = OT;
	}
	/*Control pilot fault handler*/
	if (controlSide_data.errorStatus.bit.CPFault == 1)
	{
		controlSide_data.errorStatus.bit.fault_level = PRIORITY_2;
		currentError = CP_Fault;
	}
	/*Diode check failure handler*/
	if (controlSide_data.errorStatus.bit.diodeCheck_failed == 1)
	{
		controlSide_data.errorStatus.bit.fault_level = PRIORITY_2;
		currentError = Diode_Failure;
	}
	/*Over voltage handler*/
	if (powerSide_data.errorStatus.bit.error_OV == 1)
	{
		controlSide_data.errorStatus.bit.fault_level = PRIORITY_1;
		currentError = OV;
	}
	/*Under voltage handler*/
	if (powerSide_data.errorStatus.bit.error_UV == 1)
	{
		controlSide_data.errorStatus.bit.fault_level = PRIORITY_1;
		currentError = UV;
	}
	/*Frequency error handler*/

	if (powerSide_data.errorStatus.bit.error_Freq == 1)
	{
		controlSide_data.errorStatus.bit.fault_level = PRIORITY_1;
		currentError = Freq;
	}

	/*GFI trip reset handler*/
	if (gfic_reset == true)
	{
		if (GFCI_errorCounter >= GFCI_errorCount)
		{
			if (controlSide_data.status.bit.connector_state == 0)
			{
				GFCI_errorCounter = 0;
				GFIC_RESET_ON();

				if (powerSide_data.tripStatus.bit.trip_GFI == 0)
				{
					GFI_triggered = false;
					gfic_reset = false;
					GFIC_RESET_OFF();
				}
			}
		}
		else
		{
			GFIC_RESET_ON();

			if (powerSide_data.tripStatus.bit.trip_GFI == 0)
			{
				GFI_triggered = false;
				gfic_reset = false;
				GFIC_RESET_OFF();
			}
		}
	}

	/*Handle error flags clearing*/
	if ((powerSide_data.tripStatus.all == 0)
			&& (powerSide_data.errorStatus.all == 0)
			&& (controlSide_data.errorStatus.bit.diodeCheck_failed == 0)
			&& (controlSide_data.errorStatus.bit.CPFault == 0)
			&& (controlSide_data.errorStatus.bit.serialAError == 0))
	{
		controlSide_data.errorStatus.bit.fault_level = 0;
	}
}

void errorBuffer_log(void)
{
	if (currentError != previousError)
	{
		error_buffer[errorLog_count] = currentError;
		errorLog_count++;

		if (errorLog_count >= ERROR_BUFFER_SIZE)
		{
			errorLog_count = 0;
		}

		previousError = currentError;
	}
}

void errorBuffer_report(void)
{
	static uint8_t id = 0;

	if (error_handler.timeout_8s == true)
	{
		tick_clear(&error_handler);

		controlSide_data.errorReport = error_buffer[id];
		id++;

		if (id >= errorLog_count)
		{
			id = 0;
		}
	}
}

void errorAlarm(void)
{
//	if (error_buffer[0] != 0)
//	{
//		switch (controlSide_data.errorReport)
//		{
//		case 1:
//			BUZZER_ON();
//			HAL_Delay(SHORT_TIME);
//			BUZZER_OFF();
//			HAL_Delay(PAUSE_TIME);
//			break;
//		default:
//			break;
//		}
//	}
}

void errorBuffer_clear(void)
{
	currentError = 0;
	previousError = 0;
	errorLog_count = 0;
	memset((uint8_t*) error_buffer, 0, (sizeof(error_buffer)/sizeof(error_buffer[0])));
}
