/**
 ******************************************************************************
 * @file           : protection.c
 * @brief          : Protection Functions Implementation
 ******************************************************************************
 * @attention
 *
 * Author: Nisal Bulathsinghala, Nov 21, 2022
 * Copyright (c) 2022 Vega Innovations, Sri Lanka.
 * All rights reserved.
 ******************************************************************************
 * @details
 * This file contains the implementation of various safety and protection
 * functions including:
 * - Stuck relays
 * - Ground Fault Interruption (GFI)
 * - Overcurrent (OC)
 * - Undervoltage (UV)
 * - Overvoltage (OV)
 * - Phase loss (PL)
 * - Overtemperature (OT)
 * - Frequency monitoring
 * - Contactor feedback (CFB)
 *
 * It ensures the system operates safely within predefined thresholds
 * and provides mechanisms for error detection and handling.
 ******************************************************************************
 */

#include "protection.h"

/** @brief GFI reset flag. */
bool gfic_reset = false;

/** @brief GFI test run flag. */
volatile bool run_GFITest = false;

/** @brief GFI error counter. */
uint8_t GFCI_errorCounter = 0;

/** @brief GFI triggered flag. */
static bool GFI_triggered = false;

/** @brief GFI error count threshold. */
#define GFI_errorCount 300

/** @brief GFI error in counter. */
uint16_t GFI_errorInCounter = 0;

/** @brief GFI error out counter. */
uint16_t GFI_errorOutCounter = 0;

/** @brief Overcurrent threshold in milliamps. */
#define OC_THRESHOLD 3500

/** @brief Overcurrent error count threshold. */
#define OC_errorCount (1000 / PROTECTION_TICK_TIME) // 1 second

/** @brief Overcurrent error in counter. */
uint16_t OC_errorInCounter = 0;

/** @brief Overcurrent error out counter. */
uint16_t OC_errorOutCounter = 0;

/** @brief Undervoltage lower threshold in millivolts. */
#define UV_LOWER_THRESHOLD 2000

/** @brief Undervoltage upper threshold in millivolts. */
#define UV_UPPER_THRESHOLD 2050

/** @brief Undervoltage error count threshold. */
#define UV_errorCount (5000 / PROTECTION_TICK_TIME) // 2 seconds

/** @brief Undervoltage error in counter. */
uint16_t UV_errorInCounter = 0;

/** @brief Undervoltage error out counter. */
uint16_t UV_errorOutCounter = 0;

/** @brief Overvoltage lower threshold in millivolts. */
#define OV_LOWER_THRESHOLD 2500

/** @brief Overvoltage upper threshold in millivolts. */
#define OV_UPPER_THRESHOLD 2600

/** @brief Overvoltage error count threshold. */
#define OV_errorCount (2000 / PROTECTION_TICK_TIME) // 2 seconds

/** @brief Overvoltage error in counter. */
uint16_t OV_errorInCounter = 0;

/** @brief Overvoltage error out counter. */
uint16_t OV_errorOutCounter = 0;

/** @brief Phase Loss error count threshold. */
#define PL_errorCount (2000 / PROTECTION_TICK_TIME) // 2 seconds

/** @brief Phase Loss error in counter. */
uint16_t PL_errorInCounter = 0;

/** @brief Phase Loss error out counter. */
uint16_t PL_errorOutCounter = 0;

/** @brief Frequency lower threshold in millihertz. */
#define FREQ_LOWER_THRESHOLD 4940

/** @brief Frequency upper threshold in millihertz. */
#define FREQ_UPPER_THRESHOLD 5060

/** @brief Frequency error count threshold. */
#define Freq_errorCount (2000 / PROTECTION_TICK_TIME) // 2 seconds

/** @brief Frequency period in milliseconds. */
#define FREQ_PERIOD 1000

/** @brief Frequency period value. */
uint16_t FREQ_period = 0;

/** @brief Frequency error in counter. */
uint16_t Freq_errorInCounter = 0;

/** @brief Frequency error out counter. */
uint16_t Freq_errorOutCounter = 0;

/** @brief Stuck Contactor error count threshold. */
#define SC_errorCount (1000 / PROTECTION_TICK_TIME) // 1 second

/** @brief Stuck Contactor error in counter. */
uint16_t SC_errorInCounter = 0;

/** @brief Stuck Contactor error out counter. */
uint16_t SC_errorOutCounter = 0;

/** @brief Contactor Feedback error count threshold. */
#define CFB_errorCount (10000 / PROTECTION_TICK_TIME) // 10 seconds

/** @brief Contactor Feedback error in counter. */
uint16_t CFB_errorInCounter = 0;

/** @brief Contactor Feedback error out counter. */
uint16_t CFB_errorOutCounter = 0;

/** @brief Over Temperature lower threshold in degrees Celsius. */
#define OT_LOWER_THRESHOLD 55

/** @brief Over Temperature upper threshold in degrees Celsius. */
#define OT_UPPER_THRESHOLD 60

/** @brief Over Temperature error count threshold. */
#define OT_errorCount (20000 / PROTECTION_TICK_TIME) // 20 seconds

/** @brief Over Temperature error in counter. */
uint16_t OT_errorInCounter = 0;

/** @brief Over Temperature error out counter. */
uint16_t OT_errorOutCounter = 0;

/** @brief Buffer for error logging. */
volatile uint8_t error_buffer[ERROR_BUFFER_SIZE] = { 0 };

/** @brief Current error code. */
uint8_t currentError = 0;

/** @brief Previous error code. */
uint8_t previousError = 0;

/** @brief Error log count. */
uint8_t errorLog_count = 0;

/** @brief GFI bypass flag. */
volatile bool GFI_bypasss_flag = false;

/**
 * @brief Perform Ground Fault Interruption (GFI) test.
 *
 * @details
 * Activates the GFI test mechanism, monitors the result, and updates the error
 * status based on the test outcome. Uses a state machine for sequencing the test.
 *
 * @note Should be called only when `run_GFITest` is true.
 */
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

/**
 * @brief Monitor and handle Ground Fault Interruption (GFI) trips.
 *
 * @details
 * Monitors the GFI sensor. If a fault condition is detected and persists
 * for a defined count, the trip flag is set.
 */
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

/**
 * @brief Monitor and handle overcurrent conditions.
 *
 * @details
 * Monitors the current levels for single-phase or three-phase systems.
 * Triggers an overcurrent fault if the current exceeds the defined threshold.
 */
static void check_OC(void)
{
	if (threePhaseMeter)
	{
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
	}

	if (singlePhaseMeter)
	{
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
	}
}

/**
 * @brief Monitor and handle undervoltage conditions.
 *
 * @details
 * Checks voltage levels against the lower thresholds. If an undervoltage
 * condition persists beyond a set count, triggers an undervoltage error.
 */
static void check_UV(void)
{
	if (threePhaseMeter)
	{
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
	}

	if (singlePhaseMeter)
	{
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
	}
}

/**
 * @brief Monitor and handle overvoltage conditions.
 *
 * @details
 * Checks voltage levels against upper thresholds. If the voltage exceeds
 * defined limits for a prolonged period, triggers an overvoltage error.
 */
static void check_OV(void)
{
	if (threePhaseMeter)
	{
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
	}
	if (singlePhaseMeter)
	{
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
	}
}

/**
 * @brief Monitor and handle phase loss conditions.
 *
 * @details
 * Detects phase loss scenarios by checking voltage levels below a defined
 * threshold. Updates the system's error status accordingly.
 */
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

/**
 * @brief Monitor and handle frequency errors.
 *
 * @details
 * Checks system frequency against defined upper and lower bounds.
 * Triggers an error if the frequency deviates beyond these limits.
 */
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


/**
 * @brief Monitor and handle stuck contactor conditions.
 *
 * @details
 * Detects stuck contactors by monitoring their state and triggers an error
 * if the contactor remains in an unexpected state.
 */
static void check_SC(void)
{
	//=========================== Stuck contactor condition check
	if ((sm.currentState != STATE_C2)
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

/**
 * @brief Monitor and handle contactor feedback errors.
 *
 * @details
 * Detects feedback issues in contactor operations. If a contactor state
 * mismatch persists, triggers an error.
 */
static void check_CFB(void)
{
	if ((sm.currentState == STATE_C2)
			&& (powerSide_data.status.bit.contactor_state == OFF))
	{
		CFB_errorInCounter++;
		if (CFB_errorInCounter >= CFB_errorCount)
		{
			CFB_errorInCounter = 0;
			powerSide_data.errorStatus.bit.error_CFB = 1;
		}
	}
	else
	{
		CFB_errorInCounter = 0;

		if (controlSide_data.status.bit.connector_state == 0)
		{
			powerSide_data.errorStatus.bit.error_CFB = 0;
		}
	}
}

/**
 * @brief Monitor and handle overtemperature conditions.
 *
 * @details
 * Checks temperature readings against upper thresholds. Triggers an
 * overtemperature fault if the temperature exceeds safe limits.
 */
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

/**
 * @brief Detect errors in the system.
 *
 * @details
 * Aggregates all individual checks for protection mechanisms including
 * overcurrent, undervoltage, overvoltage, and more. Triggers appropriate
 * error handling routines as required.
 */
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

//	check_CFB();
}

/**
 * @brief Handle errors detected in the system.
 *
 * @details
 * Processes all detected errors, sets fault levels, and updates error logs.
 * Handles specific faults such as GFI trips, overcurrent, undervoltage,
 * overvoltage, and stuck contactors.
 */
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
	/*Contactor feedback error handler*/
	if (powerSide_data.errorStatus.bit.error_CFB == 1)
	{
		controlSide_data.errorStatus.bit.fault_level = PRIORITY_2;
		currentError = CFB;
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

/**
 * @brief Log errors to the buffer.
 *
 * @details
 * Maintains a log of detected errors for analysis or reporting. Ensures the
 * error buffer is updated only when a new error occurs.
 */
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

/**
 * @brief Report errors from the buffer.
 *
 * @details
 * Retrieves and reports logged errors sequentially from the error buffer.
 */
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

/**
 * @brief Trigger error alarms.
 *
 * @details
 * Activates alarms based on the current error report. Alarms provide an
 * auditory indication of critical system errors.
 */
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

/**
 * @brief Clear the error buffer.
 *
 * @details
 * Resets the error buffer and clears any stored error information.
 */
void errorBuffer_clear(void)
{
	currentError = 0;
	previousError = 0;
	errorLog_count = 0;
	memset((uint8_t*) error_buffer, 0, (sizeof(error_buffer)/sizeof(error_buffer[0])));
}
