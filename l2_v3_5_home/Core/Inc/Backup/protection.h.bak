/**
 ******************************************************************************
 * @file           : protection.h
 * @brief          : Header file for protection.c
 ******************************************************************************
 * @attention
 *
 * Author: Nisal Bulathsinghala, Nov 21, 2022
 * Copyright (c) 2022 Vega Innovations, Sri Lanka.
 * All rights reserved.
 ******************************************************************************
 * @details
 * This header file contains common defines, type definitions, and function
 * prototypes used for the implementation of safety and protection mechanisms.
 * The mechanisms include:
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
 * These definitions and declarations support system protection by providing
 * thresholds, error codes, and function prototypes for detecting, logging,
 * and handling errors.
 ******************************************************************************
 */

#ifndef PROTECTION_H_
#define PROTECTION_H_

#include "main.h"

/** @brief Number of errors required to trigger a GFI error. */
#define GFCI_errorCount 3

/** @brief Priority level 1 for critical errors. */
#define PRIORITY_1 1

/** @brief Priority level 2 for non-critical errors. */
#define PRIORITY_2 2

/** @brief Tick time in milliseconds for protection system checks. */
#define PROTECTION_TICK_TIME 0.5 // 500 microseconds

/** @brief Maximum size of the error buffer. */
#define ERROR_BUFFER_SIZE 5

/**
 * @brief Enumeration of error codes for the protection system.
 */
typedef enum
{
    SC = 1,          /**< Stuck contactor error. */
    GFI_Test,        /**< GFI test failure. */
    GFI,             /**< Ground Fault Interruption. */
    modbus_error,    /**< Modbus communication error. */
    OC,              /**< Overcurrent error. */
    PL,              /**< Phase loss error. */
    OT,              /**< Overtemperature error. */
    CP_Fault,        /**< Control pilot fault. */
    Diode_Failure,   /**< Diode failure error. */
    OV,              /**< Overvoltage error. */
    UV,              /**< Undervoltage error. */
    Freq,            /**< Frequency error. */
    CFB              /**< Contactor feedback error. */
} errorCode_t;

/**
 * @brief Function to detect stuck relays.
 */
void stuck_relay_detection();

/**
 * @brief Function to sense the state of the GFCI.
 */
void gfci_sense();

/** @brief Flag to indicate if GFI test is running. */
extern bool gfic_test_run;

/** @brief Flag to reset the GFI. */
extern bool gfic_reset;

/** @brief Flag to reset overcurrent protection. */
extern bool ocp_reset;

/** @brief Counter for GFCI errors. */
extern uint8_t GFCI_errorCounter;

/** @brief Flag to run the GFI test. */
extern volatile bool run_GFITest;

/** @brief Flag to bypass the GFI functionality. */
extern volatile bool GFI_bypasss_flag;

/**
 * @brief Detect errors in the protection system.
 */
void errorDetector(void);

/**
 * @brief Handle detected errors in the protection system.
 */
void errorHandler(void);

/**
 * @brief Log errors into the error buffer.
 */
void errorBuffer_log(void);

/**
 * @brief Report errors from the error buffer.
 */
void errorBuffer_report(void);

/**
 * @brief Clear all entries in the error buffer.
 */
void errorBuffer_clear(void);

#endif /* PROTECTION_H_ */
