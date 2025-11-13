/**
 ******************************************************************************
 * @file           : protection.h
 * @brief          : Header for protection.c file.
 *					 This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Author: Nisal Bulathsinghala, Nov 21, 2022
 * Copyright (c) 2022 Vega Innovations, Sri Lanka.
 * All rights reserved.
 ******************************************************************************
 */

#ifndef PROTECTION_H_
#define PROTECTION_H_
#include "main.h"

#define GFCI_errorCount		3
#define PRIORITY_1	1
#define PRIORITY_2	2

#define PROTECTION_TICK_TIME	0.5	//500us

typedef enum
{
	SC = 1,
	GFI_Test,
	GFI,
	modbus_error,
	OC,
	PL,
	OT,
	CP_Fault,
	Diode_Failure,
	OV,
	UV,
	Freq
} errorCode_t;

void stuck_relay_detection();
void gfci_sense();

extern bool stuck_relay_test ;
extern bool gfic_test_run;
extern bool gfic_reset;
extern bool ocp_reset;

extern uint8_t GFCI_errorCounter;
extern volatile bool run_GFITest;

extern volatile bool GFI_bypasss_flag;

void errorDetector(void);
void errorHandler(void);
void errorBuffer_log(void);
void errorBuffer_report(void);
void errorBuffer_clear(void);

#define ERROR_BUFFER_SIZE	5

#endif /* PROTECTION_H_ */
