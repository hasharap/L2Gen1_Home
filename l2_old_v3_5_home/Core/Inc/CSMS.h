/*
 * CSMS.h
 *
 *  Created on: Sep 20, 2023
 *      Author: Kaushan Ranasinghe
 */

#ifndef INC_CSMS_H_
#define INC_CSMS_H_

#include "main.h"
#include "data_structures.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

/*##############################################################################
 *==============================================================================
 *  Title:  CSMS Module (chargeNET)
 *
 *  Target: Developed for STM32 MCUs.
 *==============================================================================
 *############################################################################*/


/*==============================================================================
 *  Section:  Device Parameters
 *
 *  Description: Configure device parameters here.
 *============================================================================*/

/*----------------------------------------------------------------------------
 * Configuration Defines
 *----------------------------------------------------------------------------*/

#define CSMS_TICK_TIME 				                1 // timer period - milliseconds
#define CSMS_RX_SERIAL_FAULT_ENABLE                 0

/*----------------------------------------------------------------------------
 * Public Prototypes
 *----------------------------------------------------------------------------*/

void CSMS_Init(void);
void CSMS_Intick(void); // period in milliseconds
void CSMS_transmitSerial(void (*TransmitFunction)(uint8_t*, uint16_t));
void CSMS_receiveSerial(const char* restrict data);
void CSMS_functions(void);

/*==============================================================================
 *  Section:  CSMS Stack
 *
 *  Description: Private definitions - DO NOT CHANGE!
 *============================================================================*/

/*----------------------------------------------------------------------------
 * Private Defines
 *----------------------------------------------------------------------------*/

#define CSMS_TRANSMIT_INTERVAL		                ((25 * 10) 	    / CSMS_TICK_TIME) // 250 ms
#define CSMS_RX_DEAD_TIME_INTERVAL	                ((5 * 10)       / CSMS_TICK_TIME) // 50 ms
#define CSMS_RX_SERIAL_FAULT_TIMEOUT_INTERVAL   	((5 * 1000)     / CSMS_TICK_TIME) // 5 s

/*----------------------------------------------------------------------------
 * Value Tables
 *----------------------------------------------------------------------------*/

typedef enum
{
	cNET_idle 		= 2,
	cNET_preStart 	= 4,
	cNET_start 		= 5,
	cNET_stop 		= 6,
}chargeNETState_t;

typedef enum
{
	RxError = 0,
	RxIdle,
	RxSoF,
	RxData,
	RxEoF,
}RxSerialState_t;

/*----------------------------------------------------------------------------
 * Structures
 *----------------------------------------------------------------------------*/

typedef struct
{
    struct
	{
    	chargeNETState_t currentState;
    	uint8_t stopCharge;
    	uint8_t errorTesting;
    	uint8_t msgID;
	}request;

	struct
	{
		union
		{
			uint8_t all;
			struct
			{
				uint8_t currentState 		:5;
				uint8_t charging_active 	:1;
				uint8_t cpPWM_active 		:1;
				uint8_t connector_state		:1;
			}__attribute__((packed)) bits;
		}controlSide_status;

		uint8_t powerSide_status;

		union
		{
			uint8_t all;
			struct
			{
				uint8_t start_Charge 			:1;
				uint8_t stop_Charge 			:1;
				uint8_t schedule_charge 		:1;
				uint8_t vehicle_Check 			:1;
				uint8_t charge_pause 			:1;
				uint8_t rtcUpdateComplete		:1;
				uint8_t rtcUpdateAlarmComplete	:1;
			}bits;
		}networkSide_status;

		uint8_t controlSide_error;

		union
		{
			uint8_t all;
			struct
			{
				uint8_t error_OV_L1 	:1;
				uint8_t error_UV_L1 	:1;
				uint8_t error_SR_L1 	:1;
				uint8_t error_SR_N 		:1;
				uint8_t error_SR_C 		:1;
				uint8_t error_GFI_test	:1;
			}bits;
		}powerSide_error;

		uint16_t voltage;
		uint16_t current;
		uint16_t power;
		uint16_t kWh;
		uint16_t T1;
		uint16_t T2;
		uint16_t T3;
		uint8_t v1;
		uint8_t v2;
		uint8_t v3;

	}response;

	bool authentication;
	bool stopCharge;

}CSMS_t;

typedef union
{
	uint16_t all;
    struct
    {
        uint16_t off		:1;
        uint16_t on			:1;
        uint16_t idle		:1;
        uint16_t tapCard	:1;
        uint16_t authCard	:1;
        uint16_t authPass	:1;
        uint16_t authFail	:1;
        uint16_t preStart	:1;
        uint16_t start		:1;
        uint16_t preStop	:1;
        uint16_t stop		:1;
        uint16_t close		:1;
    }bits;
}chargeNET_t;


/*==============================================================================
 *  Section:  CSMS STM32 Settings
 *
 *  Description: Private definitions - DO NOT CHANGE!
 *============================================================================*/

/*----------------------------------------------------------------------------
 * Private Defines
 *----------------------------------------------------------------------------*/

#define CSMS_QUEUE_SIZE 16
#define CSMS_FRAME_SIZE 100

/*----------------------------------------------------------------------------
 * RS485 Queue Structures
 *----------------------------------------------------------------------------*/

typedef struct
{
	uint16_t ActiveMask;
	uint8_t Length[CSMS_QUEUE_SIZE];
	uint8_t ID[CSMS_QUEUE_SIZE];
	uint8_t TxData[CSMS_QUEUE_SIZE][CSMS_FRAME_SIZE];
}CSMS_MessageQueue;

/*----------------------------------------------------------------------------
 * Public Instantiations
 *----------------------------------------------------------------------------*/

extern volatile CSMS_t CSMS;
extern volatile CSMS_MessageQueue CSMS_Queue;

/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
 * Private Prototypes
 *----------------------------------------------------------------------------*/


#endif /* INC_CSMS_H_ */
