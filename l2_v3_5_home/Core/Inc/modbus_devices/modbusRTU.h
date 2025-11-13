/*
 * modbusRTU.h
 *
 *  Created on: Jul 9, 2023
 *  Revised on: Oct 10, 2023
 *      Author: Kaushan Ranasinghe
 */

#ifndef INC_MODBUSRTU_H_
#define INC_MODBUSRTU_H_

#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stddef.h>

/*##############################################################################
 *==============================================================================
 *  Title:  Modbus-RTU Module
 *
 *  Target: Developed for STM32 MCUs.
 *==============================================================================
 *############################################################################*/


/*==============================================================================
 *  Section:  Protocol Parameters
 *
 *  Description: Configure device parameters here.
 *============================================================================*/

/*----------------------------------------------------------------------------
 * Configuration Defines
 *----------------------------------------------------------------------------*/

#define MODBUS_TICK_TIME 				1 // timer period - milliseconds

#define MODBUS_SNIFFER					0

#define MODBUS_DEVICES 					3

#define ADL_ENABLE 						1
#define IMD_ENABLE						0

/*==============================================================================
 *  Section:  Modbus-RTU Stack
 *
 *  Description: Private definitions - DO NOT CHANGE!
 *============================================================================*/

/*----------------------------------------------------------------------------
 * Private Defines
 *----------------------------------------------------------------------------*/

#define MODBUS_TIMEOUT 					((2 * 1000) 	/ MODBUS_TICK_TIME) // delay in ms
#define MODBUS_TRANSMIT_CYCLE	 		((1 * 100) 		/ MODBUS_TICK_TIME)
#define MODBUS_TRANSMIT_INTERVAL 		((1 * 10) 		/ MODBUS_TICK_TIME)
#define MODBUS_RX_TIMEOUT_INTERVAL		((1 * 1000) 	/ MODBUS_TICK_TIME)
#define MODBUS_RX_DEAD_TIME_INTERVAL	((5 * 1) 		/ MODBUS_TICK_TIME)
#define MODBUS_SNIFF_TIMEOUT			((2 * 10) 		/ MODBUS_TICK_TIME)

#define MAX_FRAME_SIZE 					64 // max packet size standard - 256
#define MAX_DATA_SIZE 					60 // standard - 252

/* Function Codes */

#define READ_COIL_STATUS				0x01
#define READ_INPUT_STATUS				0x02
#define READ_HOLDING_REGISTERS			0x03
#define READ_INPUT_REGISTERS			0x04
#define WRITE_SINGLE_COIL_STATUS		0x05
#define WRITE_SINGLE_REGISTER			0x06
#define MULTIPLE_COIL_WRITE				0x0F
#define MULTIPLE_REGISTER_WRITE			0x10

/* Register Sizes (16-bit return) */

#define HWORD_ALL						0x0000 // returns a full data frame
#define HWORD_1 						0x0001
#define HWORD_2 						0x0002
#define HWORD_3 						0x0003
#define HWORD_4 						0x0004

/*----------------------------------------------------------------------------
 * Value Tables
 *----------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------
 * Structures
 *----------------------------------------------------------------------------*/

typedef struct
{
	struct
	{
		struct
		{
			uint8_t slaveAddress;
			uint8_t functionCode;
			uint16_t startAddress;
			uint16_t numberOfRegisters;
			uint16_t writeData[MAX_DATA_SIZE];
		}pdu;
		uint16_t crc;
	}adu;
}Modbus_t;

typedef struct
{
	union
	{
		uint32_t all;
		struct
		{
			uint8_t slaveAddress;
			uint8_t functionCode;
			uint16_t startAddress;
		}bits;
	}ID;

}ModbusID_t;

/*==============================================================================
 *  Section:  CSMS STM32 Settings
 *
 *  Description: Private definitions - DO NOT CHANGE!
 *============================================================================*/

/*----------------------------------------------------------------------------
 * Private Defines
 *----------------------------------------------------------------------------*/

#define MODBUS_QUEUE_SIZE 			32

/*----------------------------------------------------------------------------
 * RS485 Queue Structures
 *----------------------------------------------------------------------------*/

typedef struct
{
	uint32_t ActiveMask;
	uint8_t Length[MODBUS_QUEUE_SIZE];
	uint32_t ID[MODBUS_QUEUE_SIZE];
	uint8_t TxData[MODBUS_QUEUE_SIZE][MAX_FRAME_SIZE]; // data array of 8 bytes for 16 data instructions
}Modbus_MessageQueue;



/*----------------------------------------------------------------------------
 * Public Instantiations
 *----------------------------------------------------------------------------*/

extern volatile bool ModbusRxError;

/*----------------------------------------------------------------------------
 * Private Prototypes
 *----------------------------------------------------------------------------*/



/*----------------------------------------------------------------------------
 * Public Prototypes
 *----------------------------------------------------------------------------*/

void ModbusRTU_Init(void);
void ModbusRTU_Intick(void); // period in milliseconds

void ModbusRTU_queueSerial(const Modbus_t* restrict obj);
void ModbusRTU_transmitSerial(void (*TransmitFunction)(uint8_t*, uint16_t));
void ModbusRTU_receiveSerial(const char* restrict data);

void ModbusRTU_functions(void);

#endif /* INC_MODBUSRTU_H_ */
