/*
 * imd.h
 *
 *  Created on: Jul 12, 2023
 *  Revised on: Oct 2, 2023
 *      Author: Kaushan Ranasinghe
 */

#ifndef INC_IMD_H_
#define INC_IMD_H_

#include <stdint.h>
#include <stdbool.h>

/*##############################################################################
 *==============================================================================
 *  Title:  Insulation Monitoring Device Library (IMD)
 *
 *  Target: Developed for STM32 MCUs.
 *==============================================================================
 *############################################################################*/


/*==============================================================================
 *  Section:  IMD Stack
 *
 *  Description: Configure device parameters here.
 *============================================================================*/

/*----------------------------------------------------------------------------
 * Public Defines
 *----------------------------------------------------------------------------*/

#define IMD1_SLAVE_ADDRESS	0x01
#define IMD2_SLAVE_ADDRESS	0x02

/*----------------------------------------------------------------------------
 * Private Defines
 *----------------------------------------------------------------------------*/

// Read Data Registers

#define IMD_BUS_VOLTAGE 					0x0010
#define IMD_POSITIVE_RESISTANCE 			0x0012
#define IMD_NEGATIVE_RESISTANCE 			0x0013

#define IMD_EV_SIDE_VOLTAGE 				0x0014

#define IMD_IO_STATUS 						0x001B

// Write Data Registers

#define IMD_INSULATION_MONITORING_CONTROL 	0x0102
#define IMD_SET_ADDRESS						0x0103
#define IMD_SET_BAUD_RATE					0x0104

// Write Commands

#define IMD_ENABLE_DC1 						0x11
#define IMD_ENABLE_DC2 						0x12
#define IMD_DISABLE							0x00

/*----------------------------------------------------------------------------
 * Value Tables
 *----------------------------------------------------------------------------*/

typedef enum
{
	IMD_DC1_ON = 0x11,
	IMD_DC2_ON = 0x12,
	IMD_OFF	= 0,
}IMDMonitoring_t;

/*----------------------------------------------------------------------------
 * Structures
 *----------------------------------------------------------------------------*/

typedef union
{
	uint8_t all;

	struct
	{
		uint8_t reserved0 						: 1; // bit0
		uint8_t insulationMonitoringEffective 	: 1; // bit1
		uint8_t insulationMonitoringOn 			: 1; // bit2
		uint8_t reserved1 						: 1; // bit3
		uint8_t reserved2 						: 1; // bit4
		uint8_t reserved3 						: 1; // bit5
		uint8_t evVoltageReverseAlarm 			: 1; // bit6
		uint8_t busVoltageReverseAlarm 			: 1; // bit7
	}bits;
}IMD_ioStatusRegister;


typedef struct
{
	uint8_t slaveAddress;
	uint8_t insulationMonitoringControl;

	struct
	{
		float busVoltage;
		float evVoltage;
		uint16_t positiveResistance; // kOhms
		uint16_t negativeResistance; // kOhms
		IMDMonitoring_t insulationMonitoringControl;
		IMD_ioStatusRegister statusRegister;
	}data;

}IMD_t;


/*
 *
 *  Invalid Resistance - 0xFFFF
 *
 *  Infinite Resistance - 0xEA60 (60000 = 10 MOhms)- normal values should be less than this
 *
 *  Negative Resistance - 0x0258
 *
 */

/*----------------------------------------------------------------------------
 * Function Prototypes
 *----------------------------------------------------------------------------*/

void imd_enableMonitoring(const IMD_t* restrict imd);
void imd_disableMonitoring(const IMD_t* restrict imd);
void imd_busVoltage(const IMD_t* restrict imd);
void imd_evVoltage(const IMD_t* restrict imd);
void imd_positiveResistance(const IMD_t* restrict imd);
void imd_negativeResistance(const IMD_t* restrict imd);
void imd_ioStatus(const IMD_t* restrict imd);
void imd_setInsulationMonitoring(const IMD_t* restrict imd, bool value);

/*----------------------------------------------------------------------------
 * Public Instantiations
 *----------------------------------------------------------------------------*/

extern volatile IMD_t IMD1;
extern volatile IMD_t IMD2;

/*----------------------------------------------------------------------------
 * Public Prototypes
 *----------------------------------------------------------------------------*/

void imd_getData(uint8_t* slaveAddress, uint16_t* startAddress, uint16_t* result16, uint32_t* result32);

#endif /* INC_IMD_H_ */
