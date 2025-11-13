/*
 * imd.c
 *
 *  Created on: Jul 12, 2023
 *  Revised on: Oct 2, 2023
 *      Author: Kaushan Ranasinghe
 */

#include "imd.h"
#include "modbusRTU.h"

/*==============================================================================
 *  Section:  IMD Stack
 *
 *  Description: Configure device parameters here.
 *============================================================================*/

/*----------------------------------------------------------------------------
 * Public Definitions
 *----------------------------------------------------------------------------*/

volatile IMD_t IMD1 = {0};
volatile IMD_t IMD2 = {0};

/*----------------------------------------------------------------------------
 * Private Definitions
 *----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------
 * Get / Set Handles
 *----------------------------------------------------------------------------*/

void imd_getData(uint8_t* slaveAddress, uint16_t* startAddress, uint16_t* result16, uint32_t* result32)
{
	IMD_t* obj;

	switch(*slaveAddress)
	{
		case IMD1_SLAVE_ADDRESS:
			obj = (IMD_t*)&IMD1;
			break;

		case IMD2_SLAVE_ADDRESS:
			obj = (IMD_t*)&IMD2;
			break;

		default:
			return;
	}

	switch(*startAddress)
	{
		case IMD_BUS_VOLTAGE:
			obj->data.busVoltage = ((float)*result16 / 10);
			break;

		case IMD_POSITIVE_RESISTANCE:
			obj->data.positiveResistance = *result16;
			break;

		case IMD_NEGATIVE_RESISTANCE:
			obj->data.positiveResistance = *result16;
			break;

		case IMD_EV_SIDE_VOLTAGE:
			obj->data.busVoltage = ((float)*result16 / 10);
			break;

		case IMD_IO_STATUS:
			obj->data.statusRegister.all = (uint8_t)(*result16 & 0xFF);
			break;

		case IMD_INSULATION_MONITORING_CONTROL:
			obj->data.insulationMonitoringControl = *result16;
			break;
	}
}

/*----------------------------------------------------------------------------
 * Functions
 *----------------------------------------------------------------------------*/

void imd_enableMonitoring(const IMD_t* restrict imd)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = imd->slaveAddress;
	obj.adu.pdu.functionCode = WRITE_SINGLE_REGISTER;
	obj.adu.pdu.startAddress = IMD_INSULATION_MONITORING_CONTROL;
	obj.adu.pdu.numberOfRegisters = HWORD_1;
	obj.adu.pdu.writeData[0] = imd->insulationMonitoringControl;
	ModbusRTU_queueSerial(&obj);
}

void imd_disableMonitoring(const IMD_t* restrict imd)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = imd->slaveAddress;
	obj.adu.pdu.functionCode = WRITE_SINGLE_REGISTER;
	obj.adu.pdu.startAddress = IMD_INSULATION_MONITORING_CONTROL;
	obj.adu.pdu.numberOfRegisters = HWORD_1;
	obj.adu.pdu.writeData[0] = IMD_DISABLE;
	ModbusRTU_queueSerial(&obj);
}


void imd_busVoltage(const IMD_t* restrict imd)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = imd->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = IMD_BUS_VOLTAGE;
	obj.adu.pdu.numberOfRegisters = HWORD_1;
	ModbusRTU_queueSerial(&obj);
}

void imd_evVoltage(const IMD_t* restrict imd)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = imd->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = IMD_EV_SIDE_VOLTAGE;
	obj.adu.pdu.numberOfRegisters = HWORD_1;
	ModbusRTU_queueSerial(&obj);
}

void imd_positiveResistance(const IMD_t* restrict imd)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = imd->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = IMD_POSITIVE_RESISTANCE;
	obj.adu.pdu.numberOfRegisters = HWORD_1;
	ModbusRTU_queueSerial(&obj);
}

void imd_negativeResistance(const IMD_t* restrict imd)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = imd->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = IMD_NEGATIVE_RESISTANCE;
	obj.adu.pdu.numberOfRegisters = HWORD_1;
	ModbusRTU_queueSerial(&obj);
}

void imd_ioStatus(const IMD_t* restrict imd)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = imd->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = IMD_IO_STATUS;
	obj.adu.pdu.numberOfRegisters = HWORD_1;
	ModbusRTU_queueSerial(&obj);
}

/*----------------------------------------------------------------------------
 * Support Functions
 *----------------------------------------------------------------------------*/

void imd_setInsulationMonitoring(const IMD_t* restrict imd, bool value)
{
	if(value)
	{
		imd_enableMonitoring(imd);
	}
	else
	{
		imd_disableMonitoring(imd);
	}
}
