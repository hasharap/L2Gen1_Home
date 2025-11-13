/*
 * eMeter.c
 *
 *  Created on: Jul 9, 2023
 *  Revised on: Oct 2, 2023
 *      Author: Kaushan Ranasinghe
 */

#include "eMeter.h"
#include "modbusRTU.h"

/*==============================================================================
 *  Section:  ADL Energy Meter Stack
 *
 *  Description: Configure device parameters here.
 *============================================================================*/

/*----------------------------------------------------------------------------
 * Public Definitions
 *----------------------------------------------------------------------------*/

volatile ADL_t ADL = {0};

volatile bool emeter_bootup = false;

/*----------------------------------------------------------------------------
 * Private Definitions
 *----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------
 * Get / Set Handles
 *----------------------------------------------------------------------------*/

void adl_getData(uint8_t* slaveAddress, uint16_t* startAddress, uint16_t* result16, uint32_t* result32)
{
	ADL_t* obj;

	switch(*slaveAddress)
	{
		case ADL_SLAVE_ADDRESS:
			obj = (ADL_t*)&ADL;
			break;

		default:
			return;
	}

	switch(*startAddress)
	{
		case VOLTAGE_OF_A_PHASE:
			obj->data.voltage_PhaseA = (*result16);
			break;

		case VOLTAGE_OF_B_PHASE:
			obj->data.voltage_PhaseB = (*result16);
			break;

		case VOLTAGE_OF_C_PHASE:
			obj->data.voltage_PhaseC = (*result16);
			break;

		case CURRENT_OF_A_PHASE:
#if ADL200_ENABLE
			obj->data.current_PhaseA = (*result16);
#else
			obj->data.current_PhaseA = (*result16);
#endif
			break;

		case CURRENT_OF_B_PHASE:
#if ADL200_ENABLE
			obj->data.current_PhaseB = (*result16);
#else
			obj->data.current_PhaseB = (*result16);
#endif
			break;

		case CURRENT_OF_C_PHASE:
#if ADL200_ENABLE
			obj->data.current_PhaseC = (*result16);
#else
			obj->data.current_PhaseC = (*result16);
#endif
			break;

		case FREQUENCY:
			obj->data.freqeuncy = (*result16);
			break;

		case CURRENT_TOTAL_ACTIVE_ENERGY:
			obj->data.activeEnergy = *result32;
			break;

		case TOTAL_ACTIVE_POWER:
#if ADL200_ENABLE
			obj->data.activePower = *result16;
#else
			obj->data.activePower = *result32;
#endif
			emeter_bootup = true;
			break;

		default:
			break;
	}
}

/*----------------------------------------------------------------------------
 * Functions
 *----------------------------------------------------------------------------*/

void adl_TotalActiveEnergy(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = CURRENT_TOTAL_ACTIVE_ENERGY;
	obj.adu.pdu.numberOfRegisters = HWORD_2;
	ModbusRTU_queueSerial(&obj);
}

void adl_SpikeTotalActiveEnergy(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = CURRENT_SPIKE_TOTAL_ACTIVE_ENERGY;
	obj.adu.pdu.numberOfRegisters = HWORD_2;
	ModbusRTU_queueSerial(&obj);
}

void adl_PeakTotalActiveEnergy(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = CURRENT_PEAK_TOTAL_ACTIVE_ENERGY;
	obj.adu.pdu.numberOfRegisters = HWORD_2;
	ModbusRTU_queueSerial(&obj);
}

void adl_FlatTotalActiveEnergy(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = CURRENT_FLAT_TOTAL_ACTIVE_ENERGY;
	obj.adu.pdu.numberOfRegisters = HWORD_2;
	ModbusRTU_queueSerial(&obj);
}

void adl_ValleyTotalActiveEnergy(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = CURRENT_VALLEY_TOTAL_ACTIVE_ENERGY;
	obj.adu.pdu.numberOfRegisters = HWORD_2;
	ModbusRTU_queueSerial(&obj);
}

void adl_ForwardActiveTotalEnergy(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = CURRENT_FORWARD_ACTIVE_TOTAL_ENERGY;
	obj.adu.pdu.numberOfRegisters = HWORD_2;
	ModbusRTU_queueSerial(&obj);
}

void adl_ForwardActiveSpikeEnergy(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = CURRENT_FORWARD_ACTIVE_SPIKE_ENERGY;
	obj.adu.pdu.numberOfRegisters = HWORD_2;
	ModbusRTU_queueSerial(&obj);
}

void adl_ForwardActivePeakEnergy(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = CURRENT_FORWARD_ACTIVE_PEAK_ENERGY;
	obj.adu.pdu.numberOfRegisters = HWORD_2;
	ModbusRTU_queueSerial(&obj);
}

void adl_ForwardActiveFlatEnergy(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = CURRENT_FORWARD_ACTIVE_FLAT_ENERGY;
	obj.adu.pdu.numberOfRegisters = HWORD_2;
	ModbusRTU_queueSerial(&obj);
}

void adl_ForwardActiveValleyEnergy(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = CURRENT_FORWARD_ACTIVE_VALLEY_ENERGY;
	obj.adu.pdu.numberOfRegisters = HWORD_2;
	ModbusRTU_queueSerial(&obj);
}

void adl_ReversingActiveTotalEnergy(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = CURRENT_REVERSING_ACTIVE_TOTAL_ENERGY;
	obj.adu.pdu.numberOfRegisters = HWORD_2;
	ModbusRTU_queueSerial(&obj);
}

void adl_ReversingActiveSpikeEnergy(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = CURRENT_REVERSING_ACTIVE_SPIKE_ENERGY;
	obj.adu.pdu.numberOfRegisters = HWORD_2;
	ModbusRTU_queueSerial(&obj);
}

void adl_ReversingActivePeakEnergy(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = CURRENT_REVERSING_ACTIVE_PEAK_ENERGY;
	obj.adu.pdu.numberOfRegisters = HWORD_2;
	ModbusRTU_queueSerial(&obj);
}

void adl_ReversingActiveFlatEnergy(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = CURRENT_REVERSING_ACTIVE_FLAT_ENERGY;
	obj.adu.pdu.numberOfRegisters = HWORD_2;
	ModbusRTU_queueSerial(&obj);
}

void adl_ReversingActiveValleyEnergy(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = CURRENT_REVERSING_ACTIVE_VALLEY_ENERGY;
	obj.adu.pdu.numberOfRegisters = HWORD_2;
	ModbusRTU_queueSerial(&obj);
}

void adl_TotalReactiveEnergy(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = CURRENT_TOTAL_REACTIVE_ENERGY;
	obj.adu.pdu.numberOfRegisters = HWORD_2;
	ModbusRTU_queueSerial(&obj);
}

void adl_SpikeTotalReactiveEnergy(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = CURRENT_TOTAL_REACTIVE_SPIKE_ENERGY;
	obj.adu.pdu.numberOfRegisters = HWORD_2;
	ModbusRTU_queueSerial(&obj);
}

void adl_PeakTotalReactiveEnergy(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = CURRENT_TOTAL_REACTIVE_PEAK_ENERGY;
	obj.adu.pdu.numberOfRegisters = HWORD_2;
	ModbusRTU_queueSerial(&obj);
}

void adl_FlatTotalReactiveEnergy(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = CURRENT_TOTAL_REACTIVE_FLAT_ENERGY;
	obj.adu.pdu.numberOfRegisters = HWORD_2;
	ModbusRTU_queueSerial(&obj);
}

void adl_ValleyTotalReactiveEnergy(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = CURRENT_TOTAL_REACTIVE_VALLEY_ENERGY;
	obj.adu.pdu.numberOfRegisters = HWORD_2;
	ModbusRTU_queueSerial(&obj);
}

void adl_ForwardTotalReactiveEnergy(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = CURRENT_FORWARD_REACTIVE_TOTAL_ENERGY;
	obj.adu.pdu.numberOfRegisters = HWORD_2;
	ModbusRTU_queueSerial(&obj);
}

void adl_ForwardSpikeTotalReactiveEnergy(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = CURRENT_FORWARD_REACTIVE_SPIKE_ENERGY;
	obj.adu.pdu.numberOfRegisters = HWORD_2;
	ModbusRTU_queueSerial(&obj);
}

void adl_ForwardPeakTotalReactiveEnergy(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = CURRENT_FORWARD_REACTIVE_PEAK_ENERGY;
	obj.adu.pdu.numberOfRegisters = HWORD_2;
	ModbusRTU_queueSerial(&obj);
}

void adl_ForwardFlatTotalReactiveEnergy(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = CURRENT_FORWARD_REACTIVE_FLAT_ENERGY;
	obj.adu.pdu.numberOfRegisters = HWORD_2;
	ModbusRTU_queueSerial(&obj);
}

void adl_ForwardValleyTotalReactiveEnergy(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = CURRENT_FORWARD_REACTIVE_VALLEY_ENERGY;
	obj.adu.pdu.numberOfRegisters = HWORD_2;
	ModbusRTU_queueSerial(&obj);
}

void adl_ReversingTotalReactiveEnergy(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = CURRENT_REVERSING_REACTIVE_TOTAL_ENERGY;
	obj.adu.pdu.numberOfRegisters = HWORD_2;
	ModbusRTU_queueSerial(&obj);
}

void adl_ReversingSpikeTotalReactiveEnergy(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = CURRENT_REVERSING_REACTIVE_SPIKE_ENERGY;
	obj.adu.pdu.numberOfRegisters = HWORD_2;
	ModbusRTU_queueSerial(&obj);
}

void adl_ReversingPeakTotalReactiveEnergy(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = CURRENT_REVERSING_REACTIVE_PEAK_ENERGY;
	obj.adu.pdu.numberOfRegisters = HWORD_2;
	ModbusRTU_queueSerial(&obj);
}

void adl_ReversingFlatTotalReactiveEnergy(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = CURRENT_REVERSING_REACTIVE_FLAT_ENERGY;
	obj.adu.pdu.numberOfRegisters = HWORD_2;
	ModbusRTU_queueSerial(&obj);
}

void adl_ReversingValleyTotalReactiveEnergy(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = CURRENT_REVERSING_REACTIVE_VALLEY_ENERGY;
	obj.adu.pdu.numberOfRegisters = HWORD_2;
	ModbusRTU_queueSerial(&obj);
}

void adl_GetDateTime(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = DATE_TIME;
	obj.adu.pdu.numberOfRegisters = HWORD_3;
	ModbusRTU_queueSerial(&obj);
}

//void adl_SetDateTime(const ADL_t* restrict adl)
//{
//
//	Modbus_t obj = {0};
//	obj.adu.pdu.slaveAddress = adl->slaveAddress;
//	obj.adu.pdu.startAddress = DATE_TIME;
//	obj.adu.pdu.numberOfRegisters = HWORD_3;
//	adl[device].mbus_adl.pdu.adu.write_data[0] = setter.datetime.all[0];
//	adl[device].mbus_adl.pdu.adu.write_data[1] = setter.datetime.all[1];
//	adl[device].mbus_adl.pdu.adu.write_data[2] = setter.datetime.all[2];
//	modbus_Send(&device, &obj);
//}

void adl_GetCommParams(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = COMMUNICATION_PATH;
	obj.adu.pdu.numberOfRegisters = HWORD_1;
	ModbusRTU_queueSerial(&obj);
}

void adl_SetCommParams(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = MULTIPLE_REGISTER_WRITE;
	obj.adu.pdu.startAddress = COMMUNICATION_PATH;
	obj.adu.pdu.numberOfRegisters = HWORD_1;
//	adl.adu.write_data[0] = setter.comm_params.all;
	ModbusRTU_queueSerial(&obj);
}

void adl_GetVoltageTransfer(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = VOLTAGE_TRANSFER;
	obj.adu.pdu.numberOfRegisters = HWORD_1;
	ModbusRTU_queueSerial(&obj);
}
void adl_SetVoltageTransfer(const ADL_t* restrict adl, uint16_t value)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = MULTIPLE_REGISTER_WRITE;
	obj.adu.pdu.startAddress = VOLTAGE_TRANSFER;
	obj.adu.pdu.numberOfRegisters = HWORD_1;
//	adl.adu.write_data[0] = value;
	ModbusRTU_queueSerial(&obj);
}

void adl_GetCurrentTransfer(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = CURRENT_TRANSFER;
	obj.adu.pdu.numberOfRegisters = HWORD_1;
	ModbusRTU_queueSerial(&obj);
}
void adl_SetCurrentTransfer(const ADL_t* restrict adl, uint16_t value)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = MULTIPLE_REGISTER_WRITE;
	obj.adu.pdu.startAddress = CURRENT_TRANSFER;
	obj.adu.pdu.numberOfRegisters = HWORD_1;
//	adl.adu.write_data[0] = value;
	ModbusRTU_queueSerial(&obj);
}

//void adl_SetThresholdStateOfLossVoltage(const ADL_t* restrict adl)
//{
//	Modbus_t obj = {0};
//	obj.adu.pdu.slaveAddress = adl->slaveAddress;
//	obj.adu.pdu.functionCode = MULTIPLE_REGISTER_WRITE;
//	obj.adu.pdu.startAddress = THRESHOLD_AND_STATE_OF_LOSS_VOLTAGE;
//	obj.adu.pdu.numberOfRegisters = HWORD_1;
//	adl[device].mbus_adl.pdu.adu.write_data[0] = setter.threshhold_state_of_loss_voltage.threshhold_voltage;
//	modbus_Send(&device, &obj);
//}

void adl_GetThresholdStateOfLossVoltage(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = THRESHOLD_AND_STATE_OF_LOSS_VOLTAGE;
	obj.adu.pdu.numberOfRegisters = HWORD_1;
	ModbusRTU_queueSerial(&obj);
}

void adl_GetRunningState(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = RUNNING_STATE;
	obj.adu.pdu.numberOfRegisters = HWORD_1;
	ModbusRTU_queueSerial(&obj);
}

//void adl_SetRunningState(const ADL_t* restrict adl)
//{
//	Modbus_t obj = {0};
//	obj.adu.pdu.slaveAddress = adl->slaveAddress;
//	obj.adu.pdu.functionCode = MULTIPLE_REGISTER_WRITE;
//	obj.adu.pdu.startAddress = RUNNING_STATE;
//	obj.adu.pdu.numberOfRegisters = HWORD_1;
//	adl[device].mbus_adl.pdu.adu.write_data[0] = setter.running_state.all;
//	modbus_Send(&device, &obj);
//}

void adl_PulseConstant(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = PULSE_CONSTANT;
	obj.adu.pdu.numberOfRegisters = HWORD_1;
	ModbusRTU_queueSerial(&obj);
}

void adl_VoltagePhaseA(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = VOLTAGE_OF_A_PHASE;
	obj.adu.pdu.numberOfRegisters = HWORD_1;
	ModbusRTU_queueSerial(&obj);
}

void adl_VoltagePhaseB(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = VOLTAGE_OF_B_PHASE;
	obj.adu.pdu.numberOfRegisters = HWORD_1;
	ModbusRTU_queueSerial(&obj);
}

void adl_VoltagePhaseC(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = VOLTAGE_OF_C_PHASE;
	obj.adu.pdu.numberOfRegisters = HWORD_1;
	ModbusRTU_queueSerial(&obj);
}

void adl_CurrentPhaseA(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = CURRENT_OF_A_PHASE;
	obj.adu.pdu.numberOfRegisters = HWORD_1;
	ModbusRTU_queueSerial(&obj);
}

void adl_CurrentPhaseB(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = CURRENT_OF_B_PHASE;
	obj.adu.pdu.numberOfRegisters = HWORD_1;
	ModbusRTU_queueSerial(&obj);
}

void adl_CurrentPhaseC(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = CURRENT_OF_C_PHASE;
	obj.adu.pdu.numberOfRegisters = HWORD_1;
	ModbusRTU_queueSerial(&obj);
}

void adl_Frequency(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = FREQUENCY;
	obj.adu.pdu.numberOfRegisters = HWORD_1;
	ModbusRTU_queueSerial(&obj);
}

void adl_VoltagePhaseAtoB(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = VOLTAGE_BETWEEN_A_B;
	obj.adu.pdu.numberOfRegisters = HWORD_1;
	ModbusRTU_queueSerial(&obj);
}

void adl_VoltagePhaseCtoB(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = VOLTAGE_BETWEEN_C_B;
	obj.adu.pdu.numberOfRegisters = HWORD_1;
	ModbusRTU_queueSerial(&obj);
}

void adl_VoltagePhaseAtoC(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = VOLTAGE_BETWEEN_A_C;
	obj.adu.pdu.numberOfRegisters = HWORD_1;
	ModbusRTU_queueSerial(&obj);
}


// Damian
void adl_ForwardMaximumActiveDemand(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = FORWARD_ACTIVE_MAXIMUM_DEMAND;
	obj.adu.pdu.numberOfRegisters = HWORD_1;
	ModbusRTU_queueSerial(&obj);
}

void adl_TimeForForwardMaximumActiveDemand(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = TIME_OF_OCCURRENCE_FOR_THE_FORWARD_ACTIVE_MAXIMUM_AMOUNT;
	obj.adu.pdu.numberOfRegisters = HWORD_2;
	ModbusRTU_queueSerial(&obj);
}

void adl_ReverseMaximumActiveDemand(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = REVERSING_ACTIVE_MAXIMUM_DEMAND;
	obj.adu.pdu.numberOfRegisters = HWORD_1;
	ModbusRTU_queueSerial(&obj);
}

void adl_TimeForReverseMaximumActiveDemand(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = TIME_OF_OCCURRENCE_FOR_THE_REVERSING_ACTIVE_MAXIMUM_AMOUNT;
	obj.adu.pdu.numberOfRegisters = HWORD_2;
	ModbusRTU_queueSerial(&obj);
}

void adl_ForwardMaximumReactiveDemand(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = MAXIMUM_FORWARD_DEMAND_FOR_REACTIVE_POWER;
	obj.adu.pdu.numberOfRegisters = HWORD_1;
	ModbusRTU_queueSerial(&obj);
}

void adl_TimeForForwardMaximumReactiveDemand(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = TIME_OF_OCCURRENCE_FOR_THE_FORWARD_REACTIVE_MAXIMUM_AMOUNT;
	obj.adu.pdu.numberOfRegisters = HWORD_2;
	ModbusRTU_queueSerial(&obj);
}

void adl_ReverseMaximumReactiveDemand(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = MAXIMUM_REVERSING_DEMAND_FOR_REACTIVE_POWER;
	obj.adu.pdu.numberOfRegisters = HWORD_1;
	ModbusRTU_queueSerial(&obj);
}

void adl_TimeForReverseMaximumReactiveDemand(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = TIME_OF_OCCURRENCE_FOR_THE_REVERSING_REACTIVE_MAXIMUM_AMOUNT;
	obj.adu.pdu.numberOfRegisters = HWORD_1;
	ModbusRTU_queueSerial(&obj);
}

void adl_ForwardActiveEnergyPhaseA(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = FORWARD_ACTIVE_ENERGY_OF_A_PHASE;
	obj.adu.pdu.numberOfRegisters = HWORD_2;
	ModbusRTU_queueSerial(&obj);
}

void adl_ForwardActiveEnergyPhaseB(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = FORWARD_ACTIVE_ENERGY_OF_B_PHASE;
	obj.adu.pdu.numberOfRegisters = HWORD_2;
	ModbusRTU_queueSerial(&obj);
}

void adl_ForwardActiveEnergyPhaseC(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = FORWARD_ACTIVE_ENERGY_OF_C_PHASE;
	obj.adu.pdu.numberOfRegisters = HWORD_2;
	ModbusRTU_queueSerial(&obj);
}

void adl_ZeroSequenceCurrent(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = ZERO_SEQUENCE_CURRENT;
	obj.adu.pdu.numberOfRegisters = HWORD_1;
	ModbusRTU_queueSerial(&obj);
}

void adl_VoltageImbalance(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = VOLTAGE_IMBALANCE;
	obj.adu.pdu.numberOfRegisters = HWORD_1;
	ModbusRTU_queueSerial(&obj);
}

void adl_CurrentImbalance(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = CURRENT_IMBALANCE ;
	obj.adu.pdu.numberOfRegisters = HWORD_1;
	ModbusRTU_queueSerial(&obj);
}

void adl_ActivePowerPhaseA(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = ACTIVE_POWER_PHASE_A  ;
	obj.adu.pdu.numberOfRegisters = HWORD_2;
	ModbusRTU_queueSerial(&obj);
}

void adl_ActivePowerPhaseB(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = ACTIVE_POWER_PHASE_B   ;
	obj.adu.pdu.numberOfRegisters = HWORD_2;
	ModbusRTU_queueSerial(&obj);
}

void adl_ActivePowerPhaseC(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = ACTIVE_POWER_PHASE_C   ;
	obj.adu.pdu.numberOfRegisters = HWORD_2;
	ModbusRTU_queueSerial(&obj);
}

void adl_TotalActivePower(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = TOTAL_ACTIVE_POWER  ;

#if ADL200_ENABLE
	obj.adu.pdu.numberOfRegisters = HWORD_1;
#else
	obj.adu.pdu.numberOfRegisters = HWORD_2;
#endif

	ModbusRTU_queueSerial(&obj);
}

void adl_ReactivePowerPhaseA(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = REACTIVE_POWER_PHASE_A  ;
	obj.adu.pdu.numberOfRegisters = HWORD_2;
	ModbusRTU_queueSerial(&obj);
}

void adl_ReactivePowerPhaseB(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = REACTIVE_POWER_PHASE_B   ;
	obj.adu.pdu.numberOfRegisters = HWORD_2;
	ModbusRTU_queueSerial(&obj);
}

void adl_ReactivePowerPhaseC(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = REACTIVE_POWER_PHASE_C   ;
	obj.adu.pdu.numberOfRegisters = HWORD_2;
	ModbusRTU_queueSerial(&obj);
}

void adl_TotalReactivePower(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = TOTAL_REACTIVE_POWER  ;
	obj.adu.pdu.numberOfRegisters = HWORD_2;
	ModbusRTU_queueSerial(&obj);
}

void adl_ApparentPowerPhaseA(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = APPARENT_POWER_PHASE_A  ;
	obj.adu.pdu.numberOfRegisters = HWORD_2;
	ModbusRTU_queueSerial(&obj);
}

void adl_ApparentPowerPhaseB(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = APPARENT_POWER_PHASE_B   ;
	obj.adu.pdu.numberOfRegisters = HWORD_2;
	ModbusRTU_queueSerial(&obj);
}

void adl_ApparentPowerPhaseC(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = APPARENT_POWER_PHASE_C   ;
	obj.adu.pdu.numberOfRegisters = HWORD_2;
	ModbusRTU_queueSerial(&obj);
}

void adl_TotalApparentPower(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = TOTAL_APPARENT_POWER  ;
	obj.adu.pdu.numberOfRegisters = HWORD_2;
	ModbusRTU_queueSerial(&obj);
}

void adl_PowerFactorPhaseA(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = POWER_FACTOR_PHASE_A    ;
	obj.adu.pdu.numberOfRegisters = HWORD_1;
	ModbusRTU_queueSerial(&obj);
}

void adl_PowerFactorPhaseB(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = POWER_FACTOR_PHASE_A    ;
	obj.adu.pdu.numberOfRegisters = HWORD_1;
	ModbusRTU_queueSerial(&obj);
}

void adl_PowerFactorPhaseC(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = POWER_FACTOR_PHASE_C   ;
	obj.adu.pdu.numberOfRegisters = HWORD_1;
	ModbusRTU_queueSerial(&obj);
}

void adl_TotalPowerFactor(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = TOTAL_POWER_FACTOR  ;
	obj.adu.pdu.numberOfRegisters = HWORD_1;
	ModbusRTU_queueSerial(&obj);
}

void adl_DailyMaximumForwardActiveDemand(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress =MAXIMUM_FORWARD_ACTIVE_DEMAND_A_DAY ;
	obj.adu.pdu.numberOfRegisters = HWORD_1;
	ModbusRTU_queueSerial(&obj);
}

void adl_TimeOfDailyMaximumForwardActiveDemand(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = MAXIMUM_FORWARD_ACTIVE_DEMAND_A_DAY_OCCUR_TIME ;
	obj.adu.pdu.numberOfRegisters = HWORD_1;
	ModbusRTU_queueSerial(&obj);
}

void adl_DailyMaximumReverseActiveDemand(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress =MAXIMUM_REVERSING_ACTIVE_DEMAND_A_DAY;
	obj.adu.pdu.numberOfRegisters = HWORD_1;
	ModbusRTU_queueSerial(&obj);
}

void adl_TimeOfDailyMaximumReverseActiveDemand(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = MAXIMUM_REVERSING_ACTIVE_DEMAND_A_DAY_OCCUR_TIME ;
	obj.adu.pdu.numberOfRegisters = HWORD_1;
	ModbusRTU_queueSerial(&obj);
}

void adl_DailyMaximumForwardReactiveDemand(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress =MAXIMUM_FORWARD_REACTIVE_DEMAND_A_DAY;
	obj.adu.pdu.numberOfRegisters = HWORD_1;
	ModbusRTU_queueSerial(&obj);
}

void adl_TimeOfDailyMaximumForwardReactiveDemand(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = MAXIMUM_FORWARD_REACTIVE_DEMAND_A_DAY_OCCUR_TIME ;
	obj.adu.pdu.numberOfRegisters = HWORD_1;
	ModbusRTU_queueSerial(&obj);
}

void adl_DailyMaximumReverseReactiveDemand(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress =MAXIMUM_REVERSING_REACTIVE_DEMAND_A_DAY;
	obj.adu.pdu.numberOfRegisters = HWORD_1;
	ModbusRTU_queueSerial(&obj);
}

void adl_TimeOfDailyMaximumReverseReactiveDemand(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = MAXIMUM_REVERSING_REACTIVE_DEMAND_A_DAY_OCCUR_TIME;
	obj.adu.pdu.numberOfRegisters = HWORD_1;
	ModbusRTU_queueSerial(&obj);
}

void adl_LastDayMaximumForwardActiveDemand(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress =MAXIMUM_FORWARD_ACTIVE_DEMAND_LAST_DAY ;
	obj.adu.pdu.numberOfRegisters = HWORD_1;
	ModbusRTU_queueSerial(&obj);
}

void adl_TimeOfLastDayMaximumForwardActiveDemand(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = MAXIMUM_FORWARD_ACTIVE_DEMAND_LAST_DAY_OCCUR_TIME ;
	obj.adu.pdu.numberOfRegisters = HWORD_1;
	ModbusRTU_queueSerial(&obj);
}

void adl_LastDayMaximumReverseActiveDemand(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress =MAXIMUM_REVERSING_ACTIVE_DEMAND_LAST_DAY ;
	obj.adu.pdu.numberOfRegisters = HWORD_1;
	ModbusRTU_queueSerial(&obj);
}

void adl_TimeOfLastDayMaximumReverseActiveDemand(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = MAXIMUM_REVERSING_ACTIVE_DEMAND_LAST_DAY_OCCUR_TIME;
	obj.adu.pdu.numberOfRegisters = HWORD_1;
	ModbusRTU_queueSerial(&obj);
}

void adl_LastDayMaximumForwardReactiveDemand(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress =MAXIMUM_FORWARD_REACTIVE_DEMAND_LAST_DAY ;
	obj.adu.pdu.numberOfRegisters = HWORD_1;
	ModbusRTU_queueSerial(&obj);
}

void adl_TimeOfLastDayMaximumForwardReactiveDemand(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress; obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = MAXIMUM_FORWARD_REACTIVE_DEMAND_LAST_DAY_OCCUR_TIME ;
	obj.adu.pdu.numberOfRegisters = HWORD_1;
	ModbusRTU_queueSerial(&obj);
}

void adl_LastDayMaximumReverseReactiveDemand(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress =MAXIMUM_REVERSING_REACTIVE_DEMAND_LAST_DAY ;
	obj.adu.pdu.numberOfRegisters = HWORD_1;
	ModbusRTU_queueSerial(&obj);
}

void adl_TimeOfLastDayMaximumReverseReactiveDemand(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = MAXIMUM_REVERSING_REACTIVE_DEMAND_LAST_DAY_OCCUR_TIME;
	obj.adu.pdu.numberOfRegisters = HWORD_1;
	ModbusRTU_queueSerial(&obj);
}

void adl_CurrentForwardActiveDemand(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = CURRENT_FORWARD_ACTIVE_DEMAND;
	obj.adu.pdu.numberOfRegisters = HWORD_1;
	ModbusRTU_queueSerial(&obj);
}

void adl_CurrentReverseActiveDemand(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = CURRENT_REVERSE_ACTIVE_DEMAND ;
	obj.adu.pdu.numberOfRegisters = HWORD_1;
	ModbusRTU_queueSerial(&obj);
}

void adl_CurrentForwardRectiveDemand(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = CURRENT_FORWARD_REACTIVE_DEMAND;
	obj.adu.pdu.numberOfRegisters = HWORD_1;
	ModbusRTU_queueSerial(&obj);
}

void adl_CurrentReverseRectiveDemand(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = CURRENT_REVERESE_REACTIVE_DEMAND;
	obj.adu.pdu.numberOfRegisters = HWORD_1;
	ModbusRTU_queueSerial(&obj);
}

void adl_MaximumVoltagePhaseA(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = MAXIMUM_VOLTAGE_ON_PHASE_A ;
	obj.adu.pdu.numberOfRegisters = HWORD_1;
	ModbusRTU_queueSerial(&obj);
}

void adl_MaximumVoltagePhaseAOccurDate(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = MAXIMUM_VOLTAGE_ON_PHASE_A_OCCUR_DATE ;
	obj.adu.pdu.numberOfRegisters = HWORD_1;
	ModbusRTU_queueSerial(&obj);
}

void adl_MaximumVoltagePhaseAOccurTime(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = MAXIMUM_VOLTAGE_ON_PHASE_A_OCCUR_TIME ;
	obj.adu.pdu.numberOfRegisters = HWORD_1;
	ModbusRTU_queueSerial(&obj);
}

void adl_MaximumVoltagePhaseBandOccurTime(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = MAXIMUM_VOLTAGE_ON_PHASE_B_AND_OCCUR_TIME;
	obj.adu.pdu.numberOfRegisters = HWORD_3;
	ModbusRTU_queueSerial(&obj);
}

void adl_MaximumVoltagePhaseCandOccurTime(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = MAXIMUM_VOLTAGE_ON_PHASE_C_AND_OCCUR_TIME;
	obj.adu.pdu.numberOfRegisters = HWORD_3;
	ModbusRTU_queueSerial(&obj);
}

void adl_MaximumCurrentPhaseAandOccurTime(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress =  MAXIMUM_CURRENT_ON_PHASE_A_AND_OCCUR_TIME;
	obj.adu.pdu.numberOfRegisters = HWORD_3;
	ModbusRTU_queueSerial(&obj);
}

void adl_MaximumCurrentPhaseBandOccurTime(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress =  MAXIMUM_CURRENT_ON_PHASE_B_AND_OCCUR_TIME;
	obj.adu.pdu.numberOfRegisters = HWORD_3;
	ModbusRTU_queueSerial(&obj);
}

void adl_MaximumCurrentPhaseCandOccurTime(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress =  MAXIMUM_CURRENT_ON_PHASE_C_AND_OCCUR_TIME;
	obj.adu.pdu.numberOfRegisters = HWORD_3;
	ModbusRTU_queueSerial(&obj);
}

void adl_MaximumActivePowerPhaseA(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = MAXIMUM_ACTIVE_POWER_ON_PHASE_A ;
	obj.adu.pdu.numberOfRegisters = HWORD_2;
	ModbusRTU_queueSerial(&obj);
}

void adl_MaximumActivePowerPhaseAOccurDate(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = MAXIMUM_ACTIVE_POWER_ON_PHASE_A_OCCUR_DATE ;
	obj.adu.pdu.numberOfRegisters = HWORD_1;
	ModbusRTU_queueSerial(&obj);
}

void adl_MaximumActivePowerPhaseAOccurTime(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = MAXIMUM_ACTIVE_POWER_ON_PHASE_A_OCCUR_TIME ;
	obj.adu.pdu.numberOfRegisters = HWORD_1;
	ModbusRTU_queueSerial(&obj);
}

void adl_MaximumActivePowerPhaseBandOccurTime(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = MAXIMUM_ACTIVE_POWER_ON_PHASE_B_AND_OCCUR_TIME;
	obj.adu.pdu.numberOfRegisters = HWORD_4;
	ModbusRTU_queueSerial(&obj);
}

void adl_MaximumActivePowerPhaseCandOccurTime(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = MAXIMUM_ACTIVE_POWER_ON_PHASE_C_AND_OCCUR_TIME;
	obj.adu.pdu.numberOfRegisters = HWORD_4;
	ModbusRTU_queueSerial(&obj);
}

void adl_MaximumActivePowerAndOccurTime(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = MAXIMUM_ACTIVE_POWER_AND_OCCUR_TIME;
	obj.adu.pdu.numberOfRegisters = HWORD_4;
	ModbusRTU_queueSerial(&obj);
}

void adl_MaximumReactivePowerPhaseAandOccurTime(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = MAXIMUM_REACTIVE_POWER_ON_PHASE_A_AND_OCCUR_TIME ;
	obj.adu.pdu.numberOfRegisters = HWORD_4;
	ModbusRTU_queueSerial(&obj);
}

void adl_MaximumReactivePowerPhaseBandOccurTime(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress; obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress =MAXIMUM_REACTIVE_POWER_ON_PHASE_B_AND_OCCUR_TIME ;
	obj.adu.pdu.numberOfRegisters = HWORD_4;
	ModbusRTU_queueSerial(&obj);
}

void adl_MaximumReactivePowerPhaseCandOccurTime(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = MAXIMUM_REACTIVE_POWER_ON_PHASE_C_AND_OCCUR_TIME;
	obj.adu.pdu.numberOfRegisters = HWORD_4;
	ModbusRTU_queueSerial(&obj);
}

void adl_MaximumReactivePowerAndOccurTime(const ADL_t* restrict adl)
{
	Modbus_t obj = {0}; obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = MAXIMUM_REACTIVE_POWER_AND_OCCUR_TIME;
	obj.adu.pdu.numberOfRegisters = HWORD_4;
	ModbusRTU_queueSerial(&obj);
}

void adl_MaximumApparentPowerPhaseAandOccurTime(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = MAXIMUM_APPARENT_POWER_ON_PHASE_A_AND_OCCUR_TIME ;
	obj.adu.pdu.numberOfRegisters = HWORD_4;
	ModbusRTU_queueSerial(&obj);
}

void adl_MaximumApparentPowerPhaseBandOccurTime(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress =MAXIMUM_APPARENT_POWER_ON_PHASE_B_AND_OCCUR_TIME ;
	obj.adu.pdu.numberOfRegisters = HWORD_4;
	ModbusRTU_queueSerial(&obj);
}

void adl_MaximumApparentPowerPhaseCandOccurTime(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = MAXIMUM_APPARENT_POWER_ON_PHASE_C_AND_OCCUR_TIME;
	obj.adu.pdu.numberOfRegisters = HWORD_4;
	ModbusRTU_queueSerial(&obj);
}

void adl_MaximumApparentPowerAndOccurTime(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = MAXIMUM_APPARENT_POWER_AND_OCCUR_TIME;
	obj.adu.pdu.numberOfRegisters = HWORD_4;
	ModbusRTU_queueSerial(&obj);
}


void adl_MinimumVoltagePhaseAandOccurTime(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = MINIMUM_VOLTAGE_ON_PHASE_A_AND_OCCUR_TIME;
	obj.adu.pdu.numberOfRegisters = HWORD_3;
	ModbusRTU_queueSerial(&obj);
}

void adl_MinimumVoltagePhaseBandOccurTime(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = MINIMUM_VOLTAGE_ON_PHASE_B_AND_OCCUR_TIME;
	obj.adu.pdu.numberOfRegisters = HWORD_3;
	ModbusRTU_queueSerial(&obj);
}

void adl_MinimumVoltagePhaseCandOccurTime(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = MINIMUM_VOLTAGE_ON_PHASE_C_AND_OCCUR_TIME ;
	obj.adu.pdu.numberOfRegisters = HWORD_3;
	ModbusRTU_queueSerial(&obj);
}

void adl_MinimumCurrentPhaseAandOccurTime(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress =  MINIMUM_CURRENT_ON_PHASE_A_AND_OCCUR_TIME ;
	obj.adu.pdu.numberOfRegisters = HWORD_3;
	ModbusRTU_queueSerial(&obj);
}

void adl_MinimumCurrentPhaseBandOccurTime(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress =  MINIMUM_CURRENT_ON_PHASE_B_AND_OCCUR_TIME;
	obj.adu.pdu.numberOfRegisters = HWORD_3;
	ModbusRTU_queueSerial(&obj);
}

void adl_MinimumCurrentPhaseCandOccurTime(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress =  MINIMUM_CURRENT_ON_PHASE_C_AND_OCCUR_TIME;
	obj.adu.pdu.numberOfRegisters = HWORD_3;
	ModbusRTU_queueSerial(&obj);
}

void adl_MinimumActivePowerPhaseAandOccurTime(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = MINIMUM_ACTIVE_POWER_ON_PHASE_A_AND_OCCUR_TIME ;
	obj.adu.pdu.numberOfRegisters = HWORD_4;
	ModbusRTU_queueSerial(&obj);
}

void adl_MinimumActivePowerPhaseBandOccurTime(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = MINIMUM_ACTIVE_POWER_ON_PHASE_B_AND_OCCUR_TIME;
	obj.adu.pdu.numberOfRegisters = HWORD_4;
	ModbusRTU_queueSerial(&obj);
}

void adl_MinimumActivePowerPhaseCandOccurTime(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = MINIMUM_ACTIVE_POWER_ON_PHASE_C_AND_OCCUR_TIME;
	obj.adu.pdu.numberOfRegisters = HWORD_4;
	ModbusRTU_queueSerial(&obj);
}

void adl_MinimumActivePowerAndOccurTime(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = MINIMUM_ACTIVE_POWER_AND_OCCUR_TIME;
	obj.adu.pdu.numberOfRegisters = HWORD_4;
	ModbusRTU_queueSerial(&obj);
}

void adl_MinimumReactivePowerPhaseAandOccurTime(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = MINIMUM_REACTIVE_POWER_ON_PHASE_A_AND_OCCUR_TIME ;
	obj.adu.pdu.numberOfRegisters = HWORD_4;
	ModbusRTU_queueSerial(&obj);
}

void adl_MinimumReactivePowerPhaseBandOccurTime(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = MINIMUM_REACTIVE_POWER_ON_PHASE_B_AND_OCCUR_TIME;
	obj.adu.pdu.numberOfRegisters = HWORD_4;
	ModbusRTU_queueSerial(&obj);
}

void adl_MinimumReactivePowerPhaseCandOccurTime(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = MINIMUM_REACTIVE_POWER_ON_PHASE_C_AND_OCCUR_TIME;
	obj.adu.pdu.numberOfRegisters = HWORD_4;
	ModbusRTU_queueSerial(&obj);
}

void adl_MinimumReactivePowerAndOccurTime(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = MINIMUM_REACTIVE_POWER_AND_OCCUR_TIME;
	obj.adu.pdu.numberOfRegisters = HWORD_4;
	ModbusRTU_queueSerial(&obj);
}

void adl_MinimumApparentPowerPhaseAandOccurTime(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = MINIMUM_APPARENT_POWER_ON_PHASE_A_AND_OCCUR_TIME;
	obj.adu.pdu.numberOfRegisters = HWORD_4;
	ModbusRTU_queueSerial(&obj);
}

void adl_MinimumApparentPowerPhaseBandOccurTime(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = MINIMUM_APPARENT_POWER_ON_PHASE_B_AND_OCCUR_TIME;
	obj.adu.pdu.numberOfRegisters = HWORD_4;
	ModbusRTU_queueSerial(&obj);
}

void adl_MinimumApparentPowerPhaseCandOccurTime(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = MINIMUM_APPARENT_POWER_ON_PHASE_C_AND_OCCUR_TIME;
	obj.adu.pdu.numberOfRegisters = HWORD_4;
	ModbusRTU_queueSerial(&obj);
}

void adl_MinimumApparentPowerAndOccurTime(const ADL_t* restrict adl)
{
	Modbus_t obj = {0};
	obj.adu.pdu.slaveAddress = adl->slaveAddress;
	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
	obj.adu.pdu.startAddress = MINIMUM_APPARENT_POWER_AND_OCCUR_TIME;
	obj.adu.pdu.numberOfRegisters = HWORD_4;
	ModbusRTU_queueSerial(&obj);
}

//void adl_TemperatureT1(const ADL_t* restrict adl)
//{
//	Modbus_t obj = {0};
//	obj.adu.pdu.slaveAddress = adl->slaveAddress;
//	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
//	obj.adu.pdu.startAddress = T1_TEMPERATURE ;
//	obj.adu.pdu.numberOfRegisters = HWORD_4;
//	ModbusRTU_queueSerial(&obj);
//}
//
//void adl_TemperatureT2(const ADL_t* restrict adl)
//{
//	Modbus_t obj = {0};
//	obj.adu.pdu.slaveAddress = adl->slaveAddress;
//	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
//	obj.adu.pdu.startAddress = T2_TEMPERATURE ;
//	obj.adu.pdu.numberOfRegisters = HWORD_4;
//	ModbusRTU_queueSerial(&obj);
//}
//
//void adl_TemperatureT3(const ADL_t* restrict adl)
//{
//	Modbus_t obj = {0};
//	obj.adu.pdu.slaveAddress = adl->slaveAddress;
//	obj.adu.pdu.functionCode = READ_HOLDING_REGISTERS;
//	obj.adu.pdu.startAddress = T3_TEMPERATURE ;
//	obj.adu.pdu.numberOfRegisters = HWORD_4;
//	ModbusRTU_queueSerial(&obj);
//}

/*----------------------------------------------------------------------------
 * Support Functions
 *----------------------------------------------------------------------------*/

