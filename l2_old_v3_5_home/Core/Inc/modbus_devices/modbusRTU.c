/*
 * modbusRTU.c
 *
 *  Created on: Jul 9, 2023
 *  Revised on: Oct 10, 2023
 *      Author: Kaushan Ranasinghe
 */

#include "modbusRTU.h"
//#include "imd.h"
#include "eMeter.h"
//#include "crc.h"
#include "main.h"

#ifdef __has_include
    #if __has_include("tables.h") // CRC lookup tables
        #include "tables.h"
    #endif
#endif

/*----------------------------------------------------------------------------
 * Public Instantiations
 *----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------
 * Private Value Tables
 *----------------------------------------------------------------------------*/

typedef enum
{
	M_RxError = 0,
	M_RxIdle,
	M_RxSoF,
	M_RxData,
	M_RxEoF,
}MODBUS_RxSerialState_t;

/*----------------------------------------------------------------------------
 * Private Instantiations
 *----------------------------------------------------------------------------*/

volatile Modbus_MessageQueue Modbus_Queue = {0};

volatile static uint32_t txID = 0;
volatile static bool txFlag = false;
volatile static bool rxFlag = false;

volatile static uint8_t slaveAddress = 0;
volatile static uint32_t resultID = 0;

volatile static bool imdFlag = false;

volatile static uint16_t TxInterval = 0;
volatile static uint16_t TxCycleInterval = 0;
volatile static bool TxIntervalFlag = false;
volatile static uint16_t RxTimeoutInterval = 0;
volatile static bool RxTimeoutIntervalFlag = false;

volatile static uint32_t sniffTxInterval = 0;
volatile static bool sniffTimeoutFlag = false;
volatile static bool calculatePeriodFlag = true;

volatile static MODBUS_RxSerialState_t Modbus_RxSerialState = 2; // Initial state = RxSoF
volatile static uint32_t RxBusTimeout = 0;

volatile static char RxBuffer[100] = {0};
volatile static uint8_t RxBufferPos = 0;
volatile static bool RxPcktCpltFlag = false;

volatile static uint8_t RxRetryCount = 0;
volatile bool ModbusRxError = 0;

/*----------------------------------------------------------------------------
 * Private Prototypes
 *----------------------------------------------------------------------------*/

static uint16_t ModbusRTU_CRC(const uint8_t *data, size_t length);
static bool ModbusRTU_checksum(const uint8_t* restrict data, uint8_t length);
bool ModbusRTU_sniffTxCommand(const uint8_t* restrict TxData, ModbusID_t* obj, const uint8_t* restrict TxDataPos, uint32_t* restrict ID);

/*----------------------------------------------------------------------------
 * Timer Function
 *----------------------------------------------------------------------------*/

void ModbusRTU_Intick(void) // period in milliseconds
{
	switch(MODBUS_TICK_TIME)
	{
		case 1:

		case 10:
			RxBusTimeout++;
			sniffTxInterval++;
			TxCycleInterval++;

		case 30:

		case 50:

		case 100:
			TxInterval++;

		case 250:

		case 500:

		case 1000:
			RxTimeoutInterval++;

		default:
			break;
	}
}

/*----------------------------------------------------------------------------
 * Send / Receive Functions
 *----------------------------------------------------------------------------*/

void ModbusRTU_queueSerial(const Modbus_t* restrict obj)
{
	static uint8_t index = 0;

	ModbusID_t modbusDevice = {0};

	modbusDevice.ID.bits.slaveAddress = obj->adu.pdu.slaveAddress;
	modbusDevice.ID.bits.functionCode = obj->adu.pdu.functionCode;
	modbusDevice.ID.bits.startAddress = obj->adu.pdu.startAddress;

	for(uint8_t i = 0; i < MODBUS_QUEUE_SIZE; i++) // fills vacant slots in queue
	{
		if((Modbus_Queue.ID[i] == modbusDevice.ID.all) && (((Modbus_Queue.ActiveMask >> i) & 0x0001) == 1))
		{
			return; // change this to current index i if found to update earliest command with new information
		}

		if(Modbus_Queue.ActiveMask == 0)
		{
			index = 0;
		}
	}

	uint8_t i = 0;

	Modbus_Queue.ActiveMask |= (1 << index);
	Modbus_Queue.ID[index] = modbusDevice.ID.all;
	Modbus_Queue.TxData[index][i++] = modbusDevice.ID.bits.slaveAddress;
	Modbus_Queue.TxData[index][i++] = obj->adu.pdu.functionCode;
	Modbus_Queue.TxData[index][i++] = ((obj->adu.pdu.startAddress >> 8) & 0xFF);
	Modbus_Queue.TxData[index][i++] = (obj->adu.pdu.startAddress & 0xFF);

	if(obj->adu.pdu.functionCode == READ_HOLDING_REGISTERS || obj->adu.pdu.functionCode == READ_INPUT_REGISTERS
			|| obj->adu.pdu.functionCode == MULTIPLE_COIL_WRITE || obj->adu.pdu.functionCode == MULTIPLE_REGISTER_WRITE)
	{
		Modbus_Queue.TxData[index][i++] = ((obj->adu.pdu.numberOfRegisters >> 8) & 0xFF);
		Modbus_Queue.TxData[index][i++] = (obj->adu.pdu.numberOfRegisters & 0xFF);
	}

	if(obj->adu.pdu.functionCode == MULTIPLE_COIL_WRITE || obj->adu.pdu.functionCode == MULTIPLE_REGISTER_WRITE ||
			obj->adu.pdu.functionCode == WRITE_SINGLE_COIL_STATUS || obj->adu.pdu.functionCode == WRITE_SINGLE_REGISTER)
	{
		for (uint8_t j = 0; j < obj->adu.pdu.numberOfRegisters; j++)
		{
			Modbus_Queue.TxData[index][i++] = ((obj->adu.pdu.writeData[j] >> 8) & 0xFF);
			Modbus_Queue.TxData[index][i++] = (obj->adu.pdu.writeData[j] & 0xFF);
		}
	}

	uint16_t crc = ModbusRTU_CRC((uint8_t*)Modbus_Queue.TxData[index], i);

	Modbus_Queue.TxData[index][i++] = (crc & 0xFF);
	Modbus_Queue.TxData[index][i++] = ((crc >> 8) & 0xFF);
	Modbus_Queue.Length[index] = i;

	index++;

	if((index >= MODBUS_QUEUE_SIZE))
	{
		index = 0;
	}
}

static void ModbusRTU_TxCompleteService(const uint8_t* restrict index)
{
	txID = Modbus_Queue.ID[*index];
	RxTimeoutInterval = 0;
	txFlag = true;
	rxFlag = false;
}

static bool ModbusRTU_RxCompleteService(void)
{
	if((rxFlag) || (RxTimeoutInterval >= MODBUS_RX_TIMEOUT_INTERVAL))
	{
		if(RxTimeoutInterval >= MODBUS_RX_TIMEOUT_INTERVAL)
		{
//			memset((ADL_t*)&ADL.data,0,sizeof(ADL.data));
			RxRetryCount++;
		}
		else if(rxFlag)
		{
			RxRetryCount = 0;
		}

		return true;
	}
	else
	{
		return false;
	}
}

void ModbusRTU_transmitSerial(void (*TransmitFunction)(uint8_t*, uint16_t))
{
	if(TxInterval >= MODBUS_TRANSMIT_INTERVAL)
	{
		TxInterval = 0;
		static uint8_t instruction = 0;

		if ((Modbus_Queue.ActiveMask & (1 << instruction)) != 0x0) // check whether activation mask not zero
		{
			if(ModbusRTU_RxCompleteService())
			{
				TransmitFunction((uint8_t*)Modbus_Queue.TxData[instruction], Modbus_Queue.Length[instruction]);
				Modbus_Queue.ActiveMask &= ~(1 << instruction); // reset activation for instruction
				ModbusRTU_TxCompleteService(&instruction);
				instruction++; // update instruction starting point

				if((instruction == MODBUS_QUEUE_SIZE) || (Modbus_Queue.ActiveMask == 0))
				{
					instruction = 0;
				}
			}
		}
	}
}

#if !MODBUS_SNIFFER

void ModbusRTU_receiveSerial(const char* restrict data)
{
	static uint8_t RxCount = 0;
	bool RxBufferFlush = false;
	static uint8_t length = 0;

	ModbusID_t obj = {0};

	obj.ID.all = txID;

    if(RxBusTimeout >= MODBUS_RX_DEAD_TIME_INTERVAL) // dead time monitor
    {
        if(Modbus_RxSerialState != M_RxIdle)
		{
			memset((uint8_t*) RxBuffer, 0,
					(sizeof(RxBuffer) / sizeof(RxBuffer[0])));
			RxBufferPos = 0;
			length = 0;
			Modbus_RxSerialState = M_RxSoF;
			RxBufferFlush = false;
			RxCount = 0;
		}
    }

    RxBusTimeout = 0;

	switch(Modbus_RxSerialState)
	{
		case M_RxError:
			break;

		case M_RxIdle:
			RxCount = 0;
			break;

		case M_RxSoF: 									// Start of Frame
			if((*data == obj.ID.bits.slaveAddress) && (RxCount == 0))
			{
				RxBuffer[RxBufferPos++] = *data;
			}
			else if((*data == obj.ID.bits.functionCode) && (RxCount == 1))
			{
				RxBuffer[RxBufferPos++] = *data;
			}
			else if((*data < 100) && (RxCount == 2)) // 100 = (sizeof(RxBuffer)/sizeof(uint8_t))
			{
				RxBuffer[RxBufferPos++] = *data;
				length = (2 + RxBuffer[2]); // slave address + function code + number of registers + data
				Modbus_RxSerialState = M_RxData;
			}
			else
			{
				RxBufferFlush = true;
			}
			break;

		case M_RxData:
			RxBuffer[RxBufferPos++] = *data;

			if(RxCount >= length)
			{
				length += 2; // add CRC bytes
				Modbus_RxSerialState = M_RxEoF;
			}
			break;

		case M_RxEoF:										// End of Frame
			RxBuffer[RxBufferPos++] = *data;

			if(RxCount == length)
			{
				RxPcktCpltFlag = true;
				Modbus_RxSerialState = M_RxIdle;
			}
			else if(RxCount > length)
			{
				RxBufferFlush = true;
			}
			break;

		default:
			break;
	}

	RxCount++;

	if(RxBufferFlush)
	{
		memset((uint8_t*)RxBuffer,0,(sizeof(RxBuffer)/sizeof(RxBuffer[0])));
		RxBufferPos = 0;
		Modbus_RxSerialState = M_RxSoF;
		RxBufferFlush = false;
		RxCount = 0;
		length = 0;
	}
}


#else

void ModbusRTU_receiveSerial(const char* restrict data)
{
	static uint8_t RxBuffer[100] = {0};
	static uint8_t RxBufferPos = 0;
	static bool sniffTxCompleteFlag = false;
	static uint32_t ID = 0;
	uint8_t length = 0;

	ModbusID_t obj = {0};

	if(sniffTimeoutFlag) // catch tx header - milliseconds
	{
		sniffTimeoutFlag = false;
		memset(RxBuffer,0,sizeof(RxBuffer));
		RxBufferPos = 0;

		sniffTxCompleteFlag = false;
		txID = 0;
		resultID = 0;
		result16 = 0;
		result32 = 0;
		slaveAddress = 0;
	}

	sniffTxInterval = 0;
	RxBuffer[RxBufferPos++] = (uint8_t)*data;

	if(sniffTxCompleteFlag == false)
	{
		if(ModbusRTU_sniffTxCommand(RxBuffer, &obj, &RxBufferPos, &ID))
		{
			sniffTxCompleteFlag = true;
			memset(RxBuffer,0,sizeof(RxBuffer));
			RxBufferPos = 0;
			resultID = 0;
		}
		else
		{
			return;
		}
	}

	obj.ID.all = ID;

	if((RxBuffer[0] == obj.ID.bits.slaveAddress) && (RxBuffer[1] == obj.ID.bits.functionCode) && (RxBufferPos > 2)) // check - remove function code
	{
		length = (2 + RxBuffer[2] + 2); // slave address + function code + number of registers + data + crc

		if(RxBufferPos > 50)
		{
			sniffTxCompleteFlag = false;
			memset(RxBuffer,0,sizeof(RxBuffer));
			RxBufferPos = 0;
			resultID = 0;
		}
		else if(RxBufferPos > (length))
		{
			if(ModbusRTU_checksum(RxBuffer, (RxBufferPos)))
			{
				slaveAddress = 0;
				result16 = 0;
				result32 = 0;

				slaveAddress = RxBuffer[0];
				resultID = obj.ID.all;

				if(RxBuffer[2] == 2)
				{
					result16 = ((RxBuffer[3] << 8) | (RxBuffer[4]));
				}
				else if(RxBuffer[2] == 4)
				{
					result32 = ((RxBuffer[3] << 24) | (RxBuffer[4] << 16) | (RxBuffer[5] << 8) | (RxBuffer[6]));
				}

				rxFlag = true;
				sniffTxCompleteFlag = false;
				memset(RxBuffer,0,sizeof(RxBuffer));
				RxBufferPos = 0;
			}
			else
			{
				sniffTxCompleteFlag = false;
				memset(RxBuffer,0,sizeof(RxBuffer));
				RxBufferPos = 0;
				resultID = 0;
			}
		}
	}
}

#endif


bool ModbusRTU_sniffTxCommand(const uint8_t* restrict TxData, ModbusID_t* obj, const uint8_t* restrict TxDataPos, uint32_t* restrict ID)
{
	if(*TxDataPos > 7)
	{
		if(ModbusRTU_checksum(TxData, (*TxDataPos)))
		{
			obj->ID.bits.slaveAddress = TxData[0];
			obj->ID.bits.functionCode = TxData[1];
			obj->ID.bits.startAddress = ((TxData[2] << 8) | (TxData[3]));
			*ID = obj->ID.all;
			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		return false;
	}
}

/*----------------------------------------------------------------------------
 * Functions
 *----------------------------------------------------------------------------*/

#ifndef INC_TABLES_H_

static uint16_t ModbusRTU_CRC(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFF;

    while (length--)
    {
        crc ^= *data++;

        for (int i = 0; i < 8; ++i)
        {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}

#else

static uint16_t ModbusRTU_CRC(const uint8_t *data, size_t length)
{
    uint8_t crc_hi = 0xFF;
    uint8_t crc_lo = 0xFF;

    while (length--) {
        uint8_t index = crc_lo ^ *data++;
        crc_lo = crc_hi ^ table_crc_hi[index];
        crc_hi = table_crc_lo[index];
    }

    return (crc_hi << 8 | crc_lo);
}

#endif


static bool ModbusRTU_checksum(const uint8_t* restrict data, uint8_t length)
{
	uint16_t crc_calc = ModbusRTU_CRC(data, (length - 2));
	uint16_t crc_rx = (data[length - 1] << 8) | (data[length - 2]);
	if (crc_calc == crc_rx && crc_calc != 0 && crc_rx != 0)
	{
		return true;
	}
	else if (crc_calc != crc_rx && crc_calc != 0 && crc_rx != 0)
	{
		return false;
	}
	else
	{
		return false;
	}
}

static void ModbusRTU_getData(void)
{
	uint16_t result16 = 0;
	uint32_t result32 = 0;

	ModbusID_t objID = {0};

	if(RxPcktCpltFlag)
	{
		RxPcktCpltFlag = false;

		if(ModbusRTU_checksum((uint8_t*)RxBuffer, RxBufferPos))
		{
//			result16 = 0;
//			result32 = 0;

			if(RxBuffer[2] == 2)
			{
				result16 = ((uint16_t)(RxBuffer[3] << 8) | ((uint16_t)RxBuffer[4]));
			}
			else if(RxBuffer[2] == 4)
			{
				result32 = ((uint32_t)(RxBuffer[3] << 24) | ((uint32_t)RxBuffer[4] << 16) | ((uint32_t)RxBuffer[5] << 8) | ((uint32_t)RxBuffer[6]));
			}

			objID.ID.all = txID;

			if((objID.ID.bits.slaveAddress == RxBuffer[0]))
			{
				adl_getData(&objID.ID.bits.slaveAddress, &objID.ID.bits.startAddress, &result16, &result32);
//				imd_getData(&objID.ID.bits.slaveAddress, &objID.ID.bits.startAddress, &result16, &result32);
			}

			memset((uint8_t*)RxBuffer,0,(sizeof(RxBuffer)/sizeof(RxBuffer[0])));
			RxBufferPos = 0;
			Modbus_RxSerialState = M_RxSoF;
			rxFlag = true;
		}
		else
		{
			memset((uint8_t*)RxBuffer,0,(sizeof(RxBuffer)/sizeof(RxBuffer[0])));
			RxBufferPos = 0;
			Modbus_RxSerialState = M_RxSoF;
		}
	}
}

static void ModbusRTU_FaultService(void)
{
	if(RxRetryCount >= 5)
	{
		ModbusRxError = true;
		controlSide_data.errorStatus.bit.serialAError = 1;
	}
	else
	{
		ModbusRxError = false;
		controlSide_data.errorStatus.bit.serialAError = 0;
	}
}


void ModbusRTU_Init(void)
{
	ADL.slaveAddress = ADL_SLAVE_ADDRESS;
//	IMD1.slaveAddress = IMD1_SLAVE_ADDRESS;
//	IMD1.insulationMonitoringControl = IMD_ENABLE_DC1;
//	IMD2.slaveAddress = IMD2_SLAVE_ADDRESS;
//	IMD2.insulationMonitoringControl = IMD_ENABLE_DC2;
}

void ModbusRTU_functions(void)
{
	ModbusRTU_getData();
#if MODBUS_ERROR
	ModbusRTU_FaultService();
#endif

#if !MODBUS_SNIFFER

#if ADL_ENABLE
	if(!Modbus_Queue.ActiveMask && TxCycleInterval >= MODBUS_TRANSMIT_CYCLE)
	{
		TxCycleInterval = 0;
		adl_VoltagePhaseA((ADL_t*)&ADL);
		adl_VoltagePhaseB((ADL_t*)&ADL);
		adl_VoltagePhaseC((ADL_t*)&ADL);
		adl_CurrentPhaseA((ADL_t*)&ADL);
		adl_CurrentPhaseB((ADL_t*)&ADL);
		adl_CurrentPhaseC((ADL_t*)&ADL);
		adl_Frequency((ADL_t*)&ADL);
		adl_TotalActiveEnergy((ADL_t*)&ADL);
		adl_TotalActivePower((ADL_t*)&ADL);
	}
#endif

#if IMD_ENABLE

	if(1) // imdFlag
	{
		imd_enableMonitoring((IMD_t*)&IMD1);
		imd_enableMonitoring((IMD_t*)&IMD2);
	}
	else
	{
		imd_disableMonitoring((IMD_t*)&IMD1);
		imd_disableMonitoring((IMD_t*)&IMD2);
	}

	imd_ioStatus((IMD_t*)&IMD1);
	imd_busVoltage((IMD_t*)&IMD1);
	imd_positiveResistance((IMD_t*)&IMD1);
	imd_negativeResistance((IMD_t*)&IMD1);
	imd_ioStatus((IMD_t*)&IMD2);
	imd_busVoltage((IMD_t*)&IMD2);
	imd_positiveResistance((IMD_t*)&IMD2);
	imd_negativeResistance((IMD_t*)&IMD2);

#endif

#endif

}






