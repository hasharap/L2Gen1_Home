/*
 * CSMS.c
 *
 *  Created on: Sep 20, 2023
 *      Author: Kaushan Ranasinghe
 */

#include "CSMS.h"

/*----------------------------------------------------------------------------
 * Public Instantiations
 *----------------------------------------------------------------------------*/

volatile CSMS_t CSMS = {0}; 					// charging station management system parameters
volatile CSMS_MessageQueue CSMS_Queue = {0};

/*----------------------------------------------------------------------------
 * Private Instantiations
 *----------------------------------------------------------------------------*/

static char RxBuffer[100] = {0};
static uint8_t RxBufferPos = 0;
static bool RxPcktCpltFlag = false;

static volatile uint32_t pingPongCount = 0;
static volatile bool rxFlag = 0;
static volatile RxSerialState_t RxSerialState = 2;

static volatile uint32_t RxBusTimeout = 0;
static volatile uint32_t RxSerialFaultTimeout = 0;

uint16_t check1 = 0;

/*----------------------------------------------------------------------------
 * Private Prototypes
 *----------------------------------------------------------------------------*/

static bool CSMS_checksum(const char* restrict data, uint8_t length);

/*----------------------------------------------------------------------------
 * Timer Function
 *----------------------------------------------------------------------------*/

void CSMS_Intick(void) // period in milliseconds
{
	switch(CSMS_TICK_TIME)
	{
		case 1:

		case 10:
            RxBusTimeout++;

		case 30:

		case 100:
            RxSerialFaultTimeout++;

		case 250:

		case 500:

		case 1000:

		default:
			break;
	}
}


/*----------------------------------------------------------------------------
 * Send / Receive Functions
 *----------------------------------------------------------------------------*/

static void CSMS_queueSerial(const char* restrict data, const uint8_t* length, uint8_t id)
{
	static uint8_t index = 0;

	for(uint8_t i = 0; i < CSMS_QUEUE_SIZE; i++)
	{
		if((CSMS_Queue.ID[i] == id) && (((CSMS_Queue.ActiveMask >> i) & 0x0001) == 1))
		{
			return; // change this to current index i if found to update earliest command with new information
		}

		if(CSMS_Queue.ActiveMask == 0)
		{
			index = 0;
		}
	}

	CSMS_Queue.ActiveMask |= (1 << index);
	CSMS_Queue.ID[index] = id;

	for(uint8_t i = 0; i < *length; i++) // fill data
	{
		CSMS_Queue.TxData[index][i] = data[i];

		if((data[i-1] == '*') && (data[i] == '\n'))
		{
			CSMS_Queue.Length[index] = (i + 1);
			break;
		}
	}

	index++;

	if((index >= CSMS_QUEUE_SIZE))
	{
		index = 0;
	}
}


void CSMS_transmitSerial(void (*TransmitFunction)(uint8_t*, uint16_t))
{
	if(1)
	{
//		TxIntervalFlag = false;
		static uint8_t instruction = 0;

		if ((CSMS_Queue.ActiveMask & (1 << instruction)) != 0x0)        // check whether activation mask not zero
		{
			TransmitFunction((uint8_t*)CSMS_Queue.TxData[instruction], CSMS_Queue.Length[instruction]);
			CSMS_Queue.ActiveMask &= ~(1 << instruction);               // reset activation for instruction
			instruction++;                                              // update instruction starting point

			if((instruction == CSMS_QUEUE_SIZE) || (CSMS_Queue.ActiveMask == 0))
			{
				instruction = 0;
			}
		}
	}
}


void CSMS_receiveSerial(const char* restrict data)
{
	static uint8_t RxCount = 0;
	static bool RxBufferFlush = false;

    if(RxBusTimeout >= CSMS_RX_DEAD_TIME_INTERVAL) // dead time monitor
    {
        if(RxSerialState != RxIdle)
        {
            memset(RxBuffer,0,(sizeof(RxBuffer)/sizeof(RxBuffer[0])));
            RxBufferPos = 0;
            RxSerialState = RxSoF;
            RxBufferFlush = false;
            RxCount = 0;
        }
    }

    RxBusTimeout = 0;

	switch(RxSerialState)
	{
		case RxError:
			break;

		case RxIdle:
			RxCount = 0;
			break;

		case RxSoF: 									// Start of Frame
			if((*data == '#') && (RxCount == 0))
			{
				RxBuffer[RxBufferPos++] = *data;
			}
			else if((*data == 'M') && (RxCount == 1))
			{
				RxBuffer[RxBufferPos++] = *data;
				RxSerialState = RxData;
			}
			else
			{
				RxBufferFlush = true;
			}
			break;

		case RxData:
			RxBuffer[RxBufferPos++] = *data;

			if(RxCount == 17)
			{
				RxSerialState = RxEoF;
			}
			break;

		case RxEoF:										// End of Frame
			if((*data == '*') && (RxCount == 18))
			{
				RxBuffer[RxBufferPos++] = *data;
			}
			else if((*data == '\n') && (RxCount == 19))
			{
				RxBuffer[RxBufferPos++] = *data;
				RxPcktCpltFlag = true;
				RxSerialState = RxIdle;
			}
			else
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
		memset(RxBuffer,0,(sizeof(RxBuffer)/sizeof(RxBuffer[0])));
		RxBufferPos = 0;
		RxSerialState = RxSoF;
		RxBufferFlush = false;
		RxCount = 0;
	}
}

/*----------------------------------------------------------------------------
 * Functions
 *----------------------------------------------------------------------------*/

static uint16_t CSMS_modbusCRC(const char* restrict data, uint8_t length)
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

static bool CSMS_checksum(const char* restrict data, uint8_t length)
{
	uint16_t crc_calc = CSMS_modbusCRC(data, (length - 2));
	uint16_t crc_rx = (data[length - 1] << 8) | (data[length - 2]);
	if ((crc_calc == crc_rx) && (crc_calc != 0) && (crc_rx != 0))
	{
		return true;
	}
	else if ((crc_calc != crc_rx) && (crc_calc != 0) && (crc_rx != 0))
	{
		return false;
	}
	else
	{
		return false;
	}
}

static void CSMS_setmSerial(void)
{
	uint8_t length = 30;
	char data[30] =
	{ 0 };

	data[0] = '#';
	data[1] = 'C';
	data[2] = controlSide_data.status.all;
	data[3] = powerSide_data.status.all;
	data[4] = controlSide_data.networkSide_request.all;
	data[5] = (controlSide_data.errorStatus.all & 0xFF);
	data[6] = controlSide_data.errorReport;
	data[7] = powerSide_data.tripStatus.all;
	data[8] = powerSide_data.errorStatus.all;
	data[9] = CSMS.request.msgID;

	uint16_t temp;
	uint32_t temp2;

#if THREE_PHASE
	switch (CSMS.request.msgID)
	{
	case 0:
		temp = 0;
		temp = powerSide_data.voltage.VA;
		data[10] = (temp & 0xFF); 			// LB
		data[11] = ((temp >> 8) & 0xFF);	// HB
		temp = 0;
		temp = powerSide_data.voltage.VB;
		data[12] = (temp & 0xFF); 			// LB
		data[13] = ((temp >> 8) & 0xFF);	// HB
		temp = 0;
		temp = powerSide_data.voltage.VC;
		data[14] = (temp & 0xFF); 			// LB
		data[15] = ((temp >> 8) & 0xFF);	// HB
		break;

	case 1:
		temp = 0;
		temp = powerSide_data.current.IA;
		data[10] = (temp & 0xFF); 			// LB
		data[11] = ((temp >> 8) & 0xFF);	// HB
		temp = 0;
		temp = powerSide_data.current.IB;
		data[12] = (temp & 0xFF); 			// LB
		data[13] = ((temp >> 8) & 0xFF);	// HB
		temp = 0;
		temp = powerSide_data.current.IC;
		data[14] = (temp & 0xFF); 			// LB
		data[15] = ((temp >> 8) & 0xFF);	// HB
		break;

	case 2:
		data[10] = powerSide_data.powerEnergy.power;
		temp2 = 0;
		temp2 = (uint32_t) powerSide_data.powerEnergy.kWh;
		data[11] = (uint8_t) (temp2 & 0xFF);
		data[12] = (uint8_t) (temp2 >> 8); 		// LB
		data[13] = (uint8_t) ((temp2 >> 16));	// HB
		data[14] = (int8_t) powerSide_data.tempSensors.T1;
		data[15] = (uint8_t) powerSide_data.frequency;
		break;
	/*---------------------------------------------------
	 Actual energy meter reading - message ID = 4 (Dec)
	 ---------------------------------------------------*/
	case 3:
		temp = 0;
		temp = (uint16_t) (controlSide_data.controlPilot.cp_max);
		data[10] = (temp & 0xFF); 			// LB
		data[11] = ((temp >> 8) & 0xFF);	// HB
		data[12] = (ADL.data.activeEnergy & 0xFF);
		data[13] = ((ADL.data.activeEnergy >> 8) & 0xFF);
		data[14] = ((ADL.data.activeEnergy >> 16) & 0xFF);
		data[15] = ((ADL.data.activeEnergy >> 24) & 0xFF);
		break;
	/*---------------------------------------------------
	 SW version - message ID = 3 (Dec)
	 ---------------------------------------------------*/
	case 4:
		data[10] = 3;
		data[11] = 0;
		data[12] = 1;
		data[13] = 0xFF;
		data[14] = 0xFF;
		data[15] = 0xFF;
		break;
	/*---------------------------------------------------
	 HW version - message ID = 5 (Dec)
	 ---------------------------------------------------*/
	case 5:
		data[10] = hw_version.v1;
		data[11] = hw_version.v2;
		data[12] = hw_version.v3;
		data[13] = 0xFF;
		data[14] = 0xFF;
		data[15] = 0xFF;
		break;
	/*---------------------------------------------------
	 OTA reply - message ID = 100 (Dec)
	 ---------------------------------------------------*/
	/*case 100:
		data[10] = 11;
		data[11] = 22;
		data[12] = 33;
		data[13] = 0xFF;
		data[14] = 0xFF;
		data[15] = 0xFF;
		break;
	*/
	/*---------------------------------------------------
	 Config command 1 reply - message ID = 46 (Dec)
	 ---------------------------------------------------*/
	case 45:
		data[10] = 0xFF;
		data[11] = 0xFF;
		data[12] = 0xFF;
		data[13] = 0xFF;
		data[14] = 0xFF;
		data[15] = 0xFF;
		break;

	case 46:
		if (charger_configSet.config_counter >= 10)
		{
			data[10] = (charger_configSet.en_1.all & 0xFF);
			data[11] = ((charger_configSet.en_1.all >> 8) & 0xFF);
			data[12] = (charger_configSet.en_2.all & 0xFF);
			data[13] = ((charger_configSet.en_2.all >> 8) & 0xFF);
			data[14] = (charger_configSet.uv_upper & 0xFF);
			data[15] = ((charger_configSet.uv_upper >> 8) & 0xFF);
		}
		else
		{
			data[10] = (charger_configGet.en_1.all & 0xFF);
			data[11] = ((charger_configGet.en_1.all >> 8) & 0xFF);
			data[12] = (charger_configGet.en_2.all & 0xFF);
			data[13] = ((charger_configGet.en_2.all >> 8) & 0xFF);
			data[14] = (charger_configGet.uv_upper & 0xFF);
			data[15] = ((charger_configGet.uv_upper >> 8) & 0xFF);
		}
		break;
	/*---------------------------------------------------
	 Config command 2 reply - message ID = 47 (Dec)
	 ---------------------------------------------------*/
	case 47:
		if (charger_configSet.config_counter >= 10)
		{
			data[10] = (charger_configSet.uv_lower & 0xFF);
			data[11] = ((charger_configSet.uv_lower >> 8) & 0xFF);
			data[12] = (charger_configSet.ov_upper & 0xFF);
			data[13] = ((charger_configSet.ov_upper >> 8) & 0xFF);
			data[14] = (charger_configSet.ov_lower & 0xFF);
			data[15] = ((charger_configSet.ov_lower >> 8) & 0xFF);
		}
		else
		{
			data[10] = (charger_configGet.uv_lower & 0xFF);
			data[11] = ((charger_configGet.uv_lower >> 8) & 0xFF);
			data[12] = (charger_configGet.ov_upper & 0xFF);
			data[13] = ((charger_configGet.ov_upper >> 8) & 0xFF);
			data[14] = (charger_configGet.ov_lower & 0xFF);
			data[15] = ((charger_configGet.ov_lower >> 8) & 0xFF);
		}
		break;
	/*---------------------------------------------------
	 Config command 3 reply - message ID = 48 (Dec)
	 ---------------------------------------------------*/
	case 48:
		if (charger_configSet.config_counter >= 10)
		{
			data[10] = (charger_configSet.freq_upper & 0xFF);
			data[11] = ((charger_configSet.freq_upper >> 8) & 0xFF);
			data[12] = (charger_configSet.freq_lower & 0xFF);
			data[13] = ((charger_configSet.freq_lower >> 8) & 0xFF);
			data[14] = charger_configSet.max_current;
			data[15] = 0xFF;
		}
		else
		{
			data[10] = (charger_configGet.freq_upper & 0xFF);
			data[11] = ((charger_configGet.freq_upper >> 8) & 0xFF);
			data[12] = (charger_configGet.freq_lower & 0xFF);
			data[13] = ((charger_configGet.freq_lower >> 8) & 0xFF);
			data[14] = charger_configGet.max_current;
			data[15] = 0xFF;
		}
		break;

	default:
		break;
	}
#else
	switch (CSMS.request.msgID)
	{
	case 0:
		temp = 0;
		temp = (uint16_t) (powerSide_data.voltage.VA);
		data[10] = (temp & 0xFF); 			// LB
		data[11] = ((temp >> 8) & 0xFF);	// HB
		data[12] = 0; 			// LB
		data[13] = 0;	// HB
		data[14] = 0; 	// LB
		data[15] = 0;	// HB
		break;

	case 1:
		temp = 0;
		temp = (uint16_t) (powerSide_data.current.IA);
		data[10] = (temp & 0xFF); 			// LB
		data[11] = ((temp >> 8) & 0xFF);	// HB
		data[12] = 0; 	// LB
		data[13] = 0;	// HB
		data[14] = 0; 	// LB
		data[15] = 0;	// HB
		break;

	case 2:
		data[10] = powerSide_data.powerEnergy.power;
		temp2 = 0;
		temp2 = (uint32_t) powerSide_data.powerEnergy.kWh;
		data[11] = (uint8_t) (temp2 & 0xFF);
		data[12] = (uint8_t) (temp2 >> 8); 		// LB
		data[13] = (uint8_t) ((temp2 >> 16));	// HB
		data[14] = (int8_t) powerSide_data.tempSensors.T1;
		data[15] = (uint8_t) powerSide_data.frequency;
		break;

	/*---------------------------------------------------
	 Actual energy meter reading - message ID = 3 (Dec)
	 ---------------------------------------------------*/
	case 3:
		temp = 0;
		temp = (uint16_t) (controlSide_data.controlPilot.cp_max);
		data[10] = (temp & 0xFF); 			// LB
		data[11] = ((temp >> 8) & 0xFF);	// HB
		data[12] = (ADL.data.activeEnergy & 0xFF);
		data[13] = ((ADL.data.activeEnergy >> 8) & 0xFF);
		data[14] = ((ADL.data.activeEnergy >> 16) & 0xFF);
		data[15] = ((ADL.data.activeEnergy >> 24) & 0xFF);
		break;
	/*---------------------------------------------------
	 SW version - message ID = 4 (Dec)
	 ---------------------------------------------------*/
	case 4:
		data[10] = 3;
		data[11] = 0;
		data[12] = 2;//1;
		data[13] = 0xFF;
		data[14] = 0xFF;
		data[15] = 0xFF;
		break;
	/*---------------------------------------------------
	 HW version - message ID = 5 (Dec)
	 ---------------------------------------------------*/
	case 5:
		data[10] = hw_version.v1;
		data[11] = hw_version.v2;
		data[12] = hw_version.v3;
		data[13] = 0xFF;
		data[14] = 0xFF;
		data[15] = 0xFF;
		break;
	/*---------------------------------------------------
	 OTA reply - message ID = 100 (Dec)
	 ---------------------------------------------------*/
	/*case 100:
		data[10] = 11;
		data[11] = 22;
		data[12] = 33;
		data[13] = 0xFF;
		data[14] = 0xFF;
		data[15] = 0xFF;
		break;
	*/
	/*---------------------------------------------------
	 Config command 0 reply - message ID = 45 (Dec)
	 ---------------------------------------------------*/
	case 45:
		data[10] = 0xFF;
		data[11] = 0xFF;
		data[12] = 0xFF;
		data[13] = 0xFF;
		data[14] = 0xFF;
		data[15] = 0xFF;
		break;
	/*---------------------------------------------------
	 Config command 1 reply - message ID = 46 (Dec)
	 ---------------------------------------------------*/
	case 46:
		if (charger_configSet.config_counter >= 10)
		{
			data[10] = (charger_configSet.en_1.all & 0xFF);
			data[11] = ((charger_configSet.en_1.all >> 8) & 0xFF);
			data[12] = (charger_configSet.en_2.all & 0xFF);
			data[13] = ((charger_configSet.en_2.all >> 8) & 0xFF);
			data[14] = (charger_configSet.uv_upper & 0xFF);
			data[15] = ((charger_configSet.uv_upper >> 8) & 0xFF);
		}
		else
		{
			data[10] = (charger_configGet.en_1.all & 0xFF);
			data[11] = ((charger_configGet.en_1.all >> 8) & 0xFF);
			data[12] = (charger_configGet.en_2.all & 0xFF);
			data[13] = ((charger_configGet.en_2.all >> 8) & 0xFF);
			data[14] = (charger_configGet.uv_upper & 0xFF);
			data[15] = ((charger_configGet.uv_upper >> 8) & 0xFF);
		}
		break;
	/*---------------------------------------------------
	 Config command 2 reply - message ID = 47 (Dec)
	 ---------------------------------------------------*/
	case 47:
		if (charger_configSet.config_counter >= 10)
		{
			data[10] = (charger_configSet.uv_lower & 0xFF);
			data[11] = ((charger_configSet.uv_lower >> 8) & 0xFF);
			data[12] = (charger_configSet.ov_upper & 0xFF);
			data[13] = ((charger_configSet.ov_upper >> 8) & 0xFF);
			data[14] = (charger_configSet.ov_lower & 0xFF);
			data[15] = ((charger_configSet.ov_lower >> 8) & 0xFF);
		}
		else
		{
			data[10] = (charger_configGet.uv_lower & 0xFF);
			data[11] = ((charger_configGet.uv_lower >> 8) & 0xFF);
			data[12] = (charger_configGet.ov_upper & 0xFF);
			data[13] = ((charger_configGet.ov_upper >> 8) & 0xFF);
			data[14] = (charger_configGet.ov_lower & 0xFF);
			data[15] = ((charger_configGet.ov_lower >> 8) & 0xFF);
		}
		break;
	/*---------------------------------------------------
	 Config command 3 reply - message ID = 48 (Dec)
	 ---------------------------------------------------*/
	case 48:
		if (charger_configSet.config_counter >= 10)
		{
			data[10] = (charger_configSet.freq_upper & 0xFF);
			data[11] = ((charger_configSet.freq_upper >> 8) & 0xFF);
			data[12] = (charger_configSet.freq_lower & 0xFF);
			data[13] = ((charger_configSet.freq_lower >> 8) & 0xFF);
			data[14] = charger_configSet.max_current;
			data[15] = 0xFF;
		}
		else
		{
			data[10] = (charger_configGet.freq_upper & 0xFF);
			data[11] = ((charger_configGet.freq_upper >> 8) & 0xFF);
			data[12] = (charger_configGet.freq_lower & 0xFF);
			data[13] = ((charger_configGet.freq_lower >> 8) & 0xFF);
			data[14] = charger_configGet.max_current;
			data[15] = 0xFF;
		}
		break;

	default:
		break;
	}
#endif

	if (CSMS.request.msgID != 100)
	{
		uint16_t crc = CSMS_modbusCRC(&data[1], (16 - 1));
		data[16] = (crc & 0xFF);
		data[17] = ((crc >> 8) & 0xFF);
		data[18] = '*';
		data[19] = '\n';

		CSMS_queueSerial(data, &length, 1);
		pingPongCount++;
	}
}

static void CSMS_getmSerial(const char* restrict data, uint8_t length)
{
	if (CSMS_checksum(data, length))
	{
		networkSide_data.status = (uint8_t) data[1];
		networkSide_data.stopCharge = (uint8_t) data[2];
		networkSide_data.scheduleCharge = (uint8_t) data[3];
		networkSide_data.isInternet_available = (uint8_t) data[4];
		networkSide_data.ledOnOff_command = (uint8_t) data[5];
		networkSide_data.chargerLock = (uint8_t) data[6];
		networkSide_data.scheduelCharge_active = (uint8_t) data[7];
		CSMS.request.msgID = (uint8_t) data[8];

		uint8_t data1 = (uint8_t) data[9];
		uint8_t data2 = (uint8_t) data[10];
		uint8_t data3 = (uint8_t) data[11];
		uint8_t data4 = (uint8_t) data[12];
		uint8_t data5 = (uint8_t) data[13];
		uint8_t data6 = (uint8_t) data[14];

		switch (CSMS.request.msgID)
		{
		case 0:
			networkSide_data.maxCurrent_req = data1;
			networkSide_data.timeReady = data2;
			networkSide_data.alarmUpdate = data3;
			networkSide_data.setTime.Hours = data4;
			networkSide_data.setTime.Minutes = data5;
			networkSide_data.loadBalancing_en = data6;
			break;
		case 1:
			networkSide_data.weekdayOn.Hours = data1;
			networkSide_data.weekdayOn.Minutes = data2;
			networkSide_data.weekdayOff.Hours = data3;
			networkSide_data.weekdayOff.Minutes = data4;
			networkSide_data.setDate.Month = data5;
			networkSide_data.setDate.Date = data6;
			break;
		case 2:
			networkSide_data.weekendOn.Hours = data1;
			networkSide_data.weekendOn.Minutes = data2;
			networkSide_data.weekendOff.Hours = data3;
			networkSide_data.weekendOff.Minutes = data4;
			networkSide_data.setDate.Year = data6;
			break;

		/*---------------------------------------------------
		 Check for OTA update flag - check message ID = 100 (Dec)
		 ---------------------------------------------------*/
		case 100:
			OTA_flag = true;
			break;
		/*---------------------------------------------------
		 System reset command - check message ID = 40 (Dec)
		 ---------------------------------------------------*/
		case 40:
			if ((data1 == 11) && (data2 == 22) && (data3 == 33))
			{
				HAL_NVIC_SystemReset();
			}
			break;
		/*---------------------------------------------------
		 Config command 1 - check message ID = 46 (Dec)
		 ---------------------------------------------------*/
		case 45:
			if (data1 == 45)
			{
				charger_configSet.config_counter = 10;
			}
			break;
		/*---------------------------------------------------
		 Config command 1 - check message ID = 46 (Dec)
		 ---------------------------------------------------*/
		case 46:
			if (charger_configSet.config_counter == 10)
			{
				charger_configSet.en_1.all = ((data2 << 8) | data1);
				charger_configSet.en_2.all = ((data4 << 8) | data3);
				charger_configSet.uv_upper = ((data6 << 8) | data5);
				charger_configSet.config_counter = 20;
			}
			break;
		/*---------------------------------------------------
		 Config command 2 - check message ID = 47 (Dec)
		 ---------------------------------------------------*/
		case 47:
			if (charger_configSet.config_counter == 20)
			{
				charger_configSet.uv_lower = ((data2 << 8) | data1);
				charger_configSet.ov_upper = ((data4 << 8) | data3);
				charger_configSet.ov_lower = ((data6 << 8) | data5);
				charger_configSet.config_counter = 30;
			}
			break;
		/*---------------------------------------------------
		 Config command 3 - check message ID = 48 (Dec)
		 ---------------------------------------------------*/
		case 48:
			if (charger_configSet.config_counter == 30)
			{
				charger_configSet.freq_upper = ((data2 << 8) | data1);
				charger_configSet.freq_lower = ((data4 << 8) | data3);
				charger_configSet.max_current = data5;
				charger_configSet.config_counter = 89;
			}

			if (charger_configSet.config_counter == 89)
			{
				charger_configSet.config_enable = true;
			}
			else
			{
				charger_configSet.config_enable = false;
				charger_configSet.config_counter = 0;
			}
			break;

		default:
			break;
		}

		rxFlag = true;
		memset(RxBuffer, 0, (sizeof(RxBuffer) / sizeof(RxBuffer[0])));
		RxBufferPos = 0;
		RxSerialState = RxSoF;
		RxSerialFaultTimeout = 0;       // reset packet fault timeout
	}
	else
	{
		memset(RxBuffer, 0, (sizeof(RxBuffer) / sizeof(RxBuffer[0])));
		RxBufferPos = 0;
		RxSerialState = RxSoF;
	}
}

void CSMS_Init(void)
{
//	RxSerialState = RxSoF;
}

void CSMS_functions(void)
{
	if(RxPcktCpltFlag)
	{
		CSMS_getmSerial(&RxBuffer[1], (RxBufferPos - 3));
		RxPcktCpltFlag = false;
	}

	if(rxFlag)
	{
		CSMS_setmSerial();
		rxFlag = false;
		networkSide_bootup = true;
	}
}
