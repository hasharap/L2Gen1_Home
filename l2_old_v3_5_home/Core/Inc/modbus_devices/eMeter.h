/*
 * eMeter.h
 *
 *  Created on: Jul 9, 2023
 *  Revised on: Oct 2, 2023
 *      Author: Kaushan Ranasinghe
 */

#include <stdint.h>
#include <stdbool.h>

#ifndef INC_EMETER_H_
#define INC_EMETER_H_

/*##############################################################################
 *==============================================================================
 *  Title:  Energy Meter Library
 *
 *  Target: Developed for STM32 MCUs.
 *==============================================================================
 *############################################################################*/


/*==============================================================================
 *  Section:  ADL Energy Meter Stack
 *
 *  Description: Configure device parameters here.
 *============================================================================*/

/*----------------------------------------------------------------------------
 * Public Defines
 *----------------------------------------------------------------------------*/

#define ADL_SLAVE_ADDRESS 		0x01

#define ADL200_ENABLE			1

/*----------------------------------------------------------------------------
 * Private Defines
 *----------------------------------------------------------------------------*/

// Baud Rate Commands

#define ADL_BAUD_9600 													0x01
#define ADL_BAUD_4800 													0x02
#define ADL_BAUD_2400 													0x03
#define ADL_BAUD_1200 													0x04

// Read Data Registers

#define CURRENT_TOTAL_ACTIVE_ENERGY 									0x0000
#define CURRENT_SPIKE_TOTAL_ACTIVE_ENERGY 								0x0002
#define CURRENT_PEAK_TOTAL_ACTIVE_ENERGY 								0x0004
#define CURRENT_FLAT_TOTAL_ACTIVE_ENERGY 								0x0006
#define CURRENT_VALLEY_TOTAL_ACTIVE_ENERGY 								0x0008

#if ADL200_ENABLE

#define VOLTAGE_OF_A_PHASE 0x000B
#define CURRENT_OF_A_PHASE 0x000C

#define TOTAL_ACTIVE_POWER 0x000D
#define TOTAL_REACTIVE_POWER 0x000E
#define TOTAL_APPARENT_POWER 0x000F
#define TOTAL_POWER_FACTOR 0x0010

#define FREQUENCY 0x0011

#else

#define FREQUENCY 0x0077
#define VOLTAGE_OF_A_PHASE 0x0061
#define CURRENT_OF_A_PHASE 0x0064
#define TOTAL_ACTIVE_POWER 0x016A
#define TOTAL_REACTIVE_POWER 0x0172
#define TOTAL_APPARENT_POWER 0x017A
#define TOTAL_POWER_FACTOR 0x017F

#endif

#define CURRENT_FORWARD_ACTIVE_TOTAL_ENERGY 							0x000A
#define CURRENT_FORWARD_ACTIVE_SPIKE_ENERGY 							0x000C
#define CURRENT_FORWARD_ACTIVE_PEAK_ENERGY 								0x000E
#define CURRENT_FORWARD_ACTIVE_FLAT_ENERGY 								0x0010
#define CURRENT_FORWARD_ACTIVE_VALLEY_ENERGY 							0x0012

#define CURRENT_REVERSING_ACTIVE_TOTAL_ENERGY 							0x0014
#define CURRENT_REVERSING_ACTIVE_SPIKE_ENERGY 							0x0016
#define CURRENT_REVERSING_ACTIVE_PEAK_ENERGY 							0x0018
#define CURRENT_REVERSING_ACTIVE_FLAT_ENERGY 							0x001A
#define CURRENT_REVERSING_ACTIVE_VALLEY_ENERGY 							0x001C

#define CURRENT_TOTAL_REACTIVE_ENERGY 									0x001E
#define CURRENT_TOTAL_REACTIVE_SPIKE_ENERGY 							0x0020
#define CURRENT_TOTAL_REACTIVE_PEAK_ENERGY 								0x0022
#define CURRENT_TOTAL_REACTIVE_FLAT_ENERGY 								0x0024
#define CURRENT_TOTAL_REACTIVE_VALLEY_ENERGY 							0x0026

#define CURRENT_FORWARD_REACTIVE_TOTAL_ENERGY 							0x0028
#define CURRENT_FORWARD_REACTIVE_SPIKE_ENERGY 							0x002A
#define CURRENT_FORWARD_REACTIVE_PEAK_ENERGY 							0x002C
#define CURRENT_FORWARD_REACTIVE_FLAT_ENERGY 							0x002E
#define CURRENT_FORWARD_REACTIVE_VALLEY_ENERGY 							0x0030

#define CURRENT_REVERSING_REACTIVE_TOTAL_ENERGY 						0x0032
#define CURRENT_REVERSING_REACTIVE_SPIKE_ENERGY 						0x0034
#define CURRENT_REVERSING_REACTIVE_PEAK_ENERGY 							0x0036
#define CURRENT_REVERSING_REACTIVE_FLAT_ENERGY 							0x0038
#define CURRENT_REVERSING_REACTIVE_VALLEY_ENERGY 						0x003A

#define DATE_TIME 														0x003C

#define COMMUNICATION_PATH 												0x003F

#define PULSE_CONSTANT 													0x0040


//#define VOLTAGE_OF_A_PHASE 												0x0061
#define VOLTAGE_OF_B_PHASE 												0x0062
#define VOLTAGE_OF_C_PHASE 												0x0063
//#define CURRENT_OF_A_PHASE 												0x0064
#define CURRENT_OF_B_PHASE 												0x0065
#define CURRENT_OF_C_PHASE 												0x0066

//#define FREQUENCY 														0x0077

#define VOLTAGE_BETWEEN_A_B 											0x0078
#define VOLTAGE_BETWEEN_C_B 											0x0079
#define VOLTAGE_BETWEEN_A_C 											0x007A

#define FORWARD_ACTIVE_MAXIMUM_DEMAND 									0x007B
#define TIME_OF_OCCURRENCE_FOR_THE_FORWARD_ACTIVE_MAXIMUM_AMOUNT 		0x007C
#define REVERSING_ACTIVE_MAXIMUM_DEMAND 								0x007E
#define TIME_OF_OCCURRENCE_FOR_THE_REVERSING_ACTIVE_MAXIMUM_AMOUNT 		0x007F
#define MAXIMUM_FORWARD_DEMAND_FOR_REACTIVE_POWER 						0x0081
#define TIME_OF_OCCURRENCE_FOR_THE_FORWARD_REACTIVE_MAXIMUM_AMOUNT 		0x0082
#define MAXIMUM_REVERSING_DEMAND_FOR_REACTIVE_POWER 					0x0083
#define TIME_OF_OCCURRENCE_FOR_THE_REVERSING_REACTIVE_MAXIMUM_AMOUNT 	0x0085
#define FORWARD_ACTIVE_ENERGY_OF_A_PHASE 								0x0087
#define FORWARD_ACTIVE_ENERGY_OF_B_PHASE 								0x0089
#define FORWARD_ACTIVE_ENERGY_OF_C_PHASE 								0x008B
#define VOLTAGE_TRANSFER 												0x008D
#define CURRENT_TRANSFER 												0x008E
#define STATE_OF_DIDO_OVER_VOLTAGE 										0x008F

#define RUNNING_STATE_1 												0x0091
#define RUNNING_STATE_2 												0x0091
#define ZERO_SEQUENCE_CURRENT 											0x0092
#define VOLTAGE_IMBALANCE 												0x0093
#define CURRENT_IMBALANCE 												0x0094

#define ACTIVE_POWER_PHASE_A 											0x0164
#define ACTIVE_POWER_PHASE_B 											0x0166
#define ACTIVE_POWER_PHASE_C 											0x0168
//#define TOTAL_ACTIVE_POWER												0x016A
#define REACTIVE_POWER_PHASE_A 											0x016C
#define REACTIVE_POWER_PHASE_B 											0x016E
#define REACTIVE_POWER_PHASE_C 											0x0170
//#define TOTAL_REACTIVE_POWER 												0x0172
#define APPARENT_POWER_PHASE_A 											0x0174
#define APPARENT_POWER_PHASE_B 											0x0176
#define APPARENT_POWER_PHASE_C 											0x0178
//#define TOTAL_APPARENT_POWER 												0x017A
#define POWER_FACTOR_PHASE_A 											0x017C
#define POWER_FACTOR_PHASE_B 											0x017D
#define POWER_FACTOR_PHASE_C 											0x017E
//#define TOTAL_POWER_FACTOR 												0x017F
#define MAXIMUM_FORWARD_ACTIVE_DEMAND_A_DAY 							0x0180
#define MAXIMUM_FORWARD_ACTIVE_DEMAND_A_DAY_OCCUR_TIME 					0x0181
#define MAXIMUM_REVERSING_ACTIVE_DEMAND_A_DAY 							0x0182
#define MAXIMUM_REVERSING_ACTIVE_DEMAND_A_DAY_OCCUR_TIME 				0x0183
#define MAXIMUM_FORWARD_REACTIVE_DEMAND_A_DAY 							0x0184
#define MAXIMUM_FORWARD_REACTIVE_DEMAND_A_DAY_OCCUR_TIME 				0x0185
#define MAXIMUM_REVERSING_REACTIVE_DEMAND_A_DAY 						0x0186
#define MAXIMUM_REVERSING_REACTIVE_DEMAND_A_DAY_OCCUR_TIME 				0x0187
#define MAXIMUM_FORWARD_ACTIVE_DEMAND_LAST_DAY 							0x0188
#define MAXIMUM_FORWARD_ACTIVE_DEMAND_LAST_DAY_OCCUR_TIME 				0x0189
#define MAXIMUM_REVERSING_ACTIVE_DEMAND_LAST_DAY 						0x018A
#define MAXIMUM_REVERSING_ACTIVE_DEMAND_LAST_DAY_OCCUR_TIME 			0x018B
#define MAXIMUM_FORWARD_REACTIVE_DEMAND_LAST_DAY 						0x018C
#define MAXIMUM_FORWARD_REACTIVE_DEMAND_LAST_DAY_OCCUR_TIME 			0x018D
#define MAXIMUM_REVERSING_REACTIVE_DEMAND_LAST_DAY 						0x018E
#define MAXIMUM_REVERSING_REACTIVE_DEMAND_LAST_DAY_OCCUR_TIME 			0x018F

#define CURRENT_FORWARD_ACTIVE_DEMAND 									0x0198
#define CURRENT_REVERSE_ACTIVE_DEMAND 									0x0199
#define CURRENT_FORWARD_REACTIVE_DEMAND 								0x019A
#define CURRENT_REVERESE_REACTIVE_DEMAND 								0x019B

#define MAXIMUM_VOLTAGE_ON_PHASE_A 										0x0200
#define MAXIMUM_VOLTAGE_ON_PHASE_A_OCCUR_DATE 							0x0201
#define MAXIMUM_VOLTAGE_ON_PHASE_A_OCCUR_TIME 							0x0202
#define MAXIMUM_VOLTAGE_ON_PHASE_B_AND_OCCUR_TIME 						0x0203
#define MAXIMUM_VOLTAGE_ON_PHASE_C_AND_OCCUR_TIME 						0x0206
#define MAXIMUM_CURRENT_ON_PHASE_A_AND_OCCUR_TIME 						0x0209
#define MAXIMUM_CURRENT_ON_PHASE_B_AND_OCCUR_TIME 						0x020C
#define MAXIMUM_CURRENT_ON_PHASE_C_AND_OCCUR_TIME 						0x020F

#define MAXIMUM_ACTIVE_POWER_ON_PHASE_A 								0x0212
#define MAXIMUM_ACTIVE_POWER_ON_PHASE_A_OCCUR_DATE 						0x0214
#define MAXIMUM_ACTIVE_POWER_ON_PHASE_A_OCCUR_TIME 						0x0215
#define MAXIMUM_ACTIVE_POWER_ON_PHASE_B_AND_OCCUR_TIME 					0x0216
#define MAXIMUM_ACTIVE_POWER_ON_PHASE_C_AND_OCCUR_TIME 					0x021A
#define MAXIMUM_ACTIVE_POWER_AND_OCCUR_TIME 							0x021E

#define MAXIMUM_REACTIVE_POWER_ON_PHASE_A_AND_OCCUR_TIME 				0x0222
#define MAXIMUM_REACTIVE_POWER_ON_PHASE_B_AND_OCCUR_TIME 				0x0226
#define MAXIMUM_REACTIVE_POWER_ON_PHASE_C_AND_OCCUR_TIME 				0x022A
#define MAXIMUM_REACTIVE_POWER_AND_OCCUR_TIME 							0x022E

#define MAXIMUM_APPARENT_POWER_ON_PHASE_A_AND_OCCUR_TIME 				0x0232
#define MAXIMUM_APPARENT_POWER_ON_PHASE_B_AND_OCCUR_TIME 				0x0236
#define MAXIMUM_APPARENT_POWER_ON_PHASE_C_AND_OCCUR_TIME 				0x023A
#define MAXIMUM_APPARENT_POWER_AND_OCCUR_TIME 							0x023E

#define MINIMUM_VOLTAGE_ON_PHASE_A_AND_OCCUR_TIME 						0x0242
#define MINIMUM_VOLTAGE_ON_PHASE_B_AND_OCCUR_TIME 						0x0245
#define MINIMUM_VOLTAGE_ON_PHASE_C_AND_OCCUR_TIME 						0x0248
#define MINIMUM_CURRENT_ON_PHASE_A_AND_OCCUR_TIME 						0x024B
#define MINIMUM_CURRENT_ON_PHASE_B_AND_OCCUR_TIME 						0x024E
#define MINIMUM_CURRENT_ON_PHASE_C_AND_OCCUR_TIME 						0x0251

#define MINIMUM_ACTIVE_POWER_ON_PHASE_A_AND_OCCUR_TIME 					0x0254
#define MINIMUM_ACTIVE_POWER_ON_PHASE_B_AND_OCCUR_TIME 					0x0258
#define MINIMUM_ACTIVE_POWER_ON_PHASE_C_AND_OCCUR_TIME 					0x025C
#define MINIMUM_ACTIVE_POWER_AND_OCCUR_TIME 							0x0260

#define MINIMUM_REACTIVE_POWER_ON_PHASE_A_AND_OCCUR_TIME 				0x0264
#define MINIMUM_REACTIVE_POWER_ON_PHASE_B_AND_OCCUR_TIME 				0x0268
#define MINIMUM_REACTIVE_POWER_ON_PHASE_C_AND_OCCUR_TIME 				0x026C
#define MINIMUM_REACTIVE_POWER_AND_OCCUR_TIME 							0x0270

#define MINIMUM_APPARENT_POWER_ON_PHASE_A_AND_OCCUR_TIME 				0x0274
#define MINIMUM_APPARENT_POWER_ON_PHASE_B_AND_OCCUR_TIME 				0x0278
#define MINIMUM_APPARENT_POWER_ON_PHASE_C_AND_OCCUR_TIME 				0x027E
#define MINIMUM_APPARENT_POWER_AND_OCCUR_TIME 							0x0280

#define T1_TEMPERATURE 													0x2000
#define T2_TEMPERATURE 													0x2001
#define T3_TEMPERATURE 													0x2002

#define VOLTAGE_TRANSFER 												0x008D
#define CURRENT_TRANSFER 												0x008E
#define THRESHOLD_AND_STATE_OF_LOSS_VOLTAGE 							0x008F
#define RUNNING_STATE 													0x0091


/*----------------------------------------------------------------------------
 * Value Tables
 *----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------
 * Structures
 *----------------------------------------------------------------------------*/

typedef struct
{
	uint8_t slaveAddress;

	struct
	{
		union
		{
			uint16_t all;
			struct
			{
				uint8_t baud_rate;
				uint8_t slave_address;
			}bytes;
		}communicationParameters;

		union
		{
			uint16_t all[3];
			struct
			{
				uint8_t ss;
				uint8_t hh;
				uint8_t dd;
				uint8_t mm;
				uint8_t yy;
				uint8_t MM;
			}bytes;
		}datetime;
	}setData;

	struct
	{
		uint16_t voltage_PhaseA;
		uint16_t voltage_PhaseB;
		uint16_t voltage_PhaseC;
		uint16_t current_PhaseA;
		uint16_t current_PhaseB;
		uint16_t current_PhaseC;
		uint16_t freqeuncy;
		uint32_t activePower;
		uint32_t activeEnergy;
		uint32_t max_apparentPower;

		struct
		{
			uint8_t second;
			uint8_t minute;
			uint8_t hour;
			uint8_t day;
			uint8_t month;
			uint8_t year;
		}datetime;

	}data;

}ADL_t;

/*----------------------------------------------------------------------------
 * Function Prototypes
 *----------------------------------------------------------------------------*/

void adl_TotalActiveEnergy(const ADL_t* restrict adl);
void adl_SpikeTotalActiveEnergy(const ADL_t* restrict adl);
void adl_PeakTotalActiveEnergy(const ADL_t* restrict adl);
void adl_FlatTotalActiveEnergy(const ADL_t* restrict adl);
void adl_ValleyTotalActiveEnergy(const ADL_t* restrict adl);
void adl_ForwardActiveTotalEnergy(const ADL_t* restrict adl);
void adl_ForwardActiveSpikeEnergy(const ADL_t* restrict adl);
void adl_ForwardActivePeakEnergy(const ADL_t* restrict adl);
void adl_ForwardActiveFlatEnergy(const ADL_t* restrict adl);
void adl_ForwardActiveValleyEnergy(const ADL_t* restrict adl);
void adl_ReversingActiveTotalEnergy(const ADL_t* restrict adl);
void adl_ReversingActiveSpikeEnergy(const ADL_t* restrict adl);
void adl_ReversingActivePeakEnergy(const ADL_t* restrict adl);
void adl_ReversingActiveFlatEnergy(const ADL_t* restrict adl);
void adl_ReversingActiveValleyEnergy(const ADL_t* restrict adl);
void adl_TotalReactiveEnergy(const ADL_t* restrict adl);
void adl_SpikeTotalReactiveEnergy(const ADL_t* restrict adl);
void adl_PeakTotalReactiveEnergy(const ADL_t* restrict adl);
void adl_FlatTotalReactiveEnergy(const ADL_t* restrict adl);
void adl_ValleyTotalReactiveEnergy(const ADL_t* restrict adl);
void adl_ForwardTotalReactiveEnergy(const ADL_t* restrict adl);
void adl_ForwardSpikeTotalReactiveEnergy(const ADL_t* restrict adl);
void adl_ForwardPeakTotalReactiveEnergy(const ADL_t* restrict adl);
void adl_ForwardFlatTotalReactiveEnergy(const ADL_t* restrict adl);
void adl_ForwardValleyTotalReactiveEnergy(const ADL_t* restrict adl);
void adl_ReversingTotalReactiveEnergy(const ADL_t* restrict adl);
void adl_ReversingSpikeTotalReactiveEnergy(const ADL_t* restrict adl);
void adl_ReversingPeakTotalReactiveEnergy(const ADL_t* restrict adl);
void adl_ReversingFlatTotalReactiveEnergy(const ADL_t* restrict adl);
void adl_ReversingValleyTotalReactiveEnergy(const ADL_t* restrict adl);
void adl_GetDateTime(const ADL_t* restrict adl);
void adl_SetDateTime(const ADL_t* restrict adl);
void adl_GetCommParams(const ADL_t* restrict adl);
void adl_SetCommParams(const ADL_t* restrict adl);
void adl_PulseConstant(const ADL_t* restrict adl);
void adl_VoltagePhaseA(const ADL_t* restrict adl);
void adl_VoltagePhaseB(const ADL_t* restrict adl);
void adl_VoltagePhaseC(const ADL_t* restrict adl);
void adl_CurrentPhaseA(const ADL_t* restrict adl);
void adl_CurrentPhaseB(const ADL_t* restrict adl);
void adl_CurrentPhaseC(const ADL_t* restrict adl);
void adl_Frequency(const ADL_t* restrict adl);
void adl_VoltagePhaseAtoB(const ADL_t* restrict adl);
void adl_VoltagePhaseCtoB(const ADL_t* restrict adl);
void adl_VoltagePhaseAtoC(const ADL_t* restrict adl);
void adl_TotalActivePower(const ADL_t* restrict adl);
void adl_MaximumApparentPowerAndOccurTime(const ADL_t* restrict adl);

/*----------------------------------------------------------------------------
 * Public Instantiations
 *----------------------------------------------------------------------------*/

extern volatile ADL_t ADL;
extern volatile bool emeter_bootup;

/*----------------------------------------------------------------------------
 * Public Prototypes
 *----------------------------------------------------------------------------*/

void adl_getData(uint8_t* slaveAddress, uint16_t* startAddress, uint16_t* result16, uint32_t* result32);

#endif /* INC_EMETER_H_ */
