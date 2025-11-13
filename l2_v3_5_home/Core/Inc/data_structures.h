/*
 * data_structures.h
 *
 *  Created on: Jul 28, 2022
 *      Author: Nisal Bulathsinghala
 */

#ifndef INC_DATA_STRUCTURES_H_
#define INC_DATA_STRUCTURES_H_

#include "main.h"

//################################################################################
//--------------------------------------------------------------------------------
//                  Control side data structures
//--------------------------------------------------------------------------------

union warning_union
{
	uint8_t all;
	struct
	{
		uint8_t UV_warn :1;
		uint8_t UC_warn :1;
		uint8_t resrvd_1 :1;
		uint8_t resrvd_2 :1;
		uint8_t resrvd_3 :1;
		uint8_t resrvd_4 :1;
		uint8_t resrvd_5 :1;
		uint8_t resrvd_6 :1;
	} bits;
};

struct output_bits
{

};

union output_union
{
	uint16_t all;
	struct output_bits bit;
};

struct input_bits
{
	uint16_t SW1 :1;
	uint16_t SW2 :1;
	uint16_t SW3 :1;
	uint16_t SW4 :1;
};

union input_union
{
	uint16_t all;
	struct input_bits bit;
};

struct tempSensors_struct
{
	float T1;
	float T2;
	float T3;
	float T4;
};

struct controlSide_status_struct
{
	uint8_t curretState :5;
	uint8_t charging_active :1;
	uint8_t cpPWM_active :1;
	uint8_t connector_state :1;
};

union controlSide_status_union
{
	uint8_t all;
	struct controlSide_status_struct bit;
};

struct controlSide_controlPilot_struct
{
	volatile uint16_t PWMSET;
	uint8_t cp_duty_enable;
	uint8_t cp_enable;
	uint16_t cpDuty;
	uint16_t cp_min;
	uint16_t cp_max;
	int cp_Vmin;
	int cp_Vmax;
};

struct controlSide_errorStatus_struct
{
	uint8_t serialAError :1;
	uint8_t diodeCheck_failed :1;
	uint8_t fault_level :2;
	uint8_t CPFault :1;
};

union controlSide_errorStatus_union
{
	uint8_t all;
	struct controlSide_errorStatus_struct bit;
};

struct powerSide_request_struct
{
	uint8_t charge_seq_start_request :1;
	uint8_t charge_start_request :1;
	uint8_t charge_stop_request :1;
};

union powerSide_request_union
{
	uint8_t all;
	struct powerSide_request_struct bit;
};

struct networkSide_request_struct
{
	uint8_t start_Charge :1;
	uint8_t stop_Charge :1;
	uint8_t schedule_charge :1;
	uint8_t vehicle_Check :1;
	uint8_t charge_pause :1;
	uint8_t rtcUpdateComplete :1;
	uint8_t rtcUpdateAlarmComplete :1;
};

union networkSide_request_union
{
	uint8_t all;
	struct networkSide_request_struct bit;
};

struct networkSide_debug_struct
{
	uint8_t debug_mode :4;
	uint8_t debug_toggle :4;
};

union networkSide_debug_union
{
	uint8_t all;
	struct networkSide_debug_struct bit;
};

struct controlSide_data_struct
{
	union controlSide_status_union status;
	union controlSide_errorStatus_union errorStatus;
	union output_union outputs;
	union input_union inputs;
	struct controlSide_controlPilot_struct controlPilot;
	union powerSide_request_union powerSide_request;
	union networkSide_request_union networkSide_request;
	union warning_union warnings;
	uint8_t errorReport;
	float mcuTemp;
};

struct cp_data_struct
{
	uint16_t cpmax;
	uint16_t cpmin;
	uint16_t cpzero;
	float cpvoltage;
};

extern volatile struct cp_data_struct cpdata;

//################################################################################
//--------------------------------------------------------------------------------
//                  Power side data structures
//--------------------------------------------------------------------------------
typedef enum
{
	POWER_ON = 0, IDLE, START, READY, CHARGE, STOP, TEMP_FAULT, FAULT
} powerSideStateType;

typedef enum
{
	OFF = 0, ON, CONTACTOR_ERROR
} conctactState_type;

struct powerSide_status_struct
{
	uint8_t powerSideState :5;
	uint8_t powerSide_ready :1;
	uint8_t contactor_state :2;
};

union powerSide_status_union
{
	uint8_t all;
	struct powerSide_status_struct bit;
};

struct powerSide_tripStatus_struct
{
	uint8_t trip_OC :1;
	uint8_t trip_GFI :1;
};

union powerSide_tripStatus_union
{
	uint8_t all;
	struct powerSide_tripStatus_struct bit;
};

struct powerSide_errorStatus_struct
{
	uint8_t error_OV :1;
	uint8_t error_UV :1;
	uint8_t error_SR_C :1;
	uint8_t error_GFI_test :1;
	uint8_t error_Freq :1;
	uint8_t error_OT :1;
	uint8_t error_PL :1;
	uint8_t error_CFB :1;
};

union powerSide_errorStatus_union
{
	uint8_t all;
	struct powerSide_errorStatus_struct bit;
};

struct powerSide_voltageVal_struct
{
	uint16_t VA;
	uint16_t VB;
	uint16_t VC;
};

struct powerSide_currentVal_struct
{
	uint16_t IA;
	uint16_t IB;
	uint16_t IC;
};

struct powerSide_powerVal_struct
{
	uint8_t power;
	uint32_t kWh;
	uint16_t reserved;
};

struct powerSide_data_struct
{
	union powerSide_status_union status;
	union powerSide_errorStatus_union errorStatus;
	union powerSide_tripStatus_union tripStatus;
	struct tempSensors_struct tempSensors;
	struct powerSide_voltageVal_struct voltage;
	struct powerSide_currentVal_struct current;
	struct powerSide_powerVal_struct powerEnergy;
	uint16_t frequency;
};

//################################################################################
//--------------------------------------------------------------------------------
//                  Network side data structures
//--------------------------------------------------------------------------------
//typedef enum
//{
//	POWER_ON_STATE = 1,
//	IDLE_STATE,
//	TAPCARD_STATE,
//	AUTHCARD_STATE,
//	AUTHPASS_STATE,
//	AUTHFAIL_STATE,
//	PRE_START_STATE,
//	START_STATE,
//	PRE_STOP_STATE,
//	STOP_STATE,
//	CLOSE_STATE,
//	FAULT_STATE
//} networkSideStateType;

typedef enum
{
	POWERON_STATE = 1,
	IDLE_STATE,
	CONFIG_STATE,
	PRE_START_STATE,
	START_STATE,
	STOP_STATE,
	CURRENT_STATE,
	PREVIOUS_STATE,
	FAULT_STATE,
} networkSideStateType;

union networkSide_status_union
{
	uint16_t all;
	uint8_t netwrokSideState;
};

struct networkSide_data_struct
{
	uint16_t status;
	uint16_t errors;
	uint8_t maxCurrent_req;
	uint8_t stopCharge;
	uint8_t ledCommnad;
	uint8_t chargerLock;
	uint8_t scheduelCharge_active;
	uint8_t timeReady;
	uint8_t scheduleCharge;
	uint8_t alarmUpdate;
	uint8_t loadBalancing_en;
	uint8_t isInternet_available;
	uint8_t ledOnOff_command;
	RTC_TimeTypeDef setTime;
	RTC_TimeTypeDef weekdayOn;
	RTC_TimeTypeDef weekdayOff;
	RTC_TimeTypeDef weekendOn;
	RTC_TimeTypeDef weekendOff;
	RTC_DateTypeDef setDate;
//	union networkSide_status_union networkSideState;
};

extern volatile struct controlSide_data_struct controlSide_data;
extern volatile struct powerSide_data_struct powerSide_data;
extern volatile struct networkSide_data_struct networkSide_data;

void init_dataStructures(void);

#define ADC_CHANNEL 11
extern volatile uint16_t adc_store[ADC_CHANNEL];

#define NTC_INSIDE_INDEX					0
#define MCU_PHASE_C_MEASURE_INDEX			1
#define MCU_EARTH_V_SENSE					2
#define MCU_CP_READ_INDEX					3
#define MCU_NTC_LIN_NIN_INDEX				4
#define MCU_NTC_RELAY_COIL_INDEX			5
#define MCU_NTC_LOUT_NOUT_INDEX				6
#define MCU_VOLTAGE_SENSE_INDEX				7
#define MCU_STUCK_RELAY_DETECTION_INDEX		8

extern volatile uint32_t counter;
extern volatile uint32_t counter2;

extern volatile uint8_t relay1;
extern volatile uint8_t relay2;

extern volatile float rms_vin;
extern volatile float rms_vout;

extern volatile bool cpMin_reached;

extern volatile bool networkSide_bootup;

extern volatile bool charger_init_flag;
extern volatile bool charger_setup_flag;

extern volatile bool buzzer_en;
void buzzer_on(void);
void buzzer_poweron(void);

extern volatile float vrefint;
extern volatile float VREF;
extern volatile float VREF_SUM;
extern volatile float VREF_AVG;
extern volatile uint32_t vref_count;
extern volatile float Temperature;

#endif /* INC_DATA_STRUCTURES_H_ */
