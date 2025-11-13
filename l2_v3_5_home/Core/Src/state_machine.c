/*
 * state_machine.c
 *
 *  Created on: Aug 30, 2022
 *      Author: Nisal Bulathsinghalage
 */
#include "state_machine.h"

volatile StateType currentState = STATE_POWER_ON;
volatile StateType previousState = STATE_POWER_ON;
volatile StateType previousState_temp = STATE_POWER_ON;
volatile bool stateEntry_flag = true;

volatile StateMachine sm;

uint16_t stateA_counter = 0;
uint16_t stateB_counter = 0;
uint16_t stateC_counter = 0;
uint16_t stateD_counter = 0;

#define STATE_A_COUNT	(40 / TIM_TICK_TIME)	//1s
#define STATE_B_COUNT	(40 / TIM_TICK_TIME)	//1s
#define STATE_C_COUNT	(40 / TIM_TICK_TIME)	//1s

//#define STATE_A_COUNT   100
//#define STATE_B_COUNT   100
//#define STATE_C_COUNT   100
//#define STATE_D_COUNT   100

bool manual_off = false;
bool manual_on = false;
bool fault_detected = false;
bool under_volt_fault = false;
bool under_volt_detected = false;

//################################################################################
//--------------------------------------------------------------------------------
//                  Timeout macros
//--------------------------------------------------------------------------------

#define VARIABLE_INIT_STATEA1_TIMEOUT       timeout.timeout_4s
#define VARIABLE_INIT_STATEA2_TIMEOUT       timeout.timeout_4s
#define SEND_CHRGSEQSTRT_STATEB1_TIMEOUT    timeout.timeout_6s
#define STPBTN_INIT_STATEB1_TIMEOUT         timeout.timeout_4s
#define STATEB2_TIMEOUT                     timeout.timeout_30s
#define SEND_CHRGSEQSTRT_STATEB2_TIMEOUT    timeout.timeout_1s
#define STATEC1_TIMEOUT                     timeout.timeout_6s
#define VARIABLE_INIT_STATEC1_TIMEOUT       timeout.timeout_8s
#define SEND_CHRGSEQSTRT_STATEC1_TIMEOUT    timeout.timeout_15s
#define STATEF_TIMEOUT                      timeout.timeout_10s
#define CHRGSTP_UV_STATETEMPF_TIMEOUT       timeout.timeout_10s
#define TAPCARD_TIMEOUT                     timeout.timeout_30s

//################################################################################
//--------------------------------------------------------------------------------
//                  Function prototypes
//--------------------------------------------------------------------------------

void sm_control_run(void);
void power_on(void);
void stateA1(void);
void stateA2(void);
void stateB1(void);
void stateB2(void);
void stateC1(void);
void stateC2(void);
void stateD(void);
void stateF(void);
void temp_stateF(void);
void charger_variable_init(void);

//################################################################################
//--------------------------------------------------------------------------------
//                  State engine
//--------------------------------------------------------------------------------

void initializeStateMachine(StateMachine *sm)
{
	sm->currentState = STATE_POWER_ON;
	sm->previousState = STATE_POWER_ON;
	sm->stateFunctions[STATE_POWER_ON] = power_on;
	sm->stateFunctions[STATE_A1] = stateA1;
	sm->stateFunctions[STATE_A2] = stateA2;
	sm->stateFunctions[STATE_B1] = stateB1;
	sm->stateFunctions[STATE_B2] = stateB2;
	sm->stateFunctions[STATE_C1] = stateC1;
	sm->stateFunctions[STATE_C2] = stateC2;
	sm->stateFunctions[STATE_F] = stateF;
	sm->stateFunctions[TEMP_STATE_F] = temp_stateF;
	sm->transitionCount = 0;
}

void trackStateTransition(StateMachine *sm, StateType previousState,
		StateType currentState)
{
	if (sm->transitionCount < MAX_TRANSITIONS)
	{
		sm->transitions[sm->transitionCount].fromState = previousState;
		sm->transitions[sm->transitionCount].toState = currentState;
		sm->transitionCount++;
	}
	else
	{
		sm->transitionCount = 0;
		printf("Transition log is full!\n");
	}
}

void changeState(StateMachine *sm, StateType newState)
{
	if (newState >= 0 && newState < STATE_COUNT)
	{
		if (sm->currentState != newState)
		{ // Only log if state actually change
			sm->previousState = sm->currentState;
			sm->currentState = newState;
			trackStateTransition(sm, sm->previousState,
					sm->currentState);
		}
	}
	else
	{
		printf("Invalid state transition\n");
	}
}

void runStateMachine(StateMachine *sm)
{
	sm->stateFunctions[sm->currentState](sm);
}

void sm_control_run(void)
{
	switch (currentState)
	{
		case STATE_POWER_ON:
			power_on();
//			previousState_temp = STATE_POWER_ON;
		case STATE_A1:
			stateA1();
//			previousState_temp = STATE_A1;
			break;
		case STATE_A2:
//			stateA2();
//			previousState_temp = STATE_A2;
			break;
		case STATE_B1:
//			stateB1();
//			previousState_temp = STATE_B1;
			break;
		case STATE_B2:
			stateB2();
//			previousState_temp = STATE_B2;
			break;
		case STATE_C1:
			stateC1();
//			previousState_temp = STATE_C1;
			break;
		case STATE_C2:
			stateC2();
//			previousState_temp = STATE_C2;
			break;
		case STATE_F:
			stateF();
//			previousState_temp = STATE_F;
			break;
		case TEMP_STATE_F:
			temp_stateF();
//			previousState_temp = TEMP_STATE_F;
			break;
		default:
//        stateF();
//        previousState_temp = STATE_F;
			break;
	}
}

void stateA_transition(uint16_t a1a2)
{
	if (controlSide_data.controlPilot.cp_Vmax > STATEA_MIN
			&& controlSide_data.controlPilot.cp_Vmax < STATEA_MAX)
	{
		stateA_counter++;
		if (stateA_counter >= STATE_A_COUNT)
		{
			stateA_counter = 0;
			stateEntry_flag = true;
			if (a1a2 == 0)
			{
				changeState((StateMachine *)&sm, STATE_A1);
			}
			else
			{
				changeState((StateMachine *)&sm, STATE_A1);
			}
		}
	}
	else
	{
		stateA_counter = 0;
	}
}

void stateB_transition(uint16_t b1b2)
{
	if (controlSide_data.controlPilot.cp_Vmax > STATEB_MIN
			&& controlSide_data.controlPilot.cp_Vmax < STATEB_MAX)
	{
		stateB_counter++;
		if (stateB_counter == STATE_B_COUNT)
		{
			stateB_counter = 0;
			stateEntry_flag = true;
			if (b1b2 == 0)
			{
				changeState((StateMachine *)&sm, STATE_B1);
			}
			else
			{
				changeState((StateMachine *)&sm, STATE_B2);
			}
		}
	}
	else
	{
		stateB_counter = 0;
	}
}

void stateC_transition(uint16_t c1c2)
{
	if (controlSide_data.controlPilot.cp_Vmax > STATEC_MIN
			&& controlSide_data.controlPilot.cp_Vmax < STATEC_MAX)
	{
		stateC_counter++;
		if (stateC_counter == STATE_C_COUNT)
		{
			stateC_counter = 0;
			stateEntry_flag = true;
			if (c1c2 == 0)
			{
				changeState((StateMachine *)&sm, STATE_C1);
			}
			else
			{
				changeState((StateMachine *)&sm, STATE_C2);

			}
		}
	}
	else
	{
		stateC_counter = 0;
	}
}

void stateD_transition(void)
{
//	if (controlSide_data.controlPilot.cp_max > stateD_Vmin //jump to STATE F if STATE D is requested
//	&& controlSide_data.controlPilot.cp_max < stateD_Vmax)
//	{
//		stateD_counter++;
//		if (stateD_counter == STATE_D_COUNT)
//		{
//			stateD_counter = 0;
//			stateEntry_flag = true;
//			currentState = STATE_F;
//		}
//	}
//	else
//	{
//		stateD_counter = 0;
//	}
}

void power_on(void)
{
	/*---------------------------------------------------
	 One time code
	 ---------------------------------------------------*/
	if (stateEntry_flag == true)
	{
		stateEntry_flag = false;
		controlSide_data.status.bit.curretState = POWERON_STATE;
		bootup_vehicleCheck();
	}

	stateA_transition(0);
	stateB_transition(0);
	stateC_transition(0);
}

void stateA1(void)
{
	/*---------------------------------------------------
	 One time code
	 ---------------------------------------------------*/

	if (stateEntry_flag == true)
	{
		//stateEntry_flag = false;
		controlSide_data.status.bit.curretState = STATE_A1;	//Current state update
		diodeCheck_flag = false;							//Diode check
		run_GFITest = true;									//GFI test flag
		diodeCheck_passed = false;
		controlSide_data.controlPilot.cp_enable = 1;
		controlSide_data.controlPilot.cp_duty_enable = 0;

		tick_clear(&timeout);								//Clear timer
		L_RELAY_OFF();									//Turn off contactor

		//charger_variable_init(); //Stuck relay test flag
		if(networkSide_data.status==IDLE_STATE)
		{
			stateEntry_flag = false;
			charger_variable_init();
		}
		errorBuffer_clear();
	}

	/*---------------------------------------------------
		Looping code
	 ---------------------------------------------------*/
	if (timeout.timeout_1m == true)
	{
		sm.transitionCount = 0;
	}

	/*---------------------------------------------------
		State transition
	 ---------------------------------------------------*/
	stateB_transition(0);
#if A_TO_C_Transition
	stateC_transition(0);
#endif
	/*---------------------------------------------------
		Fault handler
	 ---------------------------------------------------*/
	if (controlSide_data.errorStatus.bit.fault_level != 0)
	{
		stateEntry_flag = true;
		changeState((StateMachine *)&sm, STATE_F);
	}
}

void stateA2(void)
{
	/*---------------------------------------------------
	 	One time code
	 ---------------------------------------------------*/
	if (stateEntry_flag == true)
	{
		stateEntry_flag = false;

		controlSide_data.status.bit.curretState = STATE_A2;	//Current state update
		controlSide_data.controlPilot.cp_enable = 1;
		controlSide_data.controlPilot.cp_duty_enable = 1;
		diodeCheck_passed = false;

		if (controlSide_data.status.bit.charging_active == 1)
		{
			controlSide_data.networkSide_request.bit.start_Charge = 0;
			controlSide_data.networkSide_request.bit.stop_Charge = 1;
			controlSide_data.networkSide_request.bit.schedule_charge = 0;
		}

		tick_clear(&timeout);								//Clear timer

		L_RELAY_OFF();										//Turn off contactor
	}

	/*---------------------------------------------------
		Looping code
	 ---------------------------------------------------*/



	/*---------------------------------------------------
		State transition
	 ---------------------------------------------------*/
	//if idle command not received within 4s second, forcefully jump to STATE A1
	if ((VARIABLE_INIT_STATEA2_TIMEOUT == true)
			|| networkSide_data.status == IDLE_STATE)
	{
		stateEntry_flag = true;
		changeState((StateMachine *)&sm, STATE_A1);
	}

	/*---------------------------------------------------
		Fault handler
	 ---------------------------------------------------*/
	if (controlSide_data.errorStatus.bit.fault_level != 0)
	{
		stateEntry_flag = true;
		changeState((StateMachine *)&sm, STATE_F);
	}
}

void stateB1(void)
{
	/*---------------------------------------------------
	 	One time code
	 ---------------------------------------------------*/
	if (stateEntry_flag == true)
	{
		stateEntry_flag = false;

		controlSide_data.status.bit.curretState = STATE_B1;	//Current state update
		controlSide_data.controlPilot.cp_enable = 1;
		controlSide_data.controlPilot.cp_duty_enable = 0;
		diodeCheck_passed = false;

		if (controlSide_data.status.bit.charging_active == 1)
		{
			controlSide_data.status.bit.charging_active = 0;
			powerSide_data.powerEnergy.kWh = 0;
			start_energy = 0;
			controlSide_data.networkSide_request.bit.start_Charge = 0;
			controlSide_data.networkSide_request.bit.stop_Charge = 1;
			controlSide_data.networkSide_request.bit.schedule_charge = 0;
			powerSide_data.status.bit.powerSide_ready = 0;
		}

		tick_clear(&timeout);								//Clear timer

		L_RELAY_OFF();										//Turn off contactor
	}

	/*---------------------------------------------------
		Looping code
	 ---------------------------------------------------*/
	if ((timeout.timeout_2s == true)
			&& (controlSide_data.errorStatus.bit.fault_level == 0))
	{


		if (networkSide_data.status == PRE_START_STATE)
		{
			stateEntry_flag = true;
			changeState((StateMachine *)&sm, STATE_B2);
		}
#if RTC_ACTIVE
		if ((networkSide_data.scheduleCharge == 1)
				&& (networkSide_data.chargerLock == 1))
		{
			if (scheduleOnOff == 5)
			{
				stateEntry_flag = true;
				currentState = STATE_B2;
				//added by me
				changeState((StateMachine *)&sm, STATE_B2);
			}
		}
#endif
	}

	if (networkSide_data.status == STOP_STATE)
	{
		controlSide_data.networkSide_request.bit.start_Charge = 0;
		controlSide_data.networkSide_request.bit.stop_Charge = 0;
	}

	/*---------------------------------------------------
		State transition
	 ---------------------------------------------------*/
	stateA_transition(0);

	/*---------------------------------------------------
		Fault handler
	 ---------------------------------------------------*/
	if (controlSide_data.errorStatus.bit.fault_level == PRIORITY_2)
	{
		stateEntry_flag = true;
		changeState((StateMachine *)&sm, STATE_F);
	}
	else if (controlSide_data.errorStatus.bit.fault_level == PRIORITY_1)
	{
		stateEntry_flag = true;
		changeState((StateMachine *)&sm, TEMP_STATE_F);
	}
}

void stateB2(void)
{
	/*---------------------------------------------------
	 	One time code
	 ---------------------------------------------------*/
	if (stateEntry_flag == true)
	{
		stateEntry_flag = false;

		controlSide_data.status.bit.curretState = STATE_B2;	//Current state update
		controlSide_data.controlPilot.cp_enable = 1;
		controlSide_data.controlPilot.cp_duty_enable = 1;
		diodeCheck_passed = false;

//		if (controlSide_data.status.bit.charging_active == 1)
//		{
//			controlSide_data.status.bit.charging_active = 0;
//			controlSide_data.networkSide_request.bit.start_Charge = 0; //30/09/2020
//			controlSide_data.networkSide_request.bit.stop_Charge = 1;
//			controlSide_data.networkSide_request.bit.schedule_charge = 0;
//		}
//		else
		if (controlSide_data.status.bit.charging_active == 0)
		{
			diodeCheck_flag = true;
		}

		tick_clear(&timeout);								//Clear timer

		L_RELAY_OFF();										//Turn off contactor
	}

	/*---------------------------------------------------
		Looping code
	 ---------------------------------------------------*/
	if (networkSide_data.status == STOP_STATE)
	{
		controlSide_data.networkSide_request.bit.start_Charge = 0; //30/09/2020
		controlSide_data.networkSide_request.bit.stop_Charge = 0;
		stateEntry_flag = true;
		changeState((StateMachine *)&sm, STATE_B1);
	}

#if RTC_ACTIVE
	if ((networkSide_data.scheduleCharge == 1)
			&& (networkSide_data.chargerLock == 1))
	{

		if (scheduleOnOff != 5)
		{
			stateEntry_flag = true;
			currentState = STATE_B1;
		}
	} else {
		if (networkSide_data.status == IDLE_STATE) {
			stateEntry_flag = true;
			currentState = STATE_B1;
		}
	}
#else
	if (networkSide_data.status == IDLE_STATE)
	{
		stateEntry_flag = true;
		changeState((StateMachine *)&sm, STATE_B1);
	}
#endif

#if BEB_TOGGLE
	//B1->B2 Toggle and BEB Toggle
	if (timeout.timeout_30s == true)
	{
		stateEntry_flag = true;
		currentState = STATE_B1;
	}
#endif

	/*---------------------------------------------------
		State transition
	 ---------------------------------------------------*/
	stateA_transition(1);

	if (diodeCheck_passed == true)
	{
		stateC_transition(1);
	}

	stateD_transition();

	/*---------------------------------------------------
		Fault handler
	 ---------------------------------------------------*/
	if (controlSide_data.errorStatus.bit.fault_level == PRIORITY_2)
	{
		stateEntry_flag = true;
		changeState((StateMachine *)&sm, STATE_F);
	}
	else if (controlSide_data.errorStatus.bit.fault_level == PRIORITY_1)
	{
		stateEntry_flag = true;
		changeState((StateMachine *)&sm, TEMP_STATE_F);
	}
}

void stateC1(void)
{
	/*---------------------------------------------------
	 	One time code
	 ---------------------------------------------------*/
	if (stateEntry_flag == true)
	{
		stateEntry_flag = false;
		controlSide_data.status.bit.curretState = STATE_C1;	//Current state update
		controlSide_data.controlPilot.cp_enable = 1;
		controlSide_data.controlPilot.cp_duty_enable = 0;
		diodeCheck_passed = false;

		if (controlSide_data.status.bit.charging_active == 1)
		{
			controlSide_data.status.bit.charging_active = 0;
			//powerSide_data.powerEnergy.kWh = 0;
			start_energy = 0;
			controlSide_data.networkSide_request.bit.start_Charge = 0;
			controlSide_data.networkSide_request.bit.stop_Charge = 1;
			controlSide_data.networkSide_request.bit.schedule_charge = 0;
		}

		tick_clear(&timeout);								//Clear timer

		L_RELAY_OFF();										//Turn off contactor
	}

	/*---------------------------------------------------
		State transition
	 ---------------------------------------------------*/
	if (networkSide_data.status == STOP_STATE)
	{
		controlSide_data.networkSide_request.bit.start_Charge = 0;
		controlSide_data.networkSide_request.bit.stop_Charge = 0;
	}

	if ((timeout.timeout_6s == true)
			&& (controlSide_data.errorStatus.bit.fault_level == 0))
	{
		if (networkSide_data.status == PRE_START_STATE)
		{
			stateEntry_flag = true;
			changeState((StateMachine *)&sm, STATE_C2);
		}

		if (networkSide_data.status == START_STATE)
		{
			stateEntry_flag = true;
			changeState((StateMachine *)&sm, STATE_C2);
		}
	}

#if RTC_ACTIVE
	if ((networkSide_data.scheduleCharge == 1)
			&& (networkSide_data.chargerLock == 1)
			/*&& (schedule_timeUpdate == true)*/)
	{
		if (scheduleOnOff == 5)
		{
			stateEntry_flag = true;
			currentState = STATE_C2;
			BOARD_LED_ON();
			changeState((StateMachine *)&sm, STATE_C2);
		}
		else
		{
			//Handle unstopped schedule after power toggle and exceeding schedule time
			if (networkSide_data.scheduelCharge_active == 1)
			{
				controlSide_data.networkSide_request.bit.start_Charge = 0;
				controlSide_data.networkSide_request.bit.stop_Charge = 1;
				controlSide_data.networkSide_request.bit.schedule_charge = 0;
			}
		}
	}
#endif

	stateA_transition(0);
	stateB_transition(0);
	stateD_transition();

	/*---------------------------------------------------
		Fault handler
	 ---------------------------------------------------*/
	//	if ((powerSide_data.errorStatus.all != 0)
	//			|| (powerSide_data.tripStatus.all != 0)
	//			|| (controlSide_data.errorStatus.all != 0)) //Handle errors in the control side jump to STATE C1
	//	{
	//		stateEntry_flag = true;
	//		currentState = STATE_F;
	//	}

	if (controlSide_data.errorStatus.bit.fault_level == PRIORITY_2)
	{
		stateEntry_flag = true;
		changeState((StateMachine *)&sm, STATE_F);
	}
	else if (controlSide_data.errorStatus.bit.fault_level == PRIORITY_1)
	{
		stateEntry_flag = true;
		changeState((StateMachine *)&sm, TEMP_STATE_F);
	}
}

void stateC2(void)
{
	/*---------------------------------------------------
	 	One time code
	 ---------------------------------------------------*/
	if (stateEntry_flag == true)
	{
		stateEntry_flag = false;
		controlSide_data.status.bit.curretState = STATE_C2;
		controlSide_data.controlPilot.cp_enable = 1;
		controlSide_data.controlPilot.cp_duty_enable = 1;
		controlSide_data.networkSide_request.bit.start_Charge = 1;
		controlSide_data.networkSide_request.bit.stop_Charge = 0;

		buzzer_en = true;

		if (controlSide_data.status.bit.charging_active == 0)
		{
			powerSide_data.powerEnergy.kWh = 0;
			start_energy = 0;
			measure_energy = true;
		}

		controlSide_data.status.bit.charging_active = 1;

#if	RTC_ACTIVE
		if ((networkSide_data.scheduleCharge == 1)
				&& (networkSide_data.chargerLock == 1)
				&& (manual_on == false))
		{
			if (scheduleOnOff == 5)
			{
				controlSide_data.networkSide_request.bit.schedule_charge = 1;
			}
		}
#endif

		tick_clear(&timeout);
		GFI_bypasss_flag = true;
	}

	if (timeout.timeout_1s == true)
	{
		L_RELAY_ON();
	}

#if RTC_ACTIVE
	/*---------------------------------------------------
	 Stop charge during schedule charge
	 ---------------------------------------------------*/
	if ((networkSide_data.scheduleCharge == 1)
			&& (networkSide_data.chargerLock == 1))
	{
		if (scheduleOnOff != 5)
		{
			stateEntry_flag = true;
			currentState = STATE_C1;
			changeState((StateMachine *)&sm, STATE_C1);
		}
	}
#endif

	if ((networkSide_data.status == STOP_STATE)
			|| (networkSide_data.stopCharge == 1)) //Nisal 19JAN2024
	{
		stateEntry_flag = true;
		manual_off = true;
		manual_on = false;
		changeState((StateMachine *)&sm, STATE_C1);
	}

	if (timeout.timeout_30s == true)
	{
		tick_clear(&timeout);
		if (networkSide_data.status != START_STATE)
		{
			stateEntry_flag = true;
			changeState((StateMachine *)&sm, STATE_C1);
		}
	}

	/*---------------------------------------------------
		State transition
	 ---------------------------------------------------*/
	stateA_transition(1);
	stateB_transition(1);
	stateD_transition();

	/*---------------------------------------------------
		Fault handler
	 ---------------------------------------------------*/
	if (controlSide_data.errorStatus.bit.fault_level == PRIORITY_2)
	{
		stateEntry_flag = true;
		changeState((StateMachine *)&sm, STATE_F);
	}
	else if (controlSide_data.errorStatus.bit.fault_level == PRIORITY_1)
	{
		stateEntry_flag = true;
		changeState((StateMachine *)&sm, TEMP_STATE_F);
	}
}

void stateF(void)
{
	if (stateEntry_flag == true)
	{
		stateEntry_flag = false;
		controlSide_data.status.bit.curretState = STATE_F;
		controlSide_data.controlPilot.cp_enable = 0;
		controlSide_data.controlPilot.cp_duty_enable = 0;
		controlSide_data.status.bit.charging_active = 0;
		controlSide_data.networkSide_request.bit.stop_Charge = 1;
		controlSide_data.networkSide_request.bit.start_Charge = 0;
		controlSide_data.networkSide_request.bit.schedule_charge = 0;
		tick_clear(&timeout);

		L_RELAY_OFF();
	}

	if (networkSide_data.status == STOP_STATE)
	{
		controlSide_data.networkSide_request.bit.start_Charge = 0;
		controlSide_data.networkSide_request.bit.stop_Charge = 0;
	}

	if ((STATEF_TIMEOUT == true))
	{
		controlSide_data.controlPilot.cp_enable = 1;

		if ((controlSide_data.errorStatus.bit.fault_level == 0))
		{
			stateA_transition(0);
		}
	}
}

void temp_stateF(void)
{
    if (stateEntry_flag == true)
    {
        stateEntry_flag = false;
        controlSide_data.status.bit.curretState = TEMP_STATE_F;
		controlSide_data.controlPilot.cp_enable = 1;
		controlSide_data.controlPilot.cp_duty_enable = 0;

		L_RELAY_OFF();

		tick_clear(&timeout);
    }

    if (timeout.timeout_4s == true)
	{
		if (controlSide_data.errorStatus.bit.fault_level == 0)
		{
			stateA_transition(0);
			if (controlSide_data.status.bit.charging_active == 1)
			{
				stateB_transition(1);
				stateC_transition(1);
			}
			else
			{
				stateB_transition(0);
				stateC_transition(0);
			}
		}
	}

	if (controlSide_data.errorStatus.bit.fault_level == PRIORITY_2)
	{
		stateEntry_flag = true;
		changeState((StateMachine *)&sm, STATE_F);
	}
}

void charger_variable_init(void)
{
	controlSide_data.status.bit.charging_active = 0;
	controlSide_data.controlPilot.cp_enable = 1;
	controlSide_data.controlPilot.cp_duty_enable = 0;
	controlSide_data.errorStatus.all = 0;

	controlSide_data.powerSide_request.all = 0;
	controlSide_data.networkSide_request.bit.start_Charge = 0;
	controlSide_data.networkSide_request.bit.stop_Charge = 0;
	controlSide_data.networkSide_request.bit.vehicle_Check = 0;
	controlSide_data.networkSide_request.bit.charge_pause = 0;

	powerSide_data.status.bit.powerSide_ready = 0;


    powerSide_data.powerEnergy.kWh = 0;

	controlSide_data.warnings.all = 0;

	networkSide_data.errors = 0;
	networkSide_data.status = 0;

	diodeCheck_passed = false;
	manual_on = false;
	manual_off = false;

	measure_energy = false;
	start_energy = 0;
}
