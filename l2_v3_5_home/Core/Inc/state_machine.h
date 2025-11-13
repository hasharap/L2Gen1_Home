/*
 * state_machine.h
 *
 *  Created on: Aug 30, 2022
 *      Author: Nisal Bulathsinghala
 */


#ifndef INC_STATE_MACHINE_H_
#define INC_STATE_MACHINE_H_

//#include "data_structures.h"
#include "main.h"

#define MAX_TRANSITIONS 25 // Maximum number of transitions to store

#define TIM_TICK_TIME	0.5	//500us

typedef enum
{
	STATE_POWER_ON = 0,
	STATE_A1,
	STATE_A2,
	STATE_B1,
	STATE_B2,
	STATE_C1,
	STATE_C2,
	STATE_D,
	STATE_F,
	TEMP_STATE_F,
	STATE_COUNT
} StateType;

extern volatile StateType currentState;
extern volatile StateType previousState;
extern volatile StateType previousState_temp;
extern volatile bool stateEntry_flag;

typedef struct
{
	StateType fromState;
	StateType toState;
} StateTransition;

typedef struct
{
	StateType currentState;
	StateType previousState;
	void (*stateFunctions[STATE_COUNT])(); // Array of function pointers for each state
    StateTransition transitions[MAX_TRANSITIONS]; // Array to store transitions
    int transitionCount;
} StateMachine;

// Structure to store state transitions


extern volatile StateMachine sm;

void initializeStateMachine(StateMachine *sm);
void changeState(StateMachine *sm, StateType newState);
void runStateMachine(StateMachine *sm);

void sm_control_run(void);

#endif /* INC_STATE_MACHINE_H_ */
