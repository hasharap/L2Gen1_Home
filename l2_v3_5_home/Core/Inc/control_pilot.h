/**
 ******************************************************************************
 * @file           : control_pilot.h
 * @brief          : Header for control_pilot.c file.
 *					 This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Author: Nisal Bulathsinghala, Jul 27, 2022
 * Copyright (c) 2022 Vega Innovations, Sri Lanka.
 * All rights reserved.
 ******************************************************************************
 */

#ifndef INC_CONTROL_PILOT_H_
#define INC_CONTROL_PILOT_H_

#include "main.h"

#define PWMFULLON   	100
#define PWMFULLOFF  	0
#define PWM_TEMP_FAULT 	10

#define CPDUTY_DEFAULT	25 // 15
#define CPMAX			53
#define CPMIN			10

#define CP_FAULT		1.8

#define STATEA_MAX	130
#define STATEA_MIN	106
#define STATEB_MAX	104
#define STATEB_MIN	76
#define STATEC_MAX	74
#define STATEC_MIN	46
#define STATEF_MIN	(-105)
#define STATEF_MAX	(-130)

extern uint16_t stateA_Vmin, stateA_Vmax;
extern uint16_t stateB_Vmin, stateB_Vmax;
extern uint16_t stateC_Vmin, stateC_Vmax;
extern uint16_t stateD_Vmin, stateD_Vmax;
extern uint16_t stateE_Vmin, stateE_Vmax;
extern uint16_t stateF_Vmin, stateF_Vmax;

extern volatile bool diodeCheck_flag;
extern volatile bool diodeCheck_passed;
extern volatile bool loadBalance_active;

void cp_map(bool cp_calib);
void getCP_voltage(void);
void setCP_duty(void);
void load_balance(bool uv_en);
void monitor_cp(void);
void calibrateCP(int action);
void check_diode(void);
void bootup_vehicleCheck(void);
void check_vehicleConnector(void);
void check_PWMActive(void);

#endif /* INC_CONTROL_PILOT_H_ */
