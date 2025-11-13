/*
 * timer.h
 *
 *  Created on: Feb 20, 2020
 *      Author: Nisal
 */

#ifndef TIMER_H_
#define TIMER_H_

#include "main.h"
//typedef enum { false, true } bool;

typedef struct tick_tock_struct
{
	uint32_t counter;
	bool timeout_0_1s;
	bool timeout_0_2s;
	bool timeout_0_5s;
	bool timeout_1s;
	bool timeout_2s;
	bool timeout_3s;
	bool timeout_4s;
	bool timeout_6s;
	bool timeout_8s;
	bool timeout_10s;
	bool timeout_15s;
	bool timeout_30s;
	bool timeout_1m;
	bool timeout_5m;
} tick_tock;

void tick_clear(volatile tick_tock *timer_ptr);
void tick_count(volatile tick_tock *timer_ptr);

extern volatile tick_tock timeout;
extern volatile tick_tock gfi_test;
extern volatile tick_tock rtc_checkCounter;
extern volatile tick_tock rtc_updateCounter;
extern volatile tick_tock load_balance6s;
extern volatile tick_tock error_handler;

#endif /* TIMER_H_ */
