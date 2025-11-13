/*
 * timer.c
 *
 *  Created on: Aug 30, 2022
 *      Author: Nisal Bulathsinghala
 */
#include "timer.h"

void tick_clear(volatile tick_tock *timer_ptr)
{
	timer_ptr->counter = 0;
	timer_ptr->timeout_0_1s = 0;
	timer_ptr->timeout_0_2s = 0;
	timer_ptr->timeout_0_5s = 0;
	timer_ptr->timeout_1s = 0;
	timer_ptr->timeout_2s = 0;
	timer_ptr->timeout_3s = 0;
	timer_ptr->timeout_4s = 0;
	timer_ptr->timeout_6s = 0;
	timer_ptr->timeout_8s = 0;
	timer_ptr->timeout_10s = 0;
	timer_ptr->timeout_15s = 0;
	timer_ptr->timeout_30s = 0;
	timer_ptr->timeout_1m = 0;
	timer_ptr->timeout_5m = 0;
}

void tick_count(volatile tick_tock *timer_ptr)
{
	timer_ptr->counter++;

	switch (timer_ptr->counter)
	{
		case 200:
			timer_ptr->timeout_0_1s = true;
			break;
		case 400:
			timer_ptr->timeout_0_2s = true;
			break;
		case 1000:
			timer_ptr->timeout_0_5s = true;
			break;
		case 2000:
			timer_ptr->timeout_1s = true;
			break;
		case 4000:
			timer_ptr->timeout_2s = true;
			break;
		case 6000:
			timer_ptr->timeout_3s = true;
			break;
		case 8000:
			timer_ptr->timeout_4s = true;
			break;
		case 12000:
			timer_ptr->timeout_6s = true;
			break;
		case 16000:
			timer_ptr->timeout_8s = true;
			break;
		case 20000:
			timer_ptr->timeout_10s = true;
			break;
		case 30000:
			timer_ptr->timeout_15s = true;
			break;
		case 60000:
			timer_ptr->timeout_30s = true;
			break;
		case 120000:
			timer_ptr->timeout_1m = true;
			break;
		case 600000:
			timer_ptr->timeout_5m = true;
			break;
		default:
			break;
	}
}

volatile tick_tock timeout;
volatile tick_tock gfi_test;
volatile tick_tock rtc_checkCounter;
volatile tick_tock rtc_updateCounter;
volatile tick_tock load_balance6s;
volatile tick_tock error_handler;


