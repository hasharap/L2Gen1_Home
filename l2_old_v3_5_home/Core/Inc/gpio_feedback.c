/*
 * gpio_feedbacl.c
 *
 *  Created on: Jan 26, 2024
 *      Author: Nisal Bulathsinghalage
 */

#include "gpio_feedback.h"

#define checkCounter	5
uint8_t CFB_inCounter = 0;
uint8_t CFB_outCounter = 0;

static void get_ContactorFB(void)
{
	if (C_FB() == 1)
	{
		CFB_outCounter = 0;
		CFB_inCounter++;
		if (CFB_inCounter >= checkCounter)
		{
			CFB_inCounter = 0;
			powerSide_data.status.bit.contactor_state = ON;
		}
	}
	else
	{
		CFB_inCounter = 0;
		CFB_outCounter++;
		if (CFB_outCounter >= checkCounter)
		{
			CFB_outCounter = 0;
			powerSide_data.status.bit.contactor_state = OFF;
		}
	}
}

void get_GPIOFeedback(void)
{
	get_ContactorFB();
}
