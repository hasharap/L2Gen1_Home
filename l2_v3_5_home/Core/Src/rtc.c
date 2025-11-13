/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    rtc.c
  * @brief   This file provides code for the configuration
  *          of the RTC instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "rtc.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

RTC_HandleTypeDef hrtc;

/* RTC init function */
void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
  DateToUpdate.Month = RTC_MONTH_JANUARY;
  DateToUpdate.Date = 0x1;
  DateToUpdate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

void HAL_RTC_MspInit(RTC_HandleTypeDef* rtcHandle)
{

  if(rtcHandle->Instance==RTC)
  {
  /* USER CODE BEGIN RTC_MspInit 0 */

  /* USER CODE END RTC_MspInit 0 */
    HAL_PWR_EnableBkUpAccess();
    /* Enable BKP CLK enable for backup registers */
    __HAL_RCC_BKP_CLK_ENABLE();
    /* RTC clock enable */
    __HAL_RCC_RTC_ENABLE();
  /* USER CODE BEGIN RTC_MspInit 1 */

  /* USER CODE END RTC_MspInit 1 */
  }
}

void HAL_RTC_MspDeInit(RTC_HandleTypeDef* rtcHandle)
{

  if(rtcHandle->Instance==RTC)
  {
  /* USER CODE BEGIN RTC_MspDeInit 0 */

  /* USER CODE END RTC_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_RTC_DISABLE();
  /* USER CODE BEGIN RTC_MspDeInit 1 */

  /* USER CODE END RTC_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
//################################################################################
//--------------------------------------------------------------------------------
//                  RTC Variables
//--------------------------------------------------------------------------------

//============ Current Date and Time ============
RTC_TimeTypeDef RtcTime;
RTC_DateTypeDef RtcDate;

//============ Update Date and Time ============
RTC_TimeTypeDef rtcUpdate_time;
RTC_DateTypeDef rtcUpdate_date;

//============ Alarm Times ============
RTC_TimeTypeDef weekdayOn;
RTC_TimeTypeDef weekdayOff;
RTC_TimeTypeDef weekendOn;
RTC_TimeTypeDef weekendOff;

volatile bool rtcUpdate;
volatile bool rtcUpdateComplete;
volatile bool rtcUpdateAlarm;
volatile bool rtcUpdateAlarmComplete;
volatile bool alarmWeekday_active;
volatile bool alarmWeekend_active;

volatile bool bootup_timeUpdate = false;
volatile bool bootup_networkTimeUpdate = false;
volatile bool schedule_timeUpdate = false;
//scheduleOnOff = 3 - off
//scheduleOnOff = 5 - on

volatile uint8_t scheduleOnOff;
volatile uint8_t alarmCount = 0;

volatile uint16_t currentTime = 0;
volatile uint16_t offTime = 0;
volatile uint16_t onTime = 0;

volatile bool scheduleCharge_active = false;
uint16_t weekday_on = 0;
uint16_t weekend_on = 0;

#define timeMax	2359

#define HOUR_BCKP_REG 1
#define MIN_BCKP_REG 2
#define SEC_BCKP_REG 3
#define WEEKDAY_BCKP_REG 4
#define MONTH_BCKP_REG 5
#define DATE_BCKP_REG 6
#define YEAR_BCKP_REG 7
#define WEEKDAY_CURR 2
#define MONTH_CURR 2
#define DATE_CURR 3
#define YEAR_CURR 20

uint16_t unix1 = 0;
uint16_t unix2 = 0;
volatile uint32_t unix_time_set = 0;
volatile uint32_t unix_time_get = 0;
RTC_TimeTypeDef unix_time = {0};
RTC_DateTypeDef unix_date = {0};

//################################################################################
//--------------------------------------------------------------------------------
//			RTC initialization Function
//--------------------------------------------------------------------------------
void initRTC(void)
{
	RtcTime.Hours = 0;
	RtcTime.Minutes = 0;
	RtcTime.Seconds = 0;

	RtcDate.Date = 0;
	RtcDate.Month = 0;
	RtcDate.Year = 0;
	RtcDate.WeekDay = 0;

	rtcUpdate_time.Hours = 0;
	rtcUpdate_time.Minutes = 0;
	rtcUpdate_time.Seconds = 0;

	rtcUpdate_date.Date = 0;
	rtcUpdate_date.Month = 0;
	rtcUpdate_date.Year = 0;
	rtcUpdate_date.WeekDay = 0;

	weekdayOn.Hours = 0;
	weekdayOn.Minutes = 0;
	weekdayOn.Seconds = 0;

	weekdayOff.Hours = 0;
	weekdayOff.Minutes = 0;
	weekdayOff.Seconds = 0;

	weekendOn.Hours = 0;
	weekendOn.Minutes = 0;
	weekendOn.Seconds = 0;

	weekendOff.Hours = 0;
	weekendOff.Minutes = 0;
	weekendOff.Seconds = 0;

	rtcUpdate = false;
	rtcUpdateComplete = false;
	rtcUpdateAlarm = false;
	rtcUpdateAlarmComplete = false;
	alarmWeekday_active = false;
	alarmWeekend_active = false;

	alarmCount = 0;
	scheduleOnOff = 0;
}

#define JULIAN_DATE_BASE     2440588
// Convert Date/Time structures to epoch time
uint32_t RTC_ToEpoch(RTC_TimeTypeDef *time, RTC_DateTypeDef *date)
{
	uint8_t a;
	uint16_t y;
	uint8_t m;
	uint32_t JDN;

	// These hardcore math's are taken from http://en.wikipedia.org/wiki/Julian_da
	// Calculate some coefficients
	a = (14 - date->Month) / 12;
	y = (date->Year + 2000) + 4800 - a; // years since 1 March, 4801 BC
	m = date->Month + (12 * a) - 3; // since 1 March, 4801 BC

	// Gregorian calendar date compute
	JDN = date->Date;
	JDN += (153 * m + 2) / 5;
	JDN += 365 * y;
	JDN += y / 4;
	JDN += -y / 100;
	JDN += y / 400;
	JDN = JDN - 32045;
	JDN = JDN - JULIAN_DATE_BASE;    // Calculate from base date
	JDN *= 86400;                     // Days to seconds
	JDN += time->Hours * 3600;    // ... and today seconds
	JDN += time->Minutes * 60;
	JDN += time->Seconds;

	return JDN;
}

// Convert epoch time to Date/Time structures
void RTC_FromEpoch(uint32_t epoch, RTC_TimeTypeDef *time, RTC_DateTypeDef *date)
{
	uint32_t tm;
	uint32_t t1;
	uint32_t a;
	uint32_t b;
	uint32_t c;
	uint32_t d;
	uint32_t e;
	uint32_t m;
	int16_t year = 0;
	int16_t month = 0;
	int16_t dow = 0;
	int16_t mday = 0;
	int16_t hour = 0;
	int16_t min = 0;
	int16_t sec = 0;
	uint64_t JD = 0;
	uint64_t JDN = 0;

	// These hardcore math's are taken from http://en.wikipedia.org/wiki/Julian_day

	JD = ((epoch + 43200) / (86400 >> 1)) + (2440587 << 1) + 1;
	JDN = JD >> 1;

	tm = epoch;
	t1 = tm / 60;
	sec = tm - (t1 * 60);
	tm = t1;
	t1 = tm / 60;
	min = tm - (t1 * 60);
	tm = t1;
	t1 = tm / 24;
	hour = tm - (t1 * 24);

	dow = JDN % 7;
	a = JDN + 32044;
	b = ((4 * a) + 3) / 146097;
	c = a - ((146097 * b) / 4);
	d = ((4 * c) + 3) / 1461;
	e = c - ((1461 * d) / 4);
	m = ((5 * e) + 2) / 153;
	mday = e - (((153 * m) + 2) / 5) + 1;
	month = m + 3 - (12 * (m / 10));
	year = (100 * b) + d - 4800 + (m / 10);

	date->Year = year - 2000;
	date->Month = month;
	date->Date = mday;
	date->WeekDay = dow;
	time->Hours = hour;
	time->Minutes = min;
	time->Seconds = sec;
}

void get_date(void)
{
	unix1 = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1);
	unix2 = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR2);
	unix_time_get = ((unix2 << 16) | unix1);
	RTC_FromEpoch(unix_time_get, &unix_time, &unix_date);

	hrtc.DateToUpdate.Date = unix_date.Date;
	hrtc.DateToUpdate.Month = unix_date.Month;
	hrtc.DateToUpdate.Year = unix_date.Year;

	uint32_t year = 0U, weekday = 0U;

	year = 2000U + unix_date.Year;

	if (unix_date.Month < 3U) {
		/*D = { [(23 x month)/9] + day + 4 + year + [(year-1)/4] - [(year-1)/100] + [(year-1)/400] } mod 7*/
		weekday = (((23U * unix_date.Month) / 9U) + unix_date.Date + 4U + year + ((year - 1U) / 4U)
				- ((year - 1U) / 100U) + ((year - 1U) / 400U)) % 7U;
	} else {
		/*D = { [(23 x month)/9] + day + 4 + year + [year/4] - [year/100] + [year/400] - 2 } mod 7*/
		weekday = (((23U * unix_date.Month) / 9U) + unix_date.Date + 4U + year + (year / 4U)
				- (year / 100U) + (year / 400U) - 2U) % 7U;
	}

	hrtc.DateToUpdate.WeekDay = weekday;
}

void get_alarm(void)
{
	onTime = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR3);
	offTime = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR4);
	weekday_on = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR5);
}

//################################################################################
//--------------------------------------------------------------------------------
//			RTC update time Function
//--------------------------------------------------------------------------------
void updateRTC()
{
	RTC_TimeTypeDef sTime =
	{ 0 };
	RTC_DateTypeDef sDate =
	{ 0 };

	sTime.Hours = networkSide_data.setTime.Hours;
	sTime.Minutes = networkSide_data.setTime.Minutes;
	sTime.Seconds = 0;

	if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
	{
		Error_Handler();
	}

	HAL_RTC_GetTime(&hrtc, &RtcTime, RTC_FORMAT_BIN);

	sDate.Date = networkSide_data.setDate.Date;
	sDate.Month = networkSide_data.setDate.Month;
	sDate.Year = networkSide_data.setDate.Year;

	if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
	{
		Error_Handler();
	}

	HAL_RTC_GetDate(&hrtc, &RtcDate, RTC_FORMAT_BIN);

	unix_time_set = RTC_ToEpoch(&sTime, &sDate);

	//HAL_PWR_EnableBkUpAccess();
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, (unix_time_set & 0xFFFF));
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR2, (unix_time_set >> 16));
	//HAL_PWR_DisableBkUpAccess();

	controlSide_data.networkSide_request.bit.rtcUpdateComplete = true;
}

//################################################################################
//--------------------------------------------------------------------------------
//			RTC update alarm Function
//--------------------------------------------------------------------------------
void updateAlarm()
{
	weekdayOn.Hours = networkSide_data.weekdayOn.Hours;
	weekdayOn.Minutes = networkSide_data.weekdayOn.Minutes;
	weekdayOff.Hours = networkSide_data.weekdayOff.Hours;
	weekdayOff.Minutes = networkSide_data.weekdayOff.Minutes;

	weekendOn.Hours = networkSide_data.weekendOn.Hours;
	weekendOn.Minutes = networkSide_data.weekendOn.Minutes;
	weekendOff.Hours = networkSide_data.weekendOff.Hours;
	weekendOff.Minutes = networkSide_data.weekendOff.Minutes;

	if (scheduleCharge_active == true)
	{
		if (weekend_on == 1)
		{
			onTime = ((weekendOn.Hours * 100) + weekendOn.Minutes);
			offTime = ((weekendOff.Hours * 100) + weekendOff.Minutes);
		}

		if (weekend_on == 2)
		{
			onTime = ((weekdayOn.Hours * 100) + weekdayOn.Minutes);
			offTime = ((weekdayOff.Hours * 100) + weekdayOff.Minutes);
		}

//		HAL_PWR_EnableBkUpAccess();
//		HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR3, onTime);
//		HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR4, offTime);
//		HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR5, weekend_on);
//		HAL_PWR_DisableBkUpAccess();
	}

	controlSide_data.networkSide_request.bit.rtcUpdateAlarmComplete = true;
}

//################################################################################
//--------------------------------------------------------------------------------
//			RTC check schedule charge Function
//--------------------------------------------------------------------------------
void checkWeekday_Time(void)
{
	currentTime = ((RtcTime.Hours * 100) + RtcTime.Minutes);

	if (networkSide_data.scheduleCharge == 1)
	{
		if (scheduleCharge_active == false)
		{
			   schedule_timeUpdate = true;
			//Check if weekday is Saturday or Sunday
			if ((RtcDate.WeekDay == RTC_WEEKDAY_SATURDAY) || (RtcDate.WeekDay == RTC_WEEKDAY_SUNDAY))
			{
				onTime = ((weekendOn.Hours * 100) + weekendOn.Minutes);
				offTime = ((weekendOff.Hours * 100) + weekendOff.Minutes);
				weekend_on = 1;
			}
			else
			{
				onTime = ((weekdayOn.Hours * 100) + weekdayOn.Minutes);
				offTime = ((weekdayOff.Hours * 100) + weekdayOff.Minutes);
				weekend_on = 2;
			}
		}

		if (offTime < onTime)
		{
			if (currentTime >= onTime)
			{
				scheduleOnOff = 5;
				scheduleCharge_active = true;
			}
			else
			{
				if (currentTime < offTime)
				{
					scheduleOnOff = 5;
					scheduleCharge_active = true;
				}
				else
				{
					scheduleOnOff = 3;
					scheduleCharge_active = false;
				}
			}
		}
		else
		{
			if ((currentTime >= onTime) && (currentTime < offTime))
			{
				scheduleOnOff = 5;
				scheduleCharge_active = true;
			}
			else
			{
				scheduleOnOff = 3;
				scheduleCharge_active = false;
			}
		}
	}
	else
	{
		scheduleOnOff = 0;
		scheduleCharge_active = false;
	}
}
/* USER CODE END 1 */
