/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    rtc.h
  * @brief   This file contains all the function prototypes for
  *          the rtc.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __RTC_H__
#define __RTC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern RTC_HandleTypeDef hrtc;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_RTC_Init(void);

/* USER CODE BEGIN Prototypes */
//============ Current Date and Time ============
extern RTC_TimeTypeDef RtcTime;
extern RTC_DateTypeDef RtcDate;

//============ Update Date and Time ============
extern RTC_TimeTypeDef rtcUpdate_time;
extern RTC_DateTypeDef rtcUpdate_date;

//============ Alarm Times ============
extern RTC_TimeTypeDef weekdayOn;
extern RTC_TimeTypeDef weekdayOff;
extern RTC_TimeTypeDef weekendOn;
extern RTC_TimeTypeDef weekendOff;

extern volatile bool rtcUpdate;
extern volatile bool rtcUpdateComplete;
extern volatile bool rtcUpdateAlarm;
extern volatile bool rtcUpdateAlarmComplete;
extern volatile bool alarmWeekday_active;
extern volatile bool alarmWeekend_active;

extern volatile bool bootup_timeUpdate;
extern volatile bool bootup_networkTimeUpdate;
extern volatile bool schedule_timeUpdate;

extern volatile uint8_t alarmCount;
extern volatile uint8_t scheduleOnOff;

extern volatile bool scheduleCharge_active;

extern volatile uint16_t currentTime;
extern volatile uint16_t offTime;
extern volatile uint16_t onTime;

void initRTC(void);
void get_date(void);
void get_alarm(void);
void updateRTC();
void updateAlarm();
void checkWeekday_Time(void);
void RTC_FromEpoch(uint32_t epoch, RTC_TimeTypeDef *time, RTC_DateTypeDef *date);
uint32_t RTC_ToEpoch(RTC_TimeTypeDef *time, RTC_DateTypeDef *date);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __RTC_H__ */

