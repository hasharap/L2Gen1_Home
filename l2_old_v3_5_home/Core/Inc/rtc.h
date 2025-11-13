/*
 * rtc.h
 *
 *  Created on: Jan 4, 2023
 *      Author: Nisal Bulathsinghala
 */

#ifndef INC_RTC_H_
#define INC_RTC_H_

#include "main.h"

extern RTC_HandleTypeDef hrtc;

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

#endif /* INC_RTC_H_ */
