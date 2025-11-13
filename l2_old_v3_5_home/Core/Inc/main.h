/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "eeprom.h"
#include "string.h"
#include "stdbool.h"
#include "timer.h"
#include "gpio_hal.h"
#include "STM32F103_EEPROM.h"
#include "OTA.h"
#include "data_structures.h"
#include "rtc.h"
#include "protection.h"
#include "modbus_devices/modbusRTU.h"
#include "modbus_devices/eMeter.h"
#include "power_measure.h"
#include "control_pilot.h"
#include "gpio_feedback.h"
#include "temp_NTC.h"
#include "CSMS.h"
#include "state_machine.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart3;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
#define SERIAL_ACTIVE 			1
#define SERIAL_ERROR_CHECK		0
#define RTC_ACTIVE				1
#define DIODE_CHECK				0
#define LED_STRIP				0
#define WATCHDOG_ENABLE			0

#define THREE_PHASE				0

#define BEB_TOGGLE				0

#define A_TO_C_Transition		1
#define UV_dynamic				0

#define CP_FAULT_CHECK			0
#define GFI_ERROR				0
#define OC_ERROR				1
#define	GFI_TEST_ERROR			0
#define OV_ERROR				1
#define UV_ERROR				1
#define PL_ERROR				1
#define FREQ_ERROR				0
#define SC_ERROR				0

#define ERROR_ALARM				0
/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define C_SENSE_Pin GPIO_PIN_1
#define C_SENSE_GPIO_Port GPIOA
#define EARTH_V_SENSE_Pin GPIO_PIN_2
#define EARTH_V_SENSE_GPIO_Port GPIOA
#define CP_READ_Pin GPIO_PIN_3
#define CP_READ_GPIO_Port GPIOA
#define IN_T_SENSE_Pin GPIO_PIN_4
#define IN_T_SENSE_GPIO_Port GPIOA
#define RELAY_T_SENSE1_Pin GPIO_PIN_5
#define RELAY_T_SENSE1_GPIO_Port GPIOA
#define OUT_T_SENSE_Pin GPIO_PIN_6
#define OUT_T_SENSE_GPIO_Port GPIOA
#define EC_TEST_Pin GPIO_PIN_7
#define EC_TEST_GPIO_Port GPIOA
#define V_SENSE_Pin GPIO_PIN_0
#define V_SENSE_GPIO_Port GPIOB
#define SRD_SENSE_Pin GPIO_PIN_1
#define SRD_SENSE_GPIO_Port GPIOB
#define GFIC_TEST_SENSE_Pin GPIO_PIN_12
#define GFIC_TEST_SENSE_GPIO_Port GPIOB
#define OCP_RESET_Pin GPIO_PIN_13
#define OCP_RESET_GPIO_Port GPIOB
#define GFIC_SENSE_Pin GPIO_PIN_14
#define GFIC_SENSE_GPIO_Port GPIOB
#define GFIC_TEST_Pin GPIO_PIN_15
#define GFIC_TEST_GPIO_Port GPIOB
#define PHASE_SET_Pin GPIO_PIN_15
#define PHASE_SET_GPIO_Port GPIOA
#define GFIC_RESET_Pin GPIO_PIN_3
#define GFIC_RESET_GPIO_Port GPIOB
#define BOARD_LED_Pin GPIO_PIN_4
#define BOARD_LED_GPIO_Port GPIOB
#define C_FB_Pin GPIO_PIN_5
#define C_FB_GPIO_Port GPIOB
#define BUZZER_Pin GPIO_PIN_6
#define BUZZER_GPIO_Port GPIOB
#define MODE_SET_Pin GPIO_PIN_7
#define MODE_SET_GPIO_Port GPIOB
#define L_RELAY_Pin GPIO_PIN_8
#define L_RELAY_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
extern long serial_rx_counter;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
