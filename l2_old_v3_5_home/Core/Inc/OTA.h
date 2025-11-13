/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : OTA.h
 * @brief          : Header for OTA.c file.
 *                   This file contains the common defines of the application.
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

#include "main.h"
#include "stm32f1xx_hal.h"

extern volatile bool OTA_flag;

/* Memory Locations */
#define BOOTLOADER_LOCATION 		0x08000000
#define CONFIGURATION_LOCATION 		0x08004100
#define APPLICATION_LOCATION 		0x08005000
#define SLOT_LOCATION 				0x08009400

#define MAX_BLOCK_SIZE          ( 1024 )

//extern uint16_t Data[1];

uint32_t Write_Data(uint32_t StartAddress, uint32_t *Data, uint16_t WordSize);

void Read_Data(uint32_t StartAddress, uint32_t *RxBuff, uint16_t WordSize);
void Flag_up();
