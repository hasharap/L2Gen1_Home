/*
 * STM32F103_EEPROM.h
 *
 *  Created on: Nov 19, 2023
 *      Author: bulbu
 */

#ifndef INC_STM32F103_EEPROM_H_
#define INC_STM32F103_EEPROM_H_

#include "main.h"

//Page 127 - 1K
#define EEROM_START_ADDRESS ((uint32_t) 0x08004B00)
#define EEPROM_SIZE 		((uint32_t) 1024)

#define HW_VERSION_LOCATION	((uint32_t)	0x08002BF0)
#define OTA_LOCATION		((uint32_t) 0x08002C00)
#define CONFIG_LOCATION		((uint32_t) 0x08003000)
#define CP_LOCATION			((uint32_t) 0x08003430)

void Flash_Init(void);
void EEPROM_Erase(uint32_t address, uint32_t size);
void EEPROM_WriteData( uint32_t address, uint16_t* data, uint16_t size);
void EEPROM_ReadData(uint32_t address, uint16_t* data, uint32_t size);

#endif /* INC_STM32F103_EEPROM_H_ */
