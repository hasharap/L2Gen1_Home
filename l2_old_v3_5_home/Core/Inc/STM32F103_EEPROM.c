/*
 * STM32F103_EEPROM.c
 *
 *  Created on: Nov 19, 2023
 *      Author: bulbu
 */

#include "STM32F103_EEPROM.h"

void Flash_Init(void)
{
	HAL_FLASH_Unlock();
	//   __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
}

void EEPROM_Erase(uint32_t address, uint32_t size)
{
	Flash_Init();
//    uint32_t TYPEERASE_PAGES = 0xffffffff;

	FLASH_EraseInitTypeDef eraseInitStruct;
	eraseInitStruct.TypeErase = TYPEERASE_PAGES;
	eraseInitStruct.PageAddress = address; /* Specify the start address of the pages */
	eraseInitStruct.NbPages = 1; /* Specify the number of pages to erase */

	uint32_t error;

	HAL_FLASHEx_Erase(&eraseInitStruct, &error);

	HAL_FLASH_Lock();
}

void EEPROM_WriteData(uint32_t address, uint16_t *data, uint16_t size)
{
	// uint32_t address = EEPROM_START_ADDRESS;
	Flash_Init();

	for (uint16_t i = 0; i < size; i++)
	{
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address, data[i]);
		address += 2; // Increment by the word size (16 bits)
	}

	HAL_FLASH_Lock();
}

void EEPROM_ReadData(uint32_t address, uint16_t *data, uint32_t size)
{
	// uint32_t address = EEPROM_START_ADDRESS;

	for (uint32_t i = 0; i < size; i++)
	{
		data[i] = *(__IO uint16_t*) address;
		address += 2; // Increment by the word size (16 bits)
	}
}
