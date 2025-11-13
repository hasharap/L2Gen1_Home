/**
 * @file STM32F103_EEPROM.c
 * @brief Implements EEPROM emulation on the STM32F103 using Flash memory.
 *
 * @details This module provides functions to initialize, erase, write, and read
 *          data from the Flash memory, simulating EEPROM functionality.
 *
 * @date November 19, 2023
 * @version 1.0
 * @author
 * bulbu
 */

#include "STM32F103_EEPROM.h"

/**
 * @brief Initializes the Flash memory for write and erase operations.
 *
 * @details Unlocks the Flash memory to allow write and erase operations. Should
 *          be called before any Flash memory operations and followed by a lock
 *          operation when done.
 */
void Flash_Init(void)
{
    HAL_FLASH_Unlock();
    //   __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
}

/**
 * @brief Erases a page of Flash memory.
 *
 * @param address The starting address of the Flash memory page to erase.
 * @param size The number of pages to erase (currently set to 1 in the implementation).
 */
void EEPROM_Erase(uint32_t address, uint32_t size)
{
    Flash_Init();

    FLASH_EraseInitTypeDef eraseInitStruct;
    eraseInitStruct.TypeErase = TYPEERASE_PAGES;
    eraseInitStruct.PageAddress = address; /**< Specify the start address of the page. */
    eraseInitStruct.NbPages = 1; /**< Specify the number of pages to erase. */

    uint32_t error;

    HAL_FLASHEx_Erase(&eraseInitStruct, &error);

    HAL_FLASH_Lock();
}

/**
 * @brief Writes data to the Flash memory.
 *
 * @param address The starting address to write the data.
 * @param data Pointer to the data buffer containing the data to be written.
 * @param size The number of 16-bit words to write.
 */
void EEPROM_WriteData(uint32_t address, uint16_t *data, uint16_t size)
{
    Flash_Init();

    for (uint16_t i = 0; i < size; i++)
    {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address, data[i]);
        address += 2; // Increment by the word size (16 bits).
    }

    HAL_FLASH_Lock();
}

/**
 * @brief Reads data from the Flash memory.
 *
 * @param address The starting address to read the data from.
 * @param data Pointer to the buffer where the read data will be stored.
 * @param size The number of 16-bit words to read.
 */
void EEPROM_ReadData(uint32_t address, uint16_t *data, uint32_t size)
{
    for (uint32_t i = 0; i < size; i++)
    {
        data[i] = *(__IO uint16_t*) address;
        address += 2; // Increment by the word size (16 bits).
    }
}
