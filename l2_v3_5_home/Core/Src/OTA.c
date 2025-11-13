/**
 * @file OTA.c
 * @brief Implements functions for Over-the-Air (OTA) update operations.
 *
 * @details This module includes functionalities for reading and writing data to
 *          Flash memory and managing OTA update flags.
 *
 * @date January 26, 2024
 * @version 1.0
 */

#include "OTA.h"

/** @brief Flag indicating whether OTA update is active. */
volatile bool OTA_flag = false;

/**
 * @brief Retrieves the start address of the Flash page for a given address.
 *
 * @param Address The memory address to calculate the page.
 * @return The start address of the Flash page containing the given address.
 */
static uint32_t GetPage(uint32_t Address)
{
    for (int indx = 0; indx < 128; indx++)
    {
        if ((Address < (0x08000000 + (FLASH_PAGE_SIZE * (indx + 1)))) &&
            (Address >= (0x08000000 + FLASH_PAGE_SIZE * indx)))
        {
            return (0x08000000 + FLASH_PAGE_SIZE * indx);
        }
    }
    return 0;
}

/**
 * @brief Writes data to Flash memory.
 *
 * @param StartAddress The start address in Flash memory where data will be written.
 * @param Data Pointer to the data buffer to be written.
 * @param WordSize The size of the data buffer in words.
 * @return 0 on success, or an error code from `HAL_FLASH_GetError()` on failure.
 */
uint32_t Write_Data(uint32_t StartAddress, uint32_t *Data, uint16_t WordSize)
{
    static FLASH_EraseInitTypeDef EraseInitStruct;
    uint32_t PAGEError;
    int sofar = 0;

    /* Unlock the Flash memory for write access. */
    HAL_FLASH_Unlock();

    /* Calculate the start and end Flash pages. */
    uint32_t StartPage = GetPage(StartAddress);
    uint32_t EndPageAddress = StartAddress + WordSize * 4;
    uint32_t EndPage = GetPage(EndPageAddress);

    /* Configure Flash page erase settings. */
    EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    EraseInitStruct.PageAddress = StartPage;
    EraseInitStruct.NbPages = ((EndPage - StartPage) / FLASH_PAGE_SIZE) + 1;

    /* Erase the specified Flash pages. */
    if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK)
    {
        /* Error occurred during Flash erase. */
        return HAL_FLASH_GetError();
    }

    /* Write data to Flash memory word by word. */
    while (sofar < WordSize)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, StartAddress, Data[sofar]) == HAL_OK)
        {
            StartAddress += 4; // Increment by word size (4 bytes).
            sofar++;
        }
        else
        {
            /* Error occurred during Flash programming. */
            return HAL_FLASH_GetError();
        }
    }

    /* Lock the Flash memory to prevent further modification. */
    HAL_FLASH_Lock();

    return 0;
}

/**
 * @brief Reads data from Flash memory.
 *
 * @param StartAddress The start address in Flash memory to read from.
 * @param RxBuf Pointer to the buffer to store the read data.
 * @param WordSize The size of the data to read in words.
 */
void Read_Data(uint32_t StartAddress, uint32_t *RxBuf, uint16_t WordSize)
{
    while (1)
    {
        *RxBuf = *(__IO uint32_t *)StartAddress;
        StartAddress += 4; // Increment by word size (4 bytes).
        RxBuf++;
        if (!(WordSize--))
            break;
    }
}

/**
 * @brief Sets the OTA update flag and resets the system.
 *
 * @details Writes the OTA update flag to Flash memory and triggers a system reset.
 */
void Flag_up()
{
    uint32_t Data[1];
    Data[0] = 0x10;
    Write_Data(CONFIGURATION_LOCATION, Data, 1);
    HAL_NVIC_SystemReset();
}
