/**
 * @file OTA.h
 * @brief Header file for the OTA (Over-the-Air) update functionality.
 *
 * @details This file declares the functions and constants used for performing
 *          OTA updates, including reading and writing to Flash memory.
 *
 * @date January 26, 2024
 * @version 1.0
 * @attention
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component. If no LICENSE file comes
 * with this software, it is provided AS-IS.
 */

#ifndef OTA_H
#define OTA_H

#include "main.h"
#include "stm32f1xx_hal.h"

/**
 * @brief Flag indicating whether an OTA update is active.
 */
extern volatile bool OTA_flag;

/**
 * @defgroup Memory_Locations Memory Locations
 * @brief Memory addresses used for the OTA process.
 * @{
 */
#define BOOTLOADER_LOCATION     0x08000000 /**< Starting address of the bootloader in Flash memory. */
#define CONFIGURATION_LOCATION  0x08002C00 /**< Address for configuration data in Flash memory. */
#define APPLICATION_LOCATION    0x08003800 /**< Starting address of the application in Flash memory. */
/** @} */

/**
 * @brief Maximum size of a data block used during OTA updates.
 */
#define MAX_BLOCK_SIZE          (1024)

/**
 * @brief Writes data to Flash memory.
 *
 * @param StartAddress The start address in Flash memory where the data will be written.
 * @param Data Pointer to the data buffer to be written.
 * @param WordSize The size of the data buffer in words.
 * @return 0 on success, or an error code from `HAL_FLASH_GetError()` on failure.
 */
uint32_t Write_Data(uint32_t StartAddress, uint32_t *Data, uint16_t WordSize);

/**
 * @brief Reads data from Flash memory.
 *
 * @param StartAddress The start address in Flash memory to read from.
 * @param RxBuff Pointer to the buffer to store the read data.
 * @param WordSize The size of the data to read in words.
 */
void Read_Data(uint32_t StartAddress, uint32_t *RxBuff, uint16_t WordSize);

/**
 * @brief Sets the OTA flag and triggers a system reset.
 *
 * @details Writes the OTA update flag to a specified Flash memory location
 *          and performs a system reset to initiate the update process.
 */
void Flag_up();

#endif /* OTA_H */
