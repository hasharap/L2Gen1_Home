/**
 * @file STM32F103_EEPROM.h
 * @brief Header file for EEPROM emulation on the STM32F103 using Flash memory.
 *
 * @details This file provides the function declarations and defines used for
 *          EEPROM-like operations using Flash memory on STM32F103 devices.
 *
 * @date November 19, 2023
 * @version 1.0
 * @author
 * bulbu
 */

#ifndef INC_STM32F103_EEPROM_H_
#define INC_STM32F103_EEPROM_H_

#include "main.h"

/**
 * @defgroup EEPROM_Memory_Locations Memory Locations
 * @brief Flash memory locations for EEPROM emulation.
 * @{
 */
#define EEROM_START_ADDRESS  ((uint32_t)0x08004B00) /**< Starting address of the EEPROM emulation. */
#define EEPROM_SIZE          ((uint32_t)1024)       /**< Size of the EEPROM emulation in bytes. */
#define HW_VERSION_LOCATION  ((uint32_t)0x08002BF0) /**< Address for hardware version storage. */
#define OTA_LOCATION         ((uint32_t)0x08002C00) /**< Address for OTA flag storage. */
#define CONFIG_LOCATION      ((uint32_t)0x08003000) /**< Address for configuration data storage. */
#define CP_LOCATION          ((uint32_t)0x08003430) /**< Address for CP-related data storage. */
/** @} */

/**
 * @brief Initializes the Flash memory for write and erase operations.
 *
 * @details Unlocks the Flash memory to allow write and erase operations.
 *          Should be called before any Flash operations and followed by a
 *          lock operation after completing the operations.
 */
void Flash_Init(void);

/**
 * @brief Erases a page in Flash memory.
 *
 * @param address The starting address of the page to erase.
 * @param size The number of pages to erase.
 */
void EEPROM_Erase(uint32_t address, uint32_t size);

/**
 * @brief Writes data to Flash memory.
 *
 * @param address The starting address where the data will be written.
 * @param data Pointer to the buffer containing data to be written.
 * @param size The size of the data buffer in 16-bit words.
 */
void EEPROM_WriteData(uint32_t address, uint16_t* data, uint16_t size);

/**
 * @brief Reads data from Flash memory.
 *
 * @param address The starting address to read data from.
 * @param data Pointer to the buffer where the read data will be stored.
 * @param size The size of the data to read in 16-bit words.
 */
void EEPROM_ReadData(uint32_t address, uint16_t* data, uint32_t size);

#endif /* INC_STM32F103_EEPROM_H_ */
