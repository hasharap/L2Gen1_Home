/**
 * @file charge_config.c
 * @brief This file contains functions for handling charger configuration and hardware version management.
 *
 * @details Provides functionalities for reading and writing charger configurations
 *          and retrieving hardware version information from EEPROM.
 *
 * @date November 18, 2024
 * @author Nisal
 */

#include "charger_config.h"
#include "OTA.h"

/** @brief Charger configuration structure for retrieved data */
volatile struct charger_config_struct charger_configGet;

/** @brief Charger configuration structure for updated data */
volatile struct charger_config_struct charger_configSet;

/** @brief Array to hold configuration data retrieved from EEPROM */
volatile uint16_t config_get[9] = {};

/** @brief Array to hold configuration data to be written to EEPROM */
volatile uint16_t config_set[9] = {};

/** @brief Hardware version structure */
volatile struct hw_version_struct hw_version;

/** @brief Array to hold hardware version data retrieved from EEPROM */
volatile uint16_t hw_verion_get[2] = {};

/**
 * @brief Reads the charger configuration from EEPROM.
 *
 * @details This function reads the charger configuration data from the EEPROM at a
 *          predefined location and populates the `charger_configGet` structure.
 */
void get_chargerConfig(void)
{
    EEPROM_ReadData(CONFIG_LOCATION, (uint16_t *)config_get, sizeof(config_get) / sizeof(config_get[0]));
    HAL_Delay(100);

    charger_configGet.en_1.all = config_get[0];
    charger_configGet.en_2.all = config_get[1];
    charger_configGet.uv_upper = config_get[2];
    charger_configGet.uv_lower = config_get[3];
    charger_configGet.ov_upper = config_get[4];
    charger_configGet.ov_lower = config_get[5];
    charger_configGet.freq_upper = config_get[6];
    charger_configGet.freq_lower = config_get[7];
    charger_configGet.max_current = (config_get[8] & 0xFF);
}

/**
 * @brief Writes the charger configuration to EEPROM.
 *
 * @details This function updates the charger configuration data in the EEPROM at a
 *          predefined location with values from the `charger_configSet` structure.
 */
void set_chargerConfig(void)
{
    config_set[0] = charger_configSet.en_1.all;
    config_set[1] = charger_configSet.en_2.all;
    config_set[2] = charger_configSet.uv_upper;
    config_set[3] = charger_configSet.uv_lower;
    config_set[4] = charger_configSet.ov_upper;
    config_set[5] = charger_configSet.ov_lower;
    config_set[6] = charger_configSet.freq_upper;
    config_set[7] = charger_configSet.freq_lower;
    config_set[8] = (uint16_t)charger_configSet.max_current;

    EEPROM_Erase(CONFIG_LOCATION, EEPROM_SIZE);
    EEPROM_WriteData(CONFIG_LOCATION, (uint16_t *)config_set, sizeof(config_get) / sizeof(config_get[0]));
	HAL_Delay(100);
	HAL_NVIC_SystemReset();
}

/**
 * @brief Reads the hardware version information from EEPROM.
 *
 * @details This function reads the hardware version data from the EEPROM at a
 *          predefined location and populates the `hw_version` structure.
 */
void get_HWVersion(void)
{
    EEPROM_ReadData(HW_VERSION_LOCATION, (uint16_t *)hw_verion_get, sizeof(hw_verion_get) / sizeof(hw_verion_get[0]));
    HAL_Delay(100);
    hw_version.v1 = (uint8_t)(hw_verion_get[0] & 0xFF);
    hw_version.v2 = (uint8_t)(hw_verion_get[0] >> 8);
    hw_version.v3 = (uint8_t)(hw_verion_get[1] & 0xFF);
}
