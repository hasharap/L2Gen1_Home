/**
 * @file charger_config.h
 * @brief Header file for charger configuration management.
 *
 * @details Defines structures and function prototypes for handling
 *          charger configuration and hardware version information.
 *
 * @date November 18, 2024
 * @author Nisal
 */

#ifndef INC_CHARGER_CONFIG_H_
#define INC_CHARGER_CONFIG_H_

#include "main.h"

/**
 * @brief Structure to hold hardware version information.
 */
struct hw_version_struct
{
    uint8_t v1; /**< Major version */
    uint8_t v2; /**< Minor version */
    uint8_t v3; /**< Patch version */
};



/**
 * @brief Structure to hold charger configuration data.
 */
struct charger_config_struct
{
    /** @brief Union to enable or disable various features (group 1). */
    union
    {
        uint16_t all; /**< Full 16-bit value */
        struct
        {
            uint8_t uv_en :2;     /**< Under-voltage enable */
            uint8_t ov_en :2;     /**< Over-voltage enable */
            uint8_t sc_en :2;     /**< Short-circuit enable */
            uint8_t cpf_en :2;    /**< Charge profile enable */
            uint8_t gfi_en :2;    /**< Ground fault interrupt enable */
            uint8_t gfit_en :2;   /**< Ground fault interrupt threshold enable */
            uint8_t pl_en :2;     /**< Phase loss enable */
            uint8_t freq_en :2;   /**< Frequency enable */
        } bit; /**< Individual feature bits */
    } en_1;

    /** @brief Union to enable or disable various features (group 2). */
    union
    {
        uint16_t all; /**< Full 16-bit value */
        struct
        {
            uint8_t modbus_en :2; /**< Modbus enable */
            uint8_t penf_en :2;   /**< PEN fault enable */
            uint8_t ot_en :2;     /**< Over-temperature enable */
            uint8_t resv_2 :2;    /**< Reserved */
            uint8_t resv_3 :2;    /**< Reserved */
            uint8_t resv_4 :2;    /**< Reserved */
            uint8_t resv_5 :2;    /**< Reserved */
            uint8_t resv_6 :2;    /**< Reserved */
        } bit; /**< Individual feature bits */
    } en_2;

    uint8_t max_current;      /**< Maximum current allowed */
    uint16_t uv_upper;        /**< Upper limit for under-voltage */
    uint16_t uv_lower;        /**< Lower limit for under-voltage */
    uint16_t ov_upper;        /**< Upper limit for over-voltage */
    uint16_t ov_lower;        /**< Lower limit for over-voltage */
    uint16_t freq_upper;      /**< Upper limit for frequency */
    uint16_t freq_lower;      /**< Lower limit for frequency */
    uint8_t config_counter;   /**< Configuration counter */
    bool config_enable;       /**< Enable or disable configuration */
    uint8_t energyMeterModel;
    int16_t cp_positive;
    int16_t cp_negative;
    uint8_t cp_set;
};

/** @brief Charger configuration data for retrieval. */
extern volatile struct charger_config_struct charger_configGet;

/** @brief Charger configuration data for updating. */
extern volatile struct charger_config_struct charger_configSet;

/** @brief Hardware version information. */
extern volatile struct hw_version_struct hw_version;

/**
 * @brief Reads the charger configuration from EEPROM.
 */
void get_chargerConfig(void);

/**
 * @brief Writes the charger configuration to EEPROM.
 */
void set_chargerConfig(void);

/**
 * @brief Reads the hardware version information from EEPROM.
 */
void get_HWVersion(void);

#endif /* INC_CHARGER_CONFIG_H_ */
