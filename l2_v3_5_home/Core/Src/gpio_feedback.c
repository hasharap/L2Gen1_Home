/**
 * @file gpio_feedback.c
 * @brief Handles GPIO feedback operations for contactor state monitoring.
 *
 * @details This module reads the GPIO input feedback and determines the state
 *          of the contactor, updating the `powerSide_data` structure accordingly.
 *
 * @date January 26, 2024
 * @author
 * Nisal Bulathsinghalage
 */

#include "gpio_feedback.h"

/** @brief Counter threshold for contactor feedback validation. */
#define checkCounter 5

/** @brief Counter for contactor feedback when the input state is ON. */
uint8_t CFB_inCounter = 0;

/** @brief Counter for contactor feedback when the input state is OFF. */
uint8_t CFB_outCounter = 0;

/**
 * @brief Checks the contactor feedback and updates the contactor state.
 *
 * @details This function reads the contactor feedback GPIO using `C_FB()` and
 *          increments counters based on the input state. When the respective
 *          counter reaches `checkCounter`, the contactor state is updated in
 *          the `powerSide_data` structure.
 */
static void get_ContactorFB(void)
{
    if (C_FB() == 1)
    {
        CFB_outCounter = 0;
        CFB_inCounter++;
        if (CFB_inCounter >= checkCounter)
        {
            CFB_inCounter = 0;
            powerSide_data.status.bit.contactor_state = ON;
        }
    }
    else
    {
        CFB_inCounter = 0;
        CFB_outCounter++;
        if (CFB_outCounter >= checkCounter)
        {
            CFB_outCounter = 0;
            powerSide_data.status.bit.contactor_state = OFF;
        }
    }
}
/**
 * @brief Retrieves GPIO feedback for the system.
 *
 * @details This function is the main entry point for updating the contactor
 *          state based on GPIO feedback. It internally calls `get_ContactorFB()`.
 */
void get_GPIOFeedback(void)
{
    get_ContactorFB();
}
