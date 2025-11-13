/**
 * @file gpio_feedback.h
 * @brief Header file for GPIO feedback handling.
 *
 * @details This header declares the functions used to process GPIO feedback
 *          for monitoring and managing the contactor state.
 *
 * @date January 26, 2024
 * @author
 * Nisal Bulathsinghalage
 */

#ifndef INC_GPIO_FEEDBACK_H_
#define INC_GPIO_FEEDBACK_H_

#include "main.h"

/**
 * @brief Retrieves and processes GPIO feedback.
 *
 * @details This function reads GPIO input feedback to determine the state
 *          of the contactor and updates the relevant system state variables.
 */
void get_GPIOFeedback(void);

#endif /* INC_GPIO_FEEDBACK_H_ */
