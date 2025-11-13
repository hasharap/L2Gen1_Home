/**
 * @file timer.h
 * @brief Header file for timer-based timeout management.
 *
 * @details Provides structures and function declarations for managing timers
 *          with predefined timeout intervals. Includes multiple global timer
 *          instances for specific system tasks.
 *
 * @date February 20, 2020
 * @version 1.0
 * @author
 * Nisal
 */

#ifndef TIMER_H_
#define TIMER_H_

#include "main.h"

/**
 * @brief Structure to manage timeout intervals for a timer.
 */
typedef struct tick_tock_struct
{
    uint32_t counter;      /**< Counter to track elapsed ticks. */
    bool timeout_0_1s;     /**< Timeout flag for 0.1 seconds. */
    bool timeout_0_2s;     /**< Timeout flag for 0.2 seconds. */
    bool timeout_0_5s;     /**< Timeout flag for 0.5 seconds. */
    bool timeout_1s;       /**< Timeout flag for 1 second. */
    bool timeout_2s;       /**< Timeout flag for 2 seconds. */
    bool timeout_3s;       /**< Timeout flag for 3 seconds. */
    bool timeout_4s;       /**< Timeout flag for 4 seconds. */
    bool timeout_6s;       /**< Timeout flag for 6 seconds. */
    bool timeout_8s;       /**< Timeout flag for 8 seconds. */
    bool timeout_10s;      /**< Timeout flag for 10 seconds. */
    bool timeout_15s;      /**< Timeout flag for 15 seconds. */
    bool timeout_30s;      /**< Timeout flag for 30 seconds. */
    bool timeout_1m;       /**< Timeout flag for 1 minute. */
    bool timeout_5m;       /**< Timeout flag for 5 minutes. */
    bool timeout_10m;      /**< Timeout flag for 10 minutes. */
} tick_tock;

/**
 * @brief Resets all timeout flags and the counter in the specified timer structure.
 *
 * @param timer_ptr Pointer to the `tick_tock` structure to be cleared.
 */
void tick_clear(volatile tick_tock *timer_ptr);

/**
 * @brief Increments the timer counter and updates timeout flags based on predefined intervals.
 *
 * @param timer_ptr Pointer to the `tick_tock` structure to be updated.
 */
void tick_count(volatile tick_tock *timer_ptr);

/** @brief Global timer instance for general timeouts. */
extern volatile tick_tock timeout;

/** @brief Global timer instance for GFI (Ground Fault Interruption) testing. */
extern volatile tick_tock gfi_test;

/** @brief Global timer instance for RTC (Real-Time Clock) check operations. */
extern volatile tick_tock rtc_checkCounter;

/** @brief Global timer instance for RTC updates. */
extern volatile tick_tock rtc_updateCounter;

/** @brief Global timer instance for serial error detection. */
extern volatile tick_tock serial_error;

/** @brief Global timer instance for load balancing every 6 seconds. */
extern volatile tick_tock load_balance6s;

/** @brief Global timer instance for error handling. */
extern volatile tick_tock error_handler;

#endif /* TIMER_H_ */
