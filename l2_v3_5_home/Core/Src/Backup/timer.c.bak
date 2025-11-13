/**
 * @file timer.c
 * @brief Implements a timer-based system for managing timeouts and intervals.
 *
 * @details This module provides functions to initialize and update timers for
 *          various timeout durations. It also defines global timer instances
 *          for specific system tasks.
 *
 * @date August 30, 2022
 * @version 1.0
 * @author
 * Nisal Bulathsinghala
 */

#include "timer.h"

/**
 * @brief Resets all timeout flags and the counter in the specified timer structure.
 *
 * @param timer_ptr Pointer to the `tick_tock` structure to be cleared.
 */
void tick_clear(volatile tick_tock *timer_ptr)
{
    timer_ptr->counter = 0;
    timer_ptr->timeout_0_1s = 0;
    timer_ptr->timeout_0_2s = 0;
    timer_ptr->timeout_0_5s = 0;
    timer_ptr->timeout_1s = 0;
    timer_ptr->timeout_2s = 0;
    timer_ptr->timeout_3s = 0;
    timer_ptr->timeout_4s = 0;
    timer_ptr->timeout_6s = 0;
    timer_ptr->timeout_8s = 0;
    timer_ptr->timeout_10s = 0;
    timer_ptr->timeout_15s = 0;
    timer_ptr->timeout_30s = 0;
    timer_ptr->timeout_1m = 0;
    timer_ptr->timeout_5m = 0;
    timer_ptr->timeout_10m = 0;
}

/**
 * @brief Increments the timer counter and updates timeout flags based on predefined intervals.
 *
 * @param timer_ptr Pointer to the `tick_tock` structure to be updated.
 */
void tick_count(volatile tick_tock *timer_ptr)
{
    timer_ptr->counter++;

    switch (timer_ptr->counter)
    {
        case 200:
            timer_ptr->timeout_0_1s = true;
            break;
        case 400:
            timer_ptr->timeout_0_2s = true;
            break;
        case 1000:
            timer_ptr->timeout_0_5s = true;
            break;
        case 2000:
            timer_ptr->timeout_1s = true;
            break;
        case 4000:
            timer_ptr->timeout_2s = true;
            break;
        case 6000:
            timer_ptr->timeout_3s = true;
            break;
        case 8000:
            timer_ptr->timeout_4s = true;
            break;
        case 12000:
            timer_ptr->timeout_6s = true;
            break;
        case 16000:
            timer_ptr->timeout_8s = true;
            break;
        case 20000:
            timer_ptr->timeout_10s = true;
            break;
        case 30000:
            timer_ptr->timeout_15s = true;
            break;
        case 60000:
            timer_ptr->timeout_30s = true;
            break;
        case 120000:
            timer_ptr->timeout_1m = true;
            break;
        case 600000:
            timer_ptr->timeout_5m = true;
            break;
        case 1200000:
            timer_ptr->timeout_10m = true;
            break;
        default:
            break;
    }
}

/** @brief Timer instance for general timeouts. */
volatile tick_tock timeout;

/** @brief Timer instance for GFI (Ground Fault Interruption) testing. */
volatile tick_tock gfi_test;

/** @brief Timer instance for RTC (Real-Time Clock) check operations. */
volatile tick_tock rtc_checkCounter;

/** @brief Timer instance for RTC updates. */
volatile tick_tock rtc_updateCounter;

/** @brief Timer instance for serial error detection. */
volatile tick_tock serial_error;

/** @brief Timer instance for load balancing every 6 seconds. */
volatile tick_tock load_balance6s;

/** @brief Timer instance for error handling. */
volatile tick_tock error_handler;
