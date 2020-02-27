/**
 ****************************************************************************************
 *
 * @file timer.c
 *
 * @brief TIMER driver
 *
 * Copyright (C) RivieraWaves 2009-2015
 *
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup TIMER
 * @{
 ****************************************************************************************
 */
/*
 * INCLUDE FILES
 ****************************************************************************************
 */
#include "timer.h"        // timer definition
#include "reg_timer.h"
#include "ke_event.h"

/*
 * MACRO DEFINITIONS
 ****************************************************************************************
 */

/*
 * FUNCTION DEFINITIONS
 ****************************************************************************************
 */
void timer_init(void)
{
}



void timer_set_timeout(uint32_t to)
{
    tmr_gross_setf(to);
}

uint32_t timer_get_time(void)
{
    return tmr_current_time_get();
}

void timer_enable(bool enable)
{
    tmr_enable_setf(enable);
}


/**
 * used to check if timeout occurs.
 */
void timer_isr(void)
{
    if (tmr_expire_getf())
    {
        // Set kernel event for deferred handling
        ke_event_set(KE_EVENT_KE_TIMER);
    }
}

/// @} TIMER
