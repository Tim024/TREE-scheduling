/*
 * This file is part of HiKoB Openlab.
 *
 * HiKoB Openlab is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License
 * as published by the Free Software Foundation, version 3.
 *
 * HiKoB Openlab is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with HiKoB Openlab. If not, see
 * <http://www.gnu.org/licenses/>.
 *
 * Copyright (C) 2012 HiKoB.
 */

/*
 * soft_timer_delay.c
 *
 *  Created on: Jan 13, 2012
 *      Author: Clément Burin des Roziers <clement.burin-des-roziers.at.hikob.com>
 */

#if !defined(PLATFORM_OS) || (PLATFORM_OS == FREERTOS)
#include "FreeRTOS.h"
#else
static void vPortEnterCritical() {}
static void vPortExitCritical() {}
#endif

#include "soft_timer_.h"
#include "timer.h"

#define LOG_LEVEL LOG_LEVEL_INFO
#include "debug.h"

softtim_t softtim;

void soft_timer_config(openlab_timer_t timer, timer_channel_t channel)
{
    // Store parameters
    softtim.mutex = NULL;
    softtim.priority = EVENT_QUEUE_APPLI;
    softtim.timer = timer;
    softtim.channel = channel;
    softtim.first = NULL;
    softtim.remainder = 0;
    softtim.update_count = 0;
}

struct _soft_timer_time {
    uint32_t update_count;
    uint16_t time16;
};


static struct _soft_timer_time _soft_timer_time()
{
    /*
     * To prevent having the remainder being incremented while computing,
     * do the following:
     *
     *   * disable the interrupts
     *   * get the time (A)
     *   * check if an update has happened
     *   * get the time again (B)
     *   * enable the interrupts
     *
     *   * an update was missed
     *      * if the update flag is set or
     *      * if B is lower than A
     *
     *
     */
    uint32_t update_count = 0;
    uint16_t t_a, t_b;
    uint32_t update = 0;

    // Mask interrupts
    vPortEnterCritical();

    update_count = softtim.update_count;
    t_a = timer_time(softtim.timer);
    update = timer_get_update_flag(softtim.timer);
    t_b = timer_time(softtim.timer);

    // Unmask interrupts
    vPortExitCritical();

    // Add one if missed update
    update_count += ((update || (t_b < t_a)) ? 1 : 0);

    return (struct _soft_timer_time){update_count, t_b};
}

uint32_t soft_timer_time()
{
    struct _soft_timer_time t = _soft_timer_time();
    return (t.update_count << 16) + t.time16;
}

uint64_t soft_timer_time_64()
{
    struct _soft_timer_time t = _soft_timer_time();
    return (((uint64_t)t.update_count) << 16) + t.time16;
}

struct soft_timer_timeval soft_timer_time_extended()
{
    struct soft_timer_timeval tv;
    struct _soft_timer_time t = _soft_timer_time();

    tv.tv_sec = t.update_count << 1;
    tv.tv_sec += (t.time16 & 0x8000) ? 1 : 0;
    tv.tv_usec = soft_timer_ticks_to_us(t.time16 & 0x7FFF);

    return tv;
}


uint32_t soft_timer_convert_time(uint16_t t)
{
    // We assume t cannot be in the future
    uint32_t ref = soft_timer_time();
    uint16_t n = ref & 0xFFFF;
    uint16_t d = n - t;

    if (n >= t)
    {
        if (d < 32768)
        {
            // Nothing to do
        }
        else
        {
            // t is in advance
            ref += 0x10000;
        }
    }
    else
    {
        if (d < 32768)
        {
            // Adjust
            ref -= 0x10000;
        }
        else
        {
            // t is in advance
        }
    }

    return (ref & 0xFFFF0000) | t;
}

int32_t soft_timer_a_is_before_b(uint32_t a, uint32_t b)
{
    int32_t delta = b - a;
    return delta > 0;
}

void soft_timer_delay(uint32_t d)
{
    uint32_t t_end = soft_timer_time() + d;

    // Loop until delay elapsed
    while (soft_timer_a_is_before_b(soft_timer_time(), t_end))
    {
    }
}

void soft_timer_delay_us(uint32_t us)
{
    soft_timer_delay(soft_timer_us_to_ticks(us));
}

void soft_timer_delay_ms(uint32_t ms)
{
    soft_timer_delay(soft_timer_ms_to_ticks(ms));
}

void soft_timer_delay_s(uint32_t s)
{
    soft_timer_delay(soft_timer_s_to_ticks(s));
}
