/*! ----------------------------------------------------------------------------
 * @file        deca_sleep.c
 * @author      Alberto Facheris
 */


#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include "port.h"


// Sleep for time_ms milliseconds
void deca_sleep (unsigned int sleep_time_ms)
{
    k_msleep(sleep_time_ms);

    return;
}


// Sleep for time_us microseconds
void deca_usleep (unsigned long sleep_time_us)
{
    k_usleep(sleep_time_us);

    return;
}