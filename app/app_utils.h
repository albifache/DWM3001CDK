/*! ----------------------------------------------------------------------------
 * @file        app_utils.h
 * @author      Alberto Facheris
 */


#ifndef APP_UTILS_H
#define APP_UTILS_H


#include <stdint.h>


uint64_t app_read_rx_timestamp (void);


uint64_t app_read_tx_timestamp (void);


void app_set_delayed_trx_time (uint64_t start_time);


void app_set_rx_timeout (uint64_t rx_timeout);


#endif