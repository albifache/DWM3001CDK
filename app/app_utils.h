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


void app_write_tx_frame_len (uint16_t tx_frame_len);


void app_write_tx_buffer (uint8_t tx_buffer[], uint16_t tx_frame_len);


uint16_t app_read_rx_frame_len (void);


void app_read_rx_buffer (uint8_t rx_buffer[], uint16_t rx_frame_len);


#endif