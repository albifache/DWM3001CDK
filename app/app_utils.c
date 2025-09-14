/*! ----------------------------------------------------------------------------
 * @file        app_utils.c
 * @author      Alberto Facheris
 */


#include "app_utils.h"
#include "../deca_driver/deca_device_api.h"
#include <stdint.h>


#define JUNK                                0
#define NULL_RX_BUFFER_OFFSET               0                                       
#define NULL_TX_BUFFER_OFFSET               0
#define RANGING_BIT_ENABLED                 1
#define RX_TIMEOUT_OFFSET                   16


uint64_t app_read_rx_timestamp (void)
{
    uint64_t rx_stamp = 0ULL;
    uint8_t buf[5];

    dwt_readrxtimestamp(buf, DWT_COMPAT_NONE);

    for (int k = 0; k < 5; k++)
    {
        rx_stamp |= ((uint64_t) buf[k]) << (8*k);
    }

    return rx_stamp;
}


uint64_t app_read_tx_timestamp (void)
{
    uint64_t tx_stamp = 0ULL;
    uint8_t buf[5];

    dwt_readtxtimestamp(buf);

    for (int k = 0; k < 5; k++)
    {
        tx_stamp |= ((uint64_t) buf[k]) << (8*k);
    }

    return tx_stamp;
}


void app_set_delayed_trx_time (uint64_t start_time)
{
    dwt_setdelayedtrxtime(start_time >> 8);

    return;
}


void app_write_tx_frame_len (uint16_t tx_frame_len)
{
    dwt_writetxfctrl(tx_frame_len, NULL_TX_BUFFER_OFFSET, RANGING_BIT_ENABLED);

    return;
}


void app_write_tx_buffer (uint8_t tx_buffer[], uint16_t tx_frame_len)
{
    dwt_writetxdata(tx_frame_len - FCS_LEN, tx_buffer, NULL_TX_BUFFER_OFFSET);

    return;
}


uint16_t app_read_rx_frame_len (void)
{
    return dwt_getframelength(JUNK);
}


void app_read_rx_buffer (uint8_t rx_buffer[], uint16_t rx_frame_len)
{
    dwt_readrxdata(rx_buffer, rx_frame_len, NULL_RX_BUFFER_OFFSET);

    return;
}


void app_set_rx_timeout (uint64_t rx_timeout)
{
    dwt_setrxtimeout(rx_timeout >> RX_TIMEOUT_OFFSET);
}