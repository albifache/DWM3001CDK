/*! ----------------------------------------------------------------------------
 * @file        app_utils.c
 * @author      Alberto Facheris
 */


#include "app_utils.h"
#include "../deca_driver/deca_device_api.h"
#include <stdint.h>


#define RX_TS_LEN                       5
#define TX_TS_LEN                       5
#define TRX_TIME_OFFSET                 8
#define RX_TIMEOUT_OFFSET               16


uint64_t app_read_rx_timestamp (void)
{
    uint64_t rx_stamp = 0ULL;
    uint8_t buf[RX_TS_LEN];

    dwt_readrxtimestamp_sts(buf);

    for (int k = 0; k < RX_TS_LEN; k++)
    {
        rx_stamp |= ((uint64_t) buf[k]) << (8*k);
    }

    return rx_stamp;
}


uint64_t app_read_tx_timestamp (void)
{
    uint64_t tx_stamp = 0ULL;
    uint8_t buf[TX_TS_LEN];

    dwt_readtxtimestamp(buf);

    for (int k = 0; k < TX_TS_LEN; k++)
    {
        tx_stamp |= ((uint64_t) buf[k]) << (8*k);
    }

    return tx_stamp;
}


void app_set_delayed_trx_time (uint64_t start_time)
{
    dwt_setdelayedtrxtime(start_time >> TRX_TIME_OFFSET);

    return;
}


void app_set_rx_timeout (uint64_t rx_timeout)
{
    dwt_setrxtimeout(rx_timeout >> RX_TIMEOUT_OFFSET);
}