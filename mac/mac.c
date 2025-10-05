/*! ----------------------------------------------------------------------------
 * @file        mac.c
 * @author      Alberto Facheris
 */


#include <stdio.h>
#include "mac.h"


int mac_header_write (mac_header_t* mac_header, uint8_t tx_buf[], uint16_t tx_buf_len)
{
    // Validate input
    if (mac_header == NULL ||
        tx_buf == NULL ||
        tx_buf_len < MAC_HEADER_LEN)
    {
        return MAC_ERROR;
    }

    // Write MAC frame control
    tx_buf[0] = mac_header->frame_ctrl & 0xFF;
    tx_buf[1] = (mac_header->frame_ctrl >> 8) & 0xFF;

    // Write frame sequence number
    tx_buf[2] = mac_header->frame_seq_num;

    // Write PAN ID
    tx_buf[3] = mac_header->pan_id & 0xFF;
    tx_buf[4] = (mac_header->pan_id >> 8) & 0xFF;

    // Write destination MAC address
    tx_buf[5] = mac_header->dest_addr & 0xFF;
    tx_buf[6] = (mac_header->dest_addr >> 8) & 0xFF;

    // Write source MAC address
    tx_buf[7] = mac_header->src_addr & 0xFF;
    tx_buf[8] = (mac_header->src_addr >> 8) & 0xFF;

    return MAC_SUCCESS;
}


int mac_header_read (mac_header_t* mac_header, uint8_t rx_buf[], uint16_t rx_buf_len)
{
    // Validate input
    if (mac_header == NULL ||
        rx_buf == NULL ||
        rx_buf_len < MAC_HEADER_LEN)
    {
        return MAC_ERROR;
    }

    // Read MAC frame control field
    mac_header->frame_ctrl = ((uint16_t) rx_buf[0]) | (((uint16_t) rx_buf[1]) << 8);

    // Read frame sequence number
    mac_header->frame_seq_num = rx_buf[2];

    // Read PAN ID
    mac_header->pan_id = ((uint16_t) rx_buf[3]) | (((uint16_t) rx_buf[4]) << 8);

    // Read destination MAC address
    mac_header->dest_addr = ((uint16_t) rx_buf[5]) | (((uint16_t) rx_buf[6]) << 8);

    // Read source MAC address
    mac_header->src_addr = ((uint16_t) rx_buf[7]) | (((uint16_t) rx_buf[8]) << 8);

    return MAC_SUCCESS;
}