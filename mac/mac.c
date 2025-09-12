/*! ----------------------------------------------------------------------------
 * @file        mac.c
 * @author      Alberto Facheris
 */


#include "mac.h"


void mac_header_write (mac_header_t* mac_header, uint8_t tx_buffer[])
{
    tx_buffer[0] = mac_header->frame_ctrl & 0xFF;
    tx_buffer[1] = (mac_header->frame_ctrl >> 8) & 0xFF;
    tx_buffer[2] = mac_header->frame_seq_num;
    tx_buffer[3] = mac_header->pan_id & 0xFF;
    tx_buffer[4] = (mac_header->pan_id >> 8) & 0xFF;
    tx_buffer[5] = mac_header->dest_addr & 0xFF;
    tx_buffer[6] = (mac_header->dest_addr >> 8) & 0xFF;
    tx_buffer[7] = mac_header->src_addr & 0xFF;
    tx_buffer[8] = (mac_header->src_addr >> 8) & 0xFF;
}


void mac_header_read (mac_header_t* mac_header, uint8_t rx_buffer[])
{
    mac_header->frame_ctrl = ((uint16_t)rx_buffer[0]) | (((uint16_t)rx_buffer[1]) << 8);
    mac_header->frame_seq_num = rx_buffer[2];
    mac_header->pan_id = ((uint16_t)rx_buffer[3]) | (((uint16_t)rx_buffer[4]) << 8);
    mac_header->dest_addr = ((uint16_t)rx_buffer[5]) | (((uint16_t)rx_buffer[6]) << 8);
    mac_header->src_addr = ((uint16_t)rx_buffer[7]) | (((uint16_t)rx_buffer[8]) << 8);
}