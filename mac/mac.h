/*! ----------------------------------------------------------------------------
 * @file        mac.h
 * @author      Alberto Facheris
 */


#ifndef MAC_H
#define MAC_H


#include <stdint.h>


#define MAC_SUCCESS                         0
#define MAC_ERROR                           -1

#define PAN_COORDINATOR_MAC_ADDR            0x00
#define BROADCAST_MAC_ADDR                  0xFFFFu
#define BROADCAST_PAN_ID                    0xFFFFu
#define MAC_FRAME_CTRL                      0x8144u
#define MAC_HEADER_LEN                      9


typedef struct
{
    uint16_t frame_ctrl;
    uint8_t frame_seq_num;
    uint16_t pan_id;
    uint16_t dest_addr;
    uint16_t src_addr;
}
mac_header_t;


int mac_header_write (mac_header_t* mac_header, uint8_t tx_buf[], uint16_t tx_buf_len);


int mac_header_read (mac_header_t* mac_header, uint8_t rx_buf[], uint16_t rx_buf_len);


#endif