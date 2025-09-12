/*! ----------------------------------------------------------------------------
 * @file        app.h
 * @author      Alberto Facheris
 */


#ifndef APP_H
#define APP_H


#include <stdint.h>
#include "../deca_driver/deca_device_api.h"


#define MAX_NUM_ANCHORS                 8

#define MAC_SUCCESS                     0
#define MAC_HEADER_WARNING              1

#define APP_SUCCESS                     0
#define APP_RX_ERROR                    -1
#define APP_SYS_ERROR                   -2
#define APP_CONFIG_ERROR                -3
#define APP_HEADER_WARNING              2


typedef struct
{
    uint32_t superframe_id;
    uint64_t timestamp;
    uint16_t tag_mac_addr;
    uint8_t num_anchors;
    uint16_t anchor_mac_addr[MAX_NUM_ANCHORS];
    uint64_t dist[MAX_NUM_ANCHORS];
}
app_ranging_info_t;


int app_run_ieee_802_15_4_schedule (void);


int app_set_mac_addr (uint16_t mac_addr);


int app_set_pan_id (uint16_t pan_id);


int app_set_tag_mac_addr (uint16_t mac_addr);


int app_set_anchor_mac_addr (uint16_t mac_addr[], uint8_t cnt);


void app_get_ranging_info (app_ranging_info_t *ranging_info);


void app_sleep (uint16_t time_ms);


#endif