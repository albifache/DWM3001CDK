/*! ----------------------------------------------------------------------------
 * @file        app.h
 * @author      Alberto Facheris
 */


#ifndef APP_H
#define APP_H


#include <stdint.h>
#include "../deca_driver/deca_device_api.h"


#define MAX_NUM_ANCHORS                 8

#define APP_SUCCESS                     0
#define APP_INIT_ERROR                  -1
#define APP_RUN_ERROR                   -2
#define APP_RUN_WARNING                 1


typedef struct
{
    uint16_t mac_addr;
    uint16_t pan_id;
    dwt_aes_key_t aes_key;
    dwt_sts_cp_key_t sts_key;
}
app_init_obj_t;


typedef struct
{
    uint16_t tag_mac_addr;
    uint16_t anchor_mac_addr[MAX_NUM_ANCHORS];
    uint8_t num_anchors;
}
app_ctrl_obj_t;


typedef struct
{
    uint64_t superframe_id;
    uint64_t ts_init;
    uint16_t tag_mac_addr;
    uint8_t num_anchors;
    uint16_t anchor_mac_addr[MAX_NUM_ANCHORS];
    uint64_t dist[MAX_NUM_ANCHORS];
}
app_log_info_t;


int app_init (app_init_obj_t *obj);


int app_set_ctrl_params (app_ctrl_obj_t *obj);


int app_run_ieee_802_15_4z_schedule (void);


int app_read_log_info (app_log_info_t *info);


void app_sleep (uint16_t ms);


#endif