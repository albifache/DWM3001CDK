/*! ----------------------------------------------------------------------------
 * @file        phy.h
 * @author      Alberto Facheris
 */


#ifndef PHY_H
#define PHY_H


#include "../deca_driver/deca_device_api.h"


#define PHY_SUCCESS                             0
#define PHY_INIT_ERROR                          -1
#define PHY_CONFIG_ERROR                        -2
#define PHY_CONFIG_WARNING                      1


typedef struct
{
    uint32_t dev_id;
    uint32_t chip_id;
    uint64_t lot_id;
    uint8_t otp_rev;
}
phy_device_info_t;


void phy_get_device_info (phy_device_info_t* device_info);


int phy_device_init (void);


int phy_set_config (dwt_config_t* config);


int phy_set_ant_delay (uint16_t ant_delay);


int phy_set_tx_power (uint32_t tx_power);


#endif