/*! ----------------------------------------------------------------------------
 * @file        phy.h
 * @author      Alberto Facheris
 */


#ifndef PHY_H
#define PHY_H


#include "../deca_driver/deca_device_api.h"


#define PHY_SUCCESS                             0
#define PHY_INIT_ERROR                          -1
#define PHY_INIT_WARNING                        1


typedef struct
{
    dwt_pll_ch_type_e rf_chan;
    uint8_t preamble_code;
    uint16_t preamble_len;
    dwt_uwb_bit_rate_e bit_rate;
    dwt_sts_lengths_e sts_len;
}
phy_init_obj_t;


int phy_init (phy_init_obj_t *obj);


#endif