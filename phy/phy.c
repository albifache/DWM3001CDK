/*! ----------------------------------------------------------------------------
 * @file        phy.c
 * @author      Alberto Facheris
 */


#include "phy.h"
#include "../deca_driver/deca_device_api.h"
#include "../port/deca_probe_interface.h"
#include "../port/port.h"
//#include <zephyr/sys/printk.h>


#define DEFAULT_PG_DELAY                        0x34
#define DEFAULT_PG_COUNT                        0x00

#define DEFAULT_TX_ANT_DELAY                    16385u
#define DEFAULT_RX_ANT_DELAY                    16385u

#define DEFAULT_TX_POWER_CH5                    0xFDFDFDFDul
#define DEFAULT_TX_POWER_CH9                    0xFEFEFEFEul


int phy_init (phy_init_obj_t *obj)
{
    int ret;

    // Check if pointer is valid
    if (obj == NULL)
    {
        return PHY_INIT_ERROR;
    }

    // Check if DW3000 IC has been successfully initialized
    bool deca_init_done = deca_init_check();
    if (!deca_init_done)
    {
        return PHY_INIT_ERROR;
    }

    // Read OTP parameters
    deca_hw_info_t info;
    ret = deca_read_device_info(&info);
    if (ret != PORT_SUCCESS)
    {
        return PHY_INIT_ERROR;
    }

    // Set default config values
    dwt_config_t config;
    config.sfdType = DWT_SFD_IEEE_4Z;
    config.phrMode = DWT_PHRMODE_STD;
    config.phrRate = DWT_PHRRATE_STD;
    config.stsMode = DWT_STS_MODE_1;
    config.pdoaMode = DWT_PDOA_M0;

    // Check if channel is valid
    if (obj->rf_chan != 5 && obj->rf_chan != 9)
    {
        return PHY_INIT_ERROR;
    }

    // Set channel
    config.chan = obj->rf_chan;

    // Check if preamble code is valid
    if (obj->preamble_code <= 0 || obj->preamble_code > 24)
    {
        return PHY_INIT_ERROR;
    }

    // Set preamble code
    config.txCode = obj->preamble_code;
    config.rxCode = obj->preamble_code;

    // Compute SFD length
    uint16_t sfd_len = 8;
    if (config.sfdType == DWT_SFD_DW_16 || config.sfdType == DWT_SFD_LEN16)
    {
        sfd_len = 16;
    }

    // Compute preamble length and PAC size
    uint16_t tx_pr_len = 32;
    uint16_t pac_size = 4;
    switch (obj->preamble_len)
    {
        case DWT_PLEN_32:
            tx_pr_len = 32;
            pac_size = 4;
            config.rxPAC = DWT_PAC4;
        break;

        case DWT_PLEN_64:
            tx_pr_len = 64;
            pac_size = 4;
            config.rxPAC = DWT_PAC4;
        break;

        case DWT_PLEN_72:
            tx_pr_len = 72;
            pac_size = 4;
            config.rxPAC = DWT_PAC4;
        break;

        case DWT_PLEN_128:
            tx_pr_len = 128;
            pac_size = 8;
            config.rxPAC = DWT_PAC8;
        break;

        case DWT_PLEN_256:
            tx_pr_len = 256;
            pac_size = 16;
            config.rxPAC = DWT_PAC16;
        break;

        case DWT_PLEN_512:
            tx_pr_len = 512;
            pac_size = 32;
            config.rxPAC = DWT_PAC32;
        break;

        case DWT_PLEN_1024:
            tx_pr_len = 1024;
            pac_size = 32;
            config.rxPAC = DWT_PAC32;
        break;

        case DWT_PLEN_1536:
            tx_pr_len = 1536;
            pac_size = 32;
            config.rxPAC = DWT_PAC32;
        break;

        case DWT_PLEN_2048:
            tx_pr_len = 2048;
            pac_size = 32;
            config.rxPAC = DWT_PAC32;
        break;

        case DWT_PLEN_4096:
            tx_pr_len = 4096;
            pac_size = 32;
            config.rxPAC = DWT_PAC32;
        break;

        default:
            return PHY_INIT_ERROR;
        break;
    }

    // Set preamble length
    config.txPreambLength = obj->preamble_len;

    // Set SFD timeout
    config.sfdTO = tx_pr_len + sfd_len + 1 - pac_size;

    // Check if bit rate is valid
    if (obj->bit_rate != DWT_BR_850K && obj->bit_rate != DWT_BR_6M8)
    {
        return PHY_INIT_ERROR;
    }

    // Set bit rate
    config.dataRate = obj->bit_rate;

    // Check if STS length is valid
    if (obj->sts_len != DWT_STS_LEN_32 &&
        obj->sts_len != DWT_STS_LEN_64 &&
        obj->sts_len != DWT_STS_LEN_128 &&
        obj->sts_len != DWT_STS_LEN_256 &&
        obj->sts_len != DWT_STS_LEN_512 &&
        obj->sts_len != DWT_STS_LEN_1024 &&
        obj->sts_len != DWT_STS_LEN_2048)
    {
        return PHY_INIT_ERROR;
    }

    // Set STS length
    config.stsLength = obj->sts_len;

    // Configure DW3000 IC, if either the PLL or RX calibration has failed the host should reset the device
    ret = dwt_configure(&config);
    if (ret != DWT_SUCCESS)
    {
        return PHY_INIT_ERROR;
    }

    // Disable timeouts
    dwt_setrxtimeout(0);
    dwt_setpreambledetecttimeout(0);

    // Read TX power from OTP
    uint32_t tx_power = DEFAULT_TX_POWER_CH5;
    if (obj->rf_chan == 5)
    {
        if (obj->preamble_code >= 9 && obj->preamble_code <= 24)
        {
            tx_power = info.tx_power_ch5_prf64;
        }
        else if (obj->preamble_code >= 1 && obj->preamble_code <= 8)
        {
            tx_power = info.tx_power_ch5_prf16;
        }
    }
    else if (obj->rf_chan == 9)
    {
        if (obj->preamble_code >= 9 && obj->preamble_code <= 24)
        {
            tx_power = info.tx_power_ch9_prf64;
        }
        else if (obj->preamble_code >= 1 && obj->preamble_code <= 8)
        {
            tx_power = info.tx_power_ch9_prf16;
        }
    }

    // Check if TX power was precalibrated
    if (tx_power == 0)
    {
        return PHY_INIT_ERROR;
    }

    // Set TX parameters (power, PG delay and PG count)
    dwt_txconfig_t txconfig;
    txconfig.PGdly = DEFAULT_PG_DELAY;
    txconfig.power = tx_power;
    txconfig.PGcount = DEFAULT_PG_COUNT;
    dwt_configuretxrf(&txconfig);

    // Read TX antenna delay from OTP
    uint16_t tx_ant_delay = DEFAULT_TX_ANT_DELAY;
    if (obj->rf_chan == 5)
    {
        if (obj->preamble_code >= 9 && obj->preamble_code <= 24)
        {
            tx_ant_delay = info.tx_antd_ch5_prf64;
        }
        else if (obj->preamble_code >= 1 && obj->preamble_code <= 8)
        {
            tx_ant_delay = info.tx_antd_ch5_prf16;
        }
    }
    else if (obj->rf_chan == 9)
    {
        if (obj->preamble_code >= 9 && obj->preamble_code <= 24)
        {
            tx_ant_delay = info.tx_antd_ch9_prf64;
        }
        else if (obj->preamble_code >= 1 && obj->preamble_code <= 8)
        {
            tx_ant_delay = info.tx_antd_ch9_prf16;
        }
    }

    // Read RX antenna delay from OTP
    uint16_t rx_ant_delay = DEFAULT_RX_ANT_DELAY;
    if (obj->rf_chan == 5)
    {
        if (obj->preamble_code >= 9 && obj->preamble_code <= 24)
        {
            rx_ant_delay = info.rx_antd_ch5_prf64;
        }
        else if (obj->preamble_code >= 1 && obj->preamble_code <= 8)
        {
            rx_ant_delay = info.rx_antd_ch5_prf16;
        }
    }
    else if (obj->rf_chan == 9)
    {
        if (obj->preamble_code >= 9 && obj->preamble_code <= 24)
        {
            rx_ant_delay = info.rx_antd_ch9_prf64;
        }
        else if (obj->preamble_code >= 1 && obj->preamble_code <= 8)
        {
            rx_ant_delay = info.rx_antd_ch9_prf16;
        }
    }

    // Check if antenna delay was precalibrated
    if (rx_ant_delay == 0 && tx_ant_delay == 0)
    {
        return PHY_INIT_ERROR;
    }

    // Set antenna delay
    dwt_setrxantennadelay(rx_ant_delay + tx_ant_delay);
    dwt_settxantennadelay(0);

    return PHY_SUCCESS;
}