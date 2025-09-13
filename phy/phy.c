/*! ----------------------------------------------------------------------------
 * @file        phy.c
 * @author      Alberto Facheris
 */


#include "phy.h"
#include "../deca_driver/deca_device_api.h"
#include "../port/deca_probe_interface.h"
#include "../port/port.h"


#define OTP_ADDR_CHIP_ID                        0x06u
#define OTP_ADDR_LOT_ID_LO                      0x0Du
#define OTP_ADDR_LOT_ID_HI                      0x0Eu

#define OTP_ADDR_TX_POWER_CH5_PRF16             0x10u
#define OTP_ADDR_TX_POWER_CH5_PRF64             0x11u
#define OTP_ADDR_TX_POWER_CH9_PRF16             0x12u
#define OTP_ADDR_TX_POWER_CH9_PRF64             0x13u 

#define OTP_ADDR_TX_ANT_DELAY_CH5_PRF16         0x1Bu
#define OTP_ADDR_TX_ANT_DELAY_CH5_PRF64         0x1Au
#define OTP_ADDR_TX_ANT_DELAY_CH9_PRF16         0x1Du
#define OTP_ADDR_TX_ANT_DELAY_CH9_PRF64         0x1Cu

#define OTP_ADDR_RX_ANT_DELAY_CH5_PRF16         0x1Bu
#define OTP_ADDR_RX_ANT_DELAY_CH5_PRF64         0x1Au
#define OTP_ADDR_RX_ANT_DELAY_CH9_PRF16         0x1Du
#define OTP_ADDR_RX_ANT_DELAY_CH9_PRF64         0x1Cu

#define TX_ANT_DELAY_MASK                       0xFFFFul
#define RX_ANT_DELAY_MASK                       0xFFFF0000ul
#define TX_ANT_DELAY_OFFSET                     0
#define RX_ANT_DELAY_OFFSET                     16

#define DEFAULT_PG_DELAY                        0x34
#define DEFAULT_PG_COUNT                        0x00

#define DEFAULT_TX_ANT_DELAY                    16385u
#define DEFAULT_RX_ANT_DELAY                    16385u

#define DEFAULT_TX_POWER_CH5                    0xFDFDFDFDul
#define DEFAULT_TX_POWER_CH9                    0xFEFEFEFEul


static uint32_t device_id = 0;

static uint32_t chip_id = 0;

static uint32_t lot_id_lo = 0;
static uint32_t lot_id_hi = 0;
static uint64_t lot_id = 0;

static uint32_t tx_ant_delay = DEFAULT_TX_ANT_DELAY;
static uint32_t rx_ant_delay = DEFAULT_RX_ANT_DELAY;


static dwt_config_t config =
{
    .chan = 5,
    .txPreambLength = DWT_PLEN_1024,
    .rxPAC = DWT_PAC16,
    .txCode = 9,
    .rxCode = 9,
    .sfdType = DWT_SFD_IEEE_4A,
    .dataRate = DWT_BR_850K,
    .phrMode = DWT_PHRMODE_STD,
    .phrRate = DWT_PHRRATE_STD,
    .sfdTO = 1025,
    .stsMode = DWT_STS_MODE_OFF,
    .stsLength = DWT_STS_LEN_64,
    .pdoaMode = DWT_PDOA_M0
};


static dwt_txconfig_t txconfig =
{
    .PGdly = DEFAULT_PG_DELAY,
    .power = DEFAULT_TX_POWER_CH5,
    .PGcount = DEFAULT_PG_COUNT
};


static uint32_t phy_read_tx_power_from_otp (void);
static uint16_t phy_read_tx_ant_delay_from_otp (void);
static uint16_t phy_read_rx_ant_delay_from_otp (void);


static uint32_t phy_read_tx_power_from_otp (void)
{
    uint32_t buf[1];
    uint32_t tx_power_from_otp;

    // Channel 5
    if (config.chan == 5)
    {
        // PRF 64 MHz
        if (config.txCode >= 9 && config.txCode <= 24)
        {
            dwt_otpread(OTP_ADDR_TX_POWER_CH5_PRF64, buf, 1);
        }

        // PRF 16 MHz
        else if (config.txCode >= 1 && config.txCode <= 8)
        {
            dwt_otpread(OTP_ADDR_TX_POWER_CH5_PRF16, buf, 1);
        }
    }
    
    // Channel 9
    if (config.chan == 9)
    {
        // PRF 64 MHz
        if (config.txCode >= 9 && config.txCode <= 24)
        {
            dwt_otpread(OTP_ADDR_TX_POWER_CH9_PRF64, buf, 1);
        }

        // PRF 16 MHz
        else if (config.txCode >= 1 && config.txCode <= 8)
        {
            dwt_otpread(OTP_ADDR_TX_POWER_CH9_PRF16, buf, 1);
        }
    }

    // Copy TX power from OTP
    tx_power_from_otp = buf[0];

    return tx_power_from_otp;
}


static uint16_t phy_read_tx_ant_delay_from_otp (void)
{
    uint32_t buf[1];
    uint32_t tx_ant_delay_from_otp;

    // Channel 5
    if (config.chan == 5)
    {
        // PRF 64 MHz
        if (config.txCode >= 9 && config.txCode <= 24)
        {
            dwt_otpread(OTP_ADDR_TX_ANT_DELAY_CH5_PRF64, buf, 1);
        }

        // PRF 16 MHz
        else if (config.txCode >= 1 && config.txCode <= 8)
        {
            dwt_otpread(OTP_ADDR_TX_ANT_DELAY_CH5_PRF16, buf, 1);
        }
    }

    // Channel 9
    else if (config.chan == 9)
    {
        // PRF 64 MHz
        if (config.txCode >= 9 && config.txCode <= 24)
        {
            dwt_otpread(OTP_ADDR_TX_ANT_DELAY_CH9_PRF64, buf, 1);
        }

        // PRF 16 MHz
        else if (config.txCode >= 1 && config.txCode <= 8)
        {
            dwt_otpread(OTP_ADDR_TX_ANT_DELAY_CH9_PRF16, buf, 1);
        }
    }

    // Copy TX antenna delay from OTP
    tx_ant_delay_from_otp = buf[0] & TX_ANT_DELAY_MASK;
    tx_ant_delay_from_otp >>= TX_ANT_DELAY_OFFSET;

    return tx_ant_delay_from_otp;
}


static uint16_t phy_read_rx_ant_delay_from_otp (void)
{
    uint32_t buf[1];
    uint32_t rx_ant_delay_from_otp;

    // Channel 5
    if (config.chan == 5)
    {
        // PRF 64 MHz
        if (config.txCode >= 9 && config.txCode <= 24)
        {
            dwt_otpread(OTP_ADDR_RX_ANT_DELAY_CH5_PRF64, buf, 1);
        }

        // PRF 16 MHz
        else if (config.txCode >= 1 && config.txCode <= 8)
        {
            dwt_otpread(OTP_ADDR_RX_ANT_DELAY_CH5_PRF16, buf, 1);
        }
    }

    // Channel 9
    else if (config.chan == 9)
    {
        // PRF 64 MHz
        if (config.txCode >= 9 && config.txCode <= 24)
        {
            dwt_otpread(OTP_ADDR_RX_ANT_DELAY_CH9_PRF64, buf, 1);
        }

        // PRF 16 MHz
        else if (config.chan == 9 && config.txCode >= 1 && config.txCode <= 8)
        {
            dwt_otpread(OTP_ADDR_RX_ANT_DELAY_CH9_PRF16, buf, 1);
        }
    }

    // Copy RX antenna delay from OTP
    rx_ant_delay_from_otp = buf[0] & RX_ANT_DELAY_MASK;
    rx_ant_delay_from_otp >>= RX_ANT_DELAY_OFFSET;

    return rx_ant_delay_from_otp;
}


int phy_device_init (void)
{
    // Initialize the peripherals
    deca_gpio_init();
    deca_spi_init();

    // Reset DW3000 IC
    deca_reset_ic();

    // Probe for the correct device driver
    dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf);

    // Make sure DW3000 IC is in IDLE_RC before proceeding
    while (!dwt_checkidlerc());

    // Initialise DW3000 IC
    int ret = dwt_initialise(DWT_DW_INIT);
    if (ret != DWT_SUCCESS)
    {
        return PHY_INIT_ERROR;
    }

    // Read device ID
    device_id = dwt_readdevid();

    // Check if device ID is valid
    if (device_id != DWT_DW3000_DEV_ID &&
        device_id != DWT_QM33110_DEV_ID &&
        device_id != DWT_DW3000_PDOA_DEV_ID &&
        device_id!= DWT_QM33120_PDOA_DEV_ID)
    {
        return PHY_INIT_ERROR;
    }

    uint32_t buf[1];

    // Read chip ID (manually cause Qorvo API is bugged here)
    dwt_otpread(OTP_ADDR_CHIP_ID, buf, 1);
    chip_id = buf[0];

    // Read lot ID (low order bits)
    dwt_otpread(OTP_ADDR_LOT_ID_LO, buf, 1);
    lot_id_lo = buf[0];

    // Read lot ID (high order bits)
    dwt_otpread(OTP_ADDR_LOT_ID_HI, buf, 1);
    lot_id_hi = buf[0];

    // Compute lot ID
    lot_id = ((uint64_t) lot_id_lo) | (((uint64_t) lot_id_hi) << 32);

    return PHY_SUCCESS;    
}


int phy_set_config (dwt_config_t* new_config)
{
    // Update configuration
    config = *new_config;

    // Compute SFD length
    uint16_t sfd_len = 8;
    if (config.sfdType == DWT_SFD_DW_16 || config.sfdType == DWT_SFD_LEN16)
    {
        sfd_len = 16;
    }

    // Set preamble length and PAC size
    uint16_t tx_pr_len = 4096;
    uint16_t pac_size = 4;
    switch (config.txPreambLength)
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
    }

    // Set SFD timeout
    config.sfdTO = tx_pr_len + sfd_len + 1 - pac_size;

    // Configure DW3000 IC, if either the PLL or RX calibration has failed the host should reset the device
    int ret = dwt_configure(&config);
    if (ret != DWT_SUCCESS)
    {
        return PHY_CONFIG_ERROR;
    }

    // Disable timeouts
    dwt_setrxtimeout(0);
    dwt_setpreambledetecttimeout(0);

    return PHY_SUCCESS;
}


int phy_set_tx_power (uint32_t tx_power)
{
    int ret = PHY_SUCCESS;

    // Automatically select TX power
    if (tx_power == 0)
    {
        uint32_t tx_power_from_otp = phy_read_tx_power_from_otp();

        // Check if TX power is precalibrated
        if (tx_power_from_otp != 0)
        {
            tx_power = tx_power_from_otp;
            ret = PHY_SUCCESS;
        }

        // Set default TX power if not precalibrated
        else
        {
            // Channel 5
            if (config.chan == 5)
            {
                tx_power = DEFAULT_TX_POWER_CH5;
            }

            // Channel 9
            else if (config.chan == 9)
            {
                tx_power = DEFAULT_TX_POWER_CH9;
            }

            ret = PHY_CONFIG_WARNING;
        }
    }

    // Configure the TX parameters (power, PG delay and PG count)
    txconfig.PGdly = DEFAULT_PG_DELAY;
    txconfig.power = tx_power;
    txconfig.PGcount = DEFAULT_PG_COUNT;
    dwt_configuretxrf(&txconfig);

    return ret;
}


int phy_set_ant_delay (uint16_t ant_delay)
{
    int ret = PHY_SUCCESS;

    // Manually select antenna delay
    rx_ant_delay = ant_delay;
    tx_ant_delay = ant_delay;

    // Automatically select antenna delay
    if (ant_delay == 0)
    {
        uint16_t rx_ant_delay_from_otp = phy_read_rx_ant_delay_from_otp();
        uint16_t tx_ant_delay_from_otp = phy_read_tx_ant_delay_from_otp();

        // Check if antenna delay is precalibrated
        if (rx_ant_delay_from_otp != 0 || tx_ant_delay_from_otp != 0)
        {
            rx_ant_delay = rx_ant_delay_from_otp;
            tx_ant_delay = tx_ant_delay_from_otp;
            ret = PHY_SUCCESS;
        }

        // Set default antenna delay if not precalibrated
        else
        {
            rx_ant_delay = DEFAULT_RX_ANT_DELAY;
            tx_ant_delay = DEFAULT_TX_ANT_DELAY;
            ret = PHY_CONFIG_WARNING;
        }   
    }

    // Set antenna delay
    dwt_setrxantennadelay(rx_ant_delay + tx_ant_delay);
    dwt_settxantennadelay(0);

    return ret;
}


void phy_get_device_info (phy_device_info_t* device_info)
{
    device_info->dev_id = device_id;
    device_info->chip_id = chip_id;
    device_info->lot_id = lot_id;

    return;
}