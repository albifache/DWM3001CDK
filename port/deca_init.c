/*! ----------------------------------------------------------------------------
 * @file        deca_port.c
 * @author      Alberto Facheris
 */


#include "../deca_driver/deca_device_api.h"
#include "deca_probe_interface.h"
#include "port.h"


#define OTP_ADDR_CHIP_ID                        0x06

#define OTP_ADDR_LOT_ID_LO                      0x0D
#define OTP_ADDR_LOT_ID_HI                      0x0E

#define OTP_ADDR_OTP_REV                        0x1F

#define OTP_ADDR_XTAL_TRIM                      0x1E

#define OTP_ADDR_TX_POWER_CH5_PRF16             0x10
#define OTP_ADDR_TX_POWER_CH5_PRF64             0x11
#define OTP_ADDR_TX_POWER_CH9_PRF16             0x12
#define OTP_ADDR_TX_POWER_CH9_PRF64             0x13 

#define OTP_ADDR_TX_ANT_DELAY_CH5_PRF16         0x1B
#define OTP_ADDR_TX_ANT_DELAY_CH5_PRF64         0x1A
#define OTP_ADDR_TX_ANT_DELAY_CH9_PRF16         0x1D
#define OTP_ADDR_TX_ANT_DELAY_CH9_PRF64         0x1C

#define OTP_ADDR_RX_ANT_DELAY_CH5_PRF16         0x1B
#define OTP_ADDR_RX_ANT_DELAY_CH5_PRF64         0x1A
#define OTP_ADDR_RX_ANT_DELAY_CH9_PRF16         0x1D
#define OTP_ADDR_RX_ANT_DELAY_CH9_PRF64         0x1C

#define OTP_REV_MASK                            0xFF

#define XTAL_TRIM_MASK                          0x7F

#define TX_ANT_DELAY_MASK                       0xFFFFu
#define RX_ANT_DELAY_MASK                       0xFFFFu

#define TX_ANT_DELAY_OFFSET                     0
#define RX_ANT_DELAY_OFFSET                     16


static bool deca_init_done = false;
static deca_hw_info_t deca_hw_info;


int deca_init (void)
{
    int ret;

    // Initialize GPIO
    ret = deca_gpio_init();
    if (ret != PORT_SUCCESS)
    {
        return PORT_INIT_ERROR;
    }

    // Initialize SPI
    ret = deca_spi_init();
    if (ret != PORT_SUCCESS)
    {
        return PORT_INIT_ERROR;
    }

    // Reset DW3000 IC
    ret = deca_reset_ic();
    if (ret != PORT_SUCCESS)
    {
        return PORT_INIT_ERROR;
    }

    // Probe for the correct device driver
    dwt_probe((struct dwt_probe_s *)&dw3000_probe_interf);

    // Wait until DW3000 IC is in IDLE_RC state before proceeding
    while (!dwt_checkidlerc());

    // Initialise DW3000 IC
    ret = dwt_initialise(DWT_DW_INIT);
    if (ret != DWT_SUCCESS)
    {
        return PORT_INIT_ERROR;
    }

    // Read device ID
    uint32_t device_id = dwt_readdevid();

    // Check if device ID is valid
    if (device_id != DWT_DW3000_DEV_ID &&
        device_id != DWT_QM33110_DEV_ID &&
        device_id != DWT_DW3000_PDOA_DEV_ID &&
        device_id != DWT_QM33120_PDOA_DEV_ID)
    {
        return PORT_INIT_ERROR;
    }

    // Set initialization flag
    deca_init_done = true;

    // Read device ID
    deca_hw_info.device_id = device_id;

    // Read chip ID
    uint32_t chip_id;
    dwt_otpread(OTP_ADDR_CHIP_ID, &chip_id, 1);
    deca_hw_info.chip_id = chip_id;

    // Read lot ID
    uint32_t lot_id_lo;
    uint32_t lot_id_hi;
    dwt_otpread(OTP_ADDR_LOT_ID_LO, &lot_id_lo, 1);
    dwt_otpread(OTP_ADDR_LOT_ID_HI, &lot_id_hi, 1);
    deca_hw_info.lot_id = (((uint64_t) lot_id_hi) << 32) | ((uint64_t) lot_id_lo);

    // Read OTP revision
    uint32_t otp_rev;
    dwt_otpread(OTP_ADDR_OTP_REV, &otp_rev, 1);
    deca_hw_info.otp_rev = otp_rev & OTP_REV_MASK;
    
    // Read crystal trim
    uint32_t xtal_trim;
    dwt_otpread(OTP_ADDR_XTAL_TRIM, &xtal_trim, 1);
    deca_hw_info.xtal_trim = xtal_trim & XTAL_TRIM_MASK;

    // Read TX power for channel 5 and PRF 64 MHz
    uint32_t tx_power;
    dwt_otpread(OTP_ADDR_TX_POWER_CH5_PRF64, &tx_power, 1);
    deca_hw_info.tx_power_ch5_prf64 = tx_power;

    // Read TX power for channel 5 and PRF 16 MHz
    dwt_otpread(OTP_ADDR_TX_POWER_CH5_PRF16, &tx_power, 1);
    deca_hw_info.tx_power_ch5_prf16 = tx_power;

    // Read TX power for channel 9 and PRF 64 MHz
    dwt_otpread(OTP_ADDR_TX_POWER_CH9_PRF64, &tx_power, 1);
    deca_hw_info.tx_power_ch9_prf64 = tx_power;

    // Read TX power for channel 9 and PRF 16 MHz
    dwt_otpread(OTP_ADDR_TX_POWER_CH9_PRF16, &tx_power, 1);
    deca_hw_info.tx_power_ch9_prf16 = tx_power;

    // Read TX antenna delay for channel 5 and PRF 64 MHz
    uint32_t ant_delay;
    dwt_otpread(OTP_ADDR_TX_ANT_DELAY_CH5_PRF64, &ant_delay, 1);
    deca_hw_info.tx_antd_ch5_prf64 = (ant_delay >> TX_ANT_DELAY_OFFSET) & TX_ANT_DELAY_MASK;
    
    // Read TX antenna delay for channel 5 and PRF 16 MHz
    dwt_otpread(OTP_ADDR_TX_ANT_DELAY_CH5_PRF16, &ant_delay, 1);
    deca_hw_info.tx_antd_ch5_prf16 = (ant_delay >> TX_ANT_DELAY_OFFSET) & TX_ANT_DELAY_MASK;

    // Read TX antenna delay for channel 9 and PRF 64 MHz
    dwt_otpread(OTP_ADDR_TX_ANT_DELAY_CH9_PRF64, &ant_delay, 1);
    deca_hw_info.tx_antd_ch9_prf64 = (ant_delay >> TX_ANT_DELAY_OFFSET) & TX_ANT_DELAY_MASK;
    
    // Read TX antenna delay for channel 9 and PRF 16 MHz
    dwt_otpread(OTP_ADDR_TX_ANT_DELAY_CH9_PRF16, &ant_delay, 1);
    deca_hw_info.tx_antd_ch9_prf16 = (ant_delay >> TX_ANT_DELAY_OFFSET) & TX_ANT_DELAY_MASK;

    // Read RX antenna delay for channel 5 and PRF 64 MHz
    dwt_otpread(OTP_ADDR_RX_ANT_DELAY_CH5_PRF64, &ant_delay, 1);
    deca_hw_info.rx_antd_ch5_prf64 = (ant_delay >> RX_ANT_DELAY_OFFSET) & RX_ANT_DELAY_MASK;
    
    // Read RX antenna delay for channel 5 and PRF 16 MHz
    dwt_otpread(OTP_ADDR_RX_ANT_DELAY_CH5_PRF16, &ant_delay, 1);
    deca_hw_info.rx_antd_ch5_prf16 = (ant_delay >> RX_ANT_DELAY_OFFSET) & RX_ANT_DELAY_MASK;

    // Read RX antenna delay for channel 9 and PRF 64 MHz
    dwt_otpread(OTP_ADDR_RX_ANT_DELAY_CH9_PRF64, &ant_delay, 1);
    deca_hw_info.rx_antd_ch9_prf64 = (ant_delay >> RX_ANT_DELAY_OFFSET) & RX_ANT_DELAY_MASK;
    
    // Read RX antenna delay for channel 9 and PRF 16 MHz
    dwt_otpread(OTP_ADDR_RX_ANT_DELAY_CH9_PRF16, &ant_delay, 1);
    deca_hw_info.rx_antd_ch9_prf16 = (ant_delay >> RX_ANT_DELAY_OFFSET) & RX_ANT_DELAY_MASK;

    return PORT_SUCCESS;
}


int deca_read_device_info (deca_hw_info_t *info)
{
    // Check if pointer is valid
    if (info == NULL)
    {
        return PORT_RUN_ERROR;
    }

    // Read DW3000 IC info
    info->device_id = deca_hw_info.device_id;
    info->chip_id = deca_hw_info.chip_id;
    info->lot_id = deca_hw_info.lot_id;
    info->otp_rev = deca_hw_info.otp_rev;
    info->xtal_trim = deca_hw_info.xtal_trim;
    info->rx_antd_ch5_prf16 = deca_hw_info.rx_antd_ch5_prf16;
    info->rx_antd_ch5_prf64 = deca_hw_info.rx_antd_ch5_prf64;
    info->rx_antd_ch9_prf16 = deca_hw_info.rx_antd_ch9_prf16;
    info->rx_antd_ch9_prf64 = deca_hw_info.rx_antd_ch9_prf64;
    info->tx_antd_ch5_prf16 = deca_hw_info.tx_antd_ch5_prf16;
    info->tx_antd_ch5_prf64 = deca_hw_info.tx_antd_ch5_prf64;
    info->tx_antd_ch9_prf16 = deca_hw_info.tx_antd_ch9_prf16;
    info->tx_antd_ch9_prf64 = deca_hw_info.tx_antd_ch9_prf64;
    info->tx_power_ch5_prf16 = deca_hw_info.tx_power_ch5_prf16;
    info->tx_power_ch5_prf64 = deca_hw_info.tx_power_ch5_prf64;
    info->tx_power_ch9_prf16 = deca_hw_info.tx_power_ch9_prf16;
    info->tx_power_ch9_prf64 = deca_hw_info.tx_antd_ch9_prf64;

    return PORT_SUCCESS;
}


bool deca_init_check (void)
{
    return deca_init_done;
}