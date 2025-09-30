/*! ----------------------------------------------------------------------------
 * @file    port.h
 * @author  Alberto Facheris
 */


#ifndef PORT_H
#define PORT_H


#include "../deca_driver/deca_device_api.h"


#define NUM_LEDS                        4

#define PORT_SUCCESS                    0
#define PORT_INIT_ERROR                 -1
#define PORT_RUN_ERROR                  -2


typedef struct
{
    uint32_t device_id;
    uint32_t chip_id;
    uint64_t lot_id;
    uint8_t otp_rev;
    uint8_t xtal_trim;
    uint16_t rx_antd_ch5_prf16;
    uint16_t rx_antd_ch5_prf64;
    uint16_t rx_antd_ch9_prf16;
    uint16_t rx_antd_ch9_prf64;
    uint16_t tx_antd_ch5_prf16;
    uint16_t tx_antd_ch5_prf64;
    uint16_t tx_antd_ch9_prf16;
    uint16_t tx_antd_ch9_prf64;
    uint32_t tx_power_ch5_prf16;
    uint32_t tx_power_ch5_prf64;
    uint32_t tx_power_ch9_prf16;
    uint32_t tx_power_ch9_prf64;
}
deca_hw_info_t;


// Initialize the DW3000 RST pin 
int deca_gpio_init (void);


// Declared just for compatibility, does not have any operational effect
void deca_wakeup_device_with_io (void);


// Initialize SPI for DW3000
int deca_spi_init (void);


// Reset DW3000 IC
int deca_reset_ic (void);


// Transmit over SPI (header, data , CRC) with CRC mode enabled
int deca_write_to_spi_with_crc (uint16_t headerLength, const uint8_t *headerBuffer, uint16_t bodyLength, const uint8_t *bodyBuffer, uint8_t crc8);


// Transmit over SPI (header, data) with CRC mode disabled
int deca_write_to_spi (uint16_t headerLength, const uint8_t *headerBuffer, uint16_t bodyLength, const uint8_t *bodyBuffer);


// Trasmit header and receive data over SPI with CRC mode disabled
int deca_read_from_spi (uint16_t headerLength, uint8_t *headerBuffer, uint16_t readLength, uint8_t *readBuffer);


// set SPI data rate to 2 MHz
void deca_set_spi_slow_rate (void);


// set SPI data rate to 8 MHz
void deca_set_spi_fast_rate (void);


// Defined just for compatibility, does not have any effect
decaIrqStatus_t deca_mutex_on(void);


// Defined just for compatibility, does not have any effect
void deca_mutex_off (decaIrqStatus_t status);


// Sleep for time_ms milliseconds
void deca_sleep (unsigned int sleep_time_ms);


// Sleep for time_us microseconds
void deca_usleep (unsigned long sleep_time_us);


// Initialize LEDs
int led_gpio_init (void);


// Set or reset LED
void led_gpio_write (uint8_t led_id, bool state);


// Read LEDs state
bool led_gpio_read (uint8_t led_id);


// Initialize DW3000 IC
int deca_init (void);


// Check if DW3000 IC has been initialized
bool deca_init_check (void);


// Read DW3000 IC hardcoded info
void deca_read_device_info (deca_hw_info_t *info);


#endif