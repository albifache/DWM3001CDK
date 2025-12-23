/*! ----------------------------------------------------------------------------
 * @file        deca_spi.c
 * @author      Alberto Facheris
 */


#include "port.h"
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>


#define DW_SPI_DEV_NODE             DT_NODELABEL(spi3)
#define DW_CS_GPIO_DEV_NODE         DT_NODELABEL(gpio1)
#define DW_CS_GPIO_PIN              6

#define WAKEUP_TIME                 200
#define STARTUP_TIME                2000


static const struct device *spi_dev;
static const struct device *cs_gpio_dev;

static struct spi_config spi_cfg_slow;
static struct spi_config spi_cfg_fast;
static struct spi_config *current_spi_cfg;


int16_t deca_spi_init (void)
{
    // Get SPI device
    spi_dev = DEVICE_DT_GET(DW_SPI_DEV_NODE);
    if (!device_is_ready(spi_dev))
    {
        return PORT_INIT_ERROR;
    }

    // Get CS GPIO device
    cs_gpio_dev = DEVICE_DT_GET(DW_CS_GPIO_DEV_NODE);
    if (!device_is_ready(cs_gpio_dev))
    {
        return PORT_INIT_ERROR;
    }

    // Configure CS pin as output high (inactive)
    if (gpio_pin_configure(cs_gpio_dev, DW_CS_GPIO_PIN, GPIO_OUTPUT) != 0)
    {
        return PORT_INIT_ERROR;
    }

    // Set CS high (inactive)
    gpio_pin_set(cs_gpio_dev, DW_CS_GPIO_PIN, true);

    // Configure slow SPI (2MHz)
    spi_cfg_slow.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB;
    spi_cfg_slow.frequency = 2000000;

    // Configure fast SPI (8MHz)
    spi_cfg_fast.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB;
    spi_cfg_fast.frequency = 8000000;

    // Start with fast SPI configuration
    current_spi_cfg = &spi_cfg_fast;

    return PORT_SUCCESS;
}


void deca_set_spi_slow_rate (void)
{
    current_spi_cfg = &spi_cfg_slow;
}


void deca_set_spi_fast_rate (void)
{
    current_spi_cfg = &spi_cfg_fast;

    return;
}


int32_t deca_write_to_spi_with_crc (uint16_t header_len, const uint8_t header[], uint16_t data_len, const uint8_t data[], uint8_t crc8)
{
    struct spi_buf tx_bufs[3];
    struct spi_buf_set tx_buf_set;

    // Header
    tx_bufs[0].buf = (void*)header;
    tx_bufs[0].len = header_len;

    // Data
    tx_bufs[1].buf = (void*)data;
    tx_bufs[1].len = data_len;

    // CRC
    tx_bufs[2].buf = &crc8;
    tx_bufs[2].len = 1;

    tx_buf_set.buffers = tx_bufs;
    tx_buf_set.count = 3;

    // Perform SPI transaction
    gpio_pin_set(cs_gpio_dev, DW_CS_GPIO_PIN, false);
    // k_busy_wait(1);
    spi_write(spi_dev, current_spi_cfg, &tx_buf_set);
    // k_busy_wait(1);
    gpio_pin_set(cs_gpio_dev, DW_CS_GPIO_PIN, true);

    return PORT_SUCCESS;
}


int32_t deca_write_to_spi (uint16_t header_len, const uint8_t header[], uint16_t data_len, const uint8_t data[])
{
    struct spi_buf tx_bufs[2];
    struct spi_buf_set tx_buf_set;

    // Header
    tx_bufs[0].buf = (void*)header;
    tx_bufs[0].len = header_len;

    // Data
    tx_bufs[1].buf = (void*)data;
    tx_bufs[1].len = data_len;

    tx_buf_set.buffers = tx_bufs;
    tx_buf_set.count = 2;

    // Perform SPI transaction
    gpio_pin_set(cs_gpio_dev, DW_CS_GPIO_PIN, false);
    // k_busy_wait(1);
    spi_write(spi_dev, current_spi_cfg, &tx_buf_set);
    // k_busy_wait(1);
    gpio_pin_set(cs_gpio_dev, DW_CS_GPIO_PIN, true);

    return PORT_SUCCESS;
}


int32_t deca_read_from_spi (uint16_t header_len, uint8_t header[], uint16_t data_len, uint8_t data[])
{
    struct spi_buf tx_bufs[1];
    struct spi_buf_set tx_buf_set;
    struct spi_buf rx_bufs[2];
    struct spi_buf_set rx_buf_set;

    // Header (TX buffer)
    tx_bufs[0].buf = header;
    tx_bufs[0].len = header_len;

    // Header (RX buffer)
    rx_bufs[0].buf = NULL;
    rx_bufs[0].len = header_len;

    // Data (RX buffer)
    rx_bufs[1].buf = data;
    rx_bufs[1].len = data_len;

    tx_buf_set.buffers = tx_bufs;
    tx_buf_set.count = 1;
    rx_buf_set.buffers = rx_bufs;
    rx_buf_set.count = 2;

    // Perform SPI transaction
    gpio_pin_set(cs_gpio_dev, DW_CS_GPIO_PIN, false);
    // k_busy_wait(1);
    spi_transceive(spi_dev, current_spi_cfg, &tx_buf_set, &rx_buf_set);
    // k_busy_wait(1);
    gpio_pin_set(cs_gpio_dev, DW_CS_GPIO_PIN, true);

    return PORT_SUCCESS;
}


void deca_wakeup_device_with_io (void)
{
    gpio_pin_set(cs_gpio_dev, DW_CS_GPIO_PIN, false);
    k_usleep(WAKEUP_TIME);
    gpio_pin_set(cs_gpio_dev, DW_CS_GPIO_PIN, true);
    k_usleep(STARTUP_TIME);

    return;
}