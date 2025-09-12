/*! ----------------------------------------------------------------------------
 * @file        deca_gpio.c
 * @author      Alberto Facheris
 */


#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include "port.h"


#define DW_RST_GPIO_DEV_NODE        DT_NODELABEL(gpio0)
#define DW_RST_GPIO_PIN             25


static const struct device *rst_gpio_dev;


int deca_gpio_init(void)
{
    rst_gpio_dev = DEVICE_DT_GET(DW_RST_GPIO_DEV_NODE);
    
    // Initialize RST pin
    if (!device_is_ready(rst_gpio_dev))
    {
        return PORT_INIT_ERROR;
    }

    // Configure RST pin as open-drain output
    if (gpio_pin_configure(rst_gpio_dev, DW_RST_GPIO_PIN, GPIO_OUTPUT | GPIO_OPEN_DRAIN) != 0)
    {
        return PORT_CONFIG_ERROR;
    }
    
    return PORT_SUCCESS;
}


void deca_wakeup_device_with_io(void)
{
    return;
}


void deca_reset_ic(void)
{
    // Pull RST pin low
    gpio_pin_set(rst_gpio_dev, DW_RST_GPIO_PIN, 0);
    k_msleep(5);
    
    // Relrease RST pin
    gpio_pin_set(rst_gpio_dev, DW_RST_GPIO_PIN, 1);
    k_msleep(5);
}