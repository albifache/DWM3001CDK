/*! ----------------------------------------------------------------------------
 * @file        led_gpio.c
 * @author      Alberto Facheris
 */


#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include "port.h"


#define LED_GPIO_DEV_NODE                   DT_NODELABEL(gpio0)

#define LED_0_GPIO_PIN                      4       // Green LED
#define LED_1_GPIO_PIN                      14      // Red LED
#define LED_2_GPIO_PIN                      22      // Red LED
#define LED_3_GPIO_PIN                      5       // Blue LED


static const struct device *led_gpio_dev;
static volatile bool led_state[NUM_LEDS];


// Initialize LEDs 
int led_gpio_init (void)
{
    // Initialize LEDs
    led_gpio_dev = DEVICE_DT_GET(LED_GPIO_DEV_NODE);
    if (!device_is_ready(led_gpio_dev))
    {
        return PORT_INIT_ERROR;
    }

    // Configure LED 0
    if (gpio_pin_configure(led_gpio_dev, LED_0_GPIO_PIN, GPIO_OUTPUT) != 0)
    {
        return PORT_INIT_ERROR;
    }
    
    // Configure LED 1
    if (gpio_pin_configure(led_gpio_dev, LED_1_GPIO_PIN, GPIO_OUTPUT) != 0)
    {
        return PORT_INIT_ERROR;
    }

    // Configure LED 2
    if (gpio_pin_configure(led_gpio_dev, LED_2_GPIO_PIN, GPIO_OUTPUT) != 0)
    {
        return PORT_INIT_ERROR;
    }

    // Configure LED 3
    if (gpio_pin_configure(led_gpio_dev, LED_3_GPIO_PIN, GPIO_OUTPUT) != 0)
    {
        return PORT_INIT_ERROR;
    }

    // Switch off LEDs
    for (int k = 0; k < NUM_LEDS; k++)
    {
        led_state[k] = false;
        led_gpio_write(k, led_state[k]);
    }
    
    return PORT_SUCCESS;
}


// Set or reset LED
int led_gpio_write (uint8_t led_id, bool state)
{
    int ret;

    led_state[led_id] = state;

    switch (led_id)
    {
        case 0:
            ret = gpio_pin_set(led_gpio_dev, LED_0_GPIO_PIN, !state);
            if (ret != 0)
            {
                return PORT_RUN_ERROR;
            }
        break;

        case 1:
            ret = gpio_pin_set(led_gpio_dev, LED_1_GPIO_PIN, !state);
            if (ret != 0)
            {
                return PORT_RUN_ERROR;
            }
        break;

        case 2:
            ret = gpio_pin_set(led_gpio_dev, LED_2_GPIO_PIN, !state);
            if (ret != 0)
            {
                return PORT_RUN_ERROR;
            }
        break;

        case 3:
            ret = gpio_pin_set(led_gpio_dev, LED_3_GPIO_PIN, !state);
            if (ret != 0)
            {
                return PORT_RUN_ERROR;
            }
        break;

        default:
            return PORT_RUN_ERROR;
        break;
    }

    return PORT_SUCCESS;
}


// Read LED state
int led_gpio_read (uint8_t led_id)
{
    // Check if LED ID is valid
    if (led_id >= NUM_LEDS)
    {
        return PORT_RUN_ERROR;
    }

    return led_state[led_id];
}