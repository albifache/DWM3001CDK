/*! ----------------------------------------------------------------------------
 * @file        main.c
 * @author      Alberto Facheris
 */


#include <stdint.h>
#include <stdbool.h>
#include "../../app/app.h"
#include "../../mac/mac.h"
#include "../../phy/phy.h"
#include "../../port/port.h"
#include <stdio.h>
#include <zephyr/sys/printk.h>


#define RF_CHAN                                 5                           // UWB channel (5 or 9)
#define PREAMBLE_CODE                           9                           // Preamble code
#define BIT_RATE                                DWT_BR_850K                 // UWB bit rate
#define PREAMBLE_LEN                            DWT_PLEN_1024               // Preamble length (number of symbols)
#define STS_LEN                                 DWT_STS_LEN_1024            // STS length (number of symbols)

#define MAC_ADDR                                0x00u                       // MAC address of current node
#define PAN_ID                                  0x00u                       // PAN ID of current network

#define STS_KEY_0                               0ul                         // STS key (bits 0-31)
#define STS_KEY_1                               0ul                         // STS key (bits 32-63)
#define STS_KEY_2                               0ul                         // STS key (bits 64-95)
#define STS_KEY_3                               0ul                         // STS key (bits 96-127)

#define AES_KEY_0                               0ul                         // AES key (bits 0-31)
#define AES_KEY_1                               0ul                         // AES key (bits 32-63)
#define AES_KEY_2                               0ul                         // AES key (bits 64-95)
#define AES_KEY_3                               0ul                         // AES key (bits 96-127)

#define NUM_ANCHORS                             2                           // Number of anchors (max 4)

#define ANCHOR_MAC_ADDR_0                       0x06                        // MAC address of anchor 0
#define ANCHOR_MAC_ADDR_1                       0x07                        // MAC address of anchor 1
#define ANCHOR_MAC_ADDR_2                       0x00                        // MAC address of anchor 2 (unused in this case)
#define ANCHOR_MAC_ADDR_3                       0x00                        // MAC address of anchor 3 (unused in this case)

#define GREEN_LED                               0                           // LED that blinks when tag is too close to anchor 0
#define RED_LED_0                               1                           // LED that blinks when tag is too close to anchor 1
#define RED_LED_1                               2                           // LED that blinks when tag is too close to anchor 2
#define BLUE_LED                                3                           // LED that blinks when tag is too close to anchor 3

#define ALERT_DIST                              1.0f                        // Alert distance (m)

#define GUARD_TIME                              10                          // Guard time (ms)
#define SLEEP_TIME                              100                         // Sleeping time (ms)
#define BLINK_TIME                              100                         // LED blinking time (ms)

#define LIGHT_SPEED                             2.99702547e8f               // Speed of light (m/s)


static phy_init_obj_t phy_init_obj;

static app_init_obj_t app_init_obj;
static app_ctrl_obj_t app_ctrl_obj;
static app_log_info_t app_log_info;

static uint8_t led_id[NUM_LEDS];
static float dist[NUM_LEDS];


int main (void)
{
    int ret;

    // Initialize LEDs
    ret = led_gpio_init();
    if (ret != PORT_SUCCESS)
    {
        printk("\n[ERROR] LEDs initialization failed.\n");
        while(1);
    }
    
    // Initialize DW3000 IC
    ret = deca_init();
    if (ret != PORT_SUCCESS)
    {
        printk("\n[ERROR] DW3000 initialization failed.\n");
        while(1);
    }

    // Set PHY parameters
    phy_init_obj.rf_chan = RF_CHAN;
    phy_init_obj.preamble_code = PREAMBLE_CODE;
    phy_init_obj.preamble_len = PREAMBLE_LEN;
    phy_init_obj.bit_rate = BIT_RATE;
    phy_init_obj.sts_len = STS_LEN;

    // Initialize PHY
    ret = phy_init(&phy_init_obj);
    if (ret != PHY_SUCCESS)
    {
        printk("\n[ERROR] DW3000 configuration failed.\n");
        while(1);
    }

    // Set MAC address
    app_init_obj.mac_addr = MAC_ADDR;

    // Set PAN ID
    app_init_obj.pan_id = PAN_ID;

    // Set STS key (128 bits)
    app_init_obj.sts_key.key0 = STS_KEY_0;
    app_init_obj.sts_key.key1 = STS_KEY_1;
    app_init_obj.sts_key.key2 = STS_KEY_2;
    app_init_obj.sts_key.key3 = STS_KEY_3;

    // Set AES key (128 bits)
    app_init_obj.aes_key.key0 = AES_KEY_0;
    app_init_obj.aes_key.key1 = AES_KEY_1;
    app_init_obj.aes_key.key2 = AES_KEY_2;
    app_init_obj.aes_key.key3 = AES_KEY_3;
    app_init_obj.aes_key.key4 = 0;
    app_init_obj.aes_key.key5 = 0;
    app_init_obj.aes_key.key6 = 0;
    app_init_obj.aes_key.key7 = 0;

    // Initialize app
    ret = app_init(&app_init_obj);
    if (ret != APP_SUCCESS)
    {
        printk("\n[ERROR] App initialization failed.\n");
        while(1);
    }

    // Set tag MAC address
    app_ctrl_obj.tag_mac_addr = PAN_COORDINATOR_MAC_ADDR;

    // Set number of anchors
    app_ctrl_obj.num_anchors = NUM_ANCHORS;

    // Anchors MAC addresses
    app_ctrl_obj.anchor_mac_addr[0] = ANCHOR_MAC_ADDR_0;
    app_ctrl_obj.anchor_mac_addr[1] = ANCHOR_MAC_ADDR_1;
    app_ctrl_obj.anchor_mac_addr[2] = ANCHOR_MAC_ADDR_2;
    app_ctrl_obj.anchor_mac_addr[3] = ANCHOR_MAC_ADDR_3;

    // Check if number of anchors is valid
    if (app_ctrl_obj.num_anchors > NUM_LEDS)
    {
        printk("\n[ERROR] Total number of anchors should not exceed %d.\n", NUM_LEDS);
        while(1);
    }

    // Set app runtime parameters
    ret = app_set_ctrl_params(&app_ctrl_obj);
    if (ret != APP_SUCCESS)
    {
        printk("\n[ERROR] App configuration settings are not valid.\n");
        while(1);
    }

    // Set LEDs ID table
    led_id[0] = BLUE_LED;
    led_id[1] = RED_LED_0;
    led_id[2] = RED_LED_1;
    led_id[3] = GREEN_LED;

    // Switch off LEDs as default
    for (int k = 0; k < NUM_LEDS; k++)
    {
        led_gpio_write(led_id[k], false);
    }

    // Begin loop
    while (1)
    {
        #if (MAC_ADDR == PAN_COORDINATOR_MAC_ADDR)
        
        // Wait a bit before proceeding to make sure all the anchors have receiver switched on
        app_sleep(GUARD_TIME);

        #endif
        
        // Run ranging session
        ret = app_run_ieee_802_15_4z_schedule();
        if (ret != APP_SUCCESS)
        {
            continue;
        }

        // Read ranging session logs
        app_read_log_info(&app_log_info);
        
        // Compute distances (m)
        for (int k = 0; k < NUM_LEDS; k++)
        {
            dist[k] = LIGHT_SPEED * DWT_TIME_UNITS * app_log_info.dist[k];
        }

        // Switch on LEDs
        for (int k = 0; k < NUM_LEDS; k++)
        {
            if (dist[k] < ALERT_DIST &&
                dist[k] > 0 &&
                app_init_obj.mac_addr == PAN_COORDINATOR_MAC_ADDR)
            {
                led_gpio_write(led_id[k], true);
            }
        }
        
        // Sleep while LEDs are ON
        app_sleep(BLINK_TIME);

        // Switch off LEDs
        for (int k = 0; k < NUM_LEDS; k++)
        {
            if (dist[k] < ALERT_DIST &&
                dist[k] > 0 &&
                app_init_obj.mac_addr == PAN_COORDINATOR_MAC_ADDR)
            {
                led_gpio_write(led_id[k], false);
            }
        }
        
        // Sleep while LEDs are OFF
        app_sleep(SLEEP_TIME);
    }
}